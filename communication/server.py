"""
IPC Communication Server for Webots Controller.
Supports dual-channel communication:
1. High-speed TCP Socket Server (localhost:5050)
2. Atomic File-based State Exchange (logs/telemetry.json and logs/command.json)

This ensures bulletproof operation across all Windows environments, firewall configurations,
and local permission policies.
"""

import json
import logging
import os
from pathlib import Path
import queue
import select
import socket
import threading
import time
from typing import Any, Dict, List, Optional

from communication.protocol import parse_message
from config.settings import SOCKET_HOST, SOCKET_PORT, SOCKET_BUFFER_SIZE, LOGS_DIR

logger = logging.getLogger("SimulationServer")

TELEMETRY_FILE = LOGS_DIR / "telemetry.json"
COMMAND_FILE = LOGS_DIR / "command.json"


class SimulationServer:
    def __init__(self, host: str = SOCKET_HOST, port: int = SOCKET_PORT):
        self.host = host
        self.port = port
        self.running = False
        self._server_socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

        self._command_queue: queue.Queue = queue.Queue()
        self._state_lock = threading.Lock()
        self._latest_state: Dict[str, Any] = {
            "status": "OFFLINE",
            "connected": True,
            "timestamp": time.time()
        }

    def start(self) -> bool:
        """Start the communication server (TCP socket + file sync)."""
        self.running = True
        # Try to initialize socket server
        try:
            self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server_socket.bind((self.host, self.port))
            self._server_socket.listen(5)
            self._thread = threading.Thread(target=self._run_server, daemon=True)
            self._thread.start()
            logger.info(f"SimulationServer TCP listening on {self.host}:{self.port}")
        except Exception as e:
            logger.warning(f"TCP socket bind skipped ({e}). Operating in resilient File-IPC mode.")

        # Initialize telemetry file
        self._write_telemetry_file(self._latest_state)
        return True

    def stop(self):
        """Stop server and clean up resources."""
        self.running = False
        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)

    def update_state(self, state: Dict[str, Any]):
        """Update telemetry state for both Socket and File IPC."""
        state["timestamp"] = time.time()
        with self._state_lock:
            self._latest_state = state
        self._write_telemetry_file(state)

    def get_pending_commands(self) -> List[Dict[str, Any]]:
        """Fetch all queued commands from both Socket and File channels."""
        cmds = []
        # 1. Socket commands
        while not self._command_queue.empty():
            try:
                cmds.append(self._command_queue.get_nowait())
            except queue.Empty:
                break

        # 2. File commands
        file_cmd = self._read_command_file()
        if file_cmd:
            cmds.append(file_cmd)

        return cmds

    def _write_telemetry_file(self, state: Dict[str, Any]):
        """Atomic write of telemetry to disk."""
        try:
            tmp_file = LOGS_DIR / "telemetry.json.tmp"
            with open(tmp_file, "w", encoding="utf-8") as f:
                json.dump(state, f)
            os.replace(tmp_file, TELEMETRY_FILE)
        except Exception as e:
            logger.debug(f"Telemetry file write error: {e}")

    def _read_command_file(self) -> Optional[Dict[str, Any]]:
        """Read and consume pending command file."""
        if not COMMAND_FILE.exists():
            return None
        try:
            with open(COMMAND_FILE, "r", encoding="utf-8") as f:
                cmd = json.load(f)
            COMMAND_FILE.unlink(missing_ok=True)
            return cmd
        except Exception:
            return None

    def _run_server(self):
        """Background TCP socket listener loop."""
        inputs = [self._server_socket]
        client_buffers: Dict[socket.socket, str] = {}

        while self.running:
            try:
                readable, _, exceptional = select.select(inputs, [], inputs, 0.05)
                for s in readable:
                    if s is self._server_socket:
                        try:
                            client_sock, _ = self._server_socket.accept()
                            client_sock.setblocking(False)
                            inputs.append(client_sock)
                            client_buffers[client_sock] = ""
                        except Exception:
                            pass
                    else:
                        try:
                            data = s.recv(SOCKET_BUFFER_SIZE)
                            if data:
                                client_buffers[s] = client_buffers.get(s, "") + data.decode("utf-8")
                                while "\n" in client_buffers[s]:
                                    line, client_buffers[s] = client_buffers[s].split("\n", 1)
                                    msg = parse_message(line)
                                    if msg:
                                        self._handle_client_message(s, msg)
                            else:
                                self._cleanup_client(s, inputs, client_buffers)
                        except Exception:
                            self._cleanup_client(s, inputs, client_buffers)

                for s in exceptional:
                    self._cleanup_client(s, inputs, client_buffers)
            except Exception:
                if self.running:
                    time.sleep(0.05)

    def _cleanup_client(self, client_sock: socket.socket, inputs: list, client_buffers: dict):
        if client_sock in inputs:
            inputs.remove(client_sock)
        if client_sock in client_buffers:
            del client_buffers[client_sock]
        try:
            client_sock.close()
        except Exception:
            pass

    def _handle_client_message(self, client_sock: socket.socket, msg: Dict[str, Any]):
        cmd = msg.get("command")
        if cmd == "GET_STATE":
            with self._state_lock:
                resp = json.dumps(self._latest_state) + "\n"
            try:
                client_sock.sendall(resp.encode("utf-8"))
            except Exception:
                pass
        else:
            self._command_queue.put(msg)
            try:
                ack = json.dumps({"status": "ACK", "command": cmd}) + "\n"
                client_sock.sendall(ack.encode("utf-8"))
            except Exception:
                pass
