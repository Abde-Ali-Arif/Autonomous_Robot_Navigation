"""
IPC Communication Client for Streamlit Dashboard.
Supports dual-channel communication:
1. High-speed TCP Socket Client (localhost:5050)
2. Atomic File-based State Exchange (logs/telemetry.json and logs/command.json)

Provides effortless fallback to File-IPC whenever TCP loopback encounters OS permission,
firewall, or sandbox restrictions.
"""

import json
import logging
import os
from pathlib import Path
import socket
import time
from typing import Any, Dict, Optional

from communication.protocol import CommandType, create_command, parse_message
from config.settings import SOCKET_HOST, SOCKET_PORT, SOCKET_TIMEOUT, LOGS_DIR

logger = logging.getLogger("SimulationClient")

TELEMETRY_FILE = LOGS_DIR / "telemetry.json"
COMMAND_FILE = LOGS_DIR / "command.json"


class SimulationClient:
    def __init__(self, host: str = SOCKET_HOST, port: int = SOCKET_PORT, timeout: float = SOCKET_TIMEOUT):
        self.host = host
        self.port = port
        self.timeout = timeout

    def _create_connection(self) -> Optional[socket.socket]:
        """Establish a short-lived TCP connection."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            return sock
        except Exception:
            return None

    def is_connected(self) -> bool:
        """Check if Webots simulation is actively reporting state."""
        # Check TCP socket first
        sock = self._create_connection()
        if sock:
            try:
                sock.close()
            except Exception:
                pass
            return True

        # Check telemetry file freshness (updated within last 5 seconds)
        if TELEMETRY_FILE.exists():
            try:
                mtime = os.path.getmtime(TELEMETRY_FILE)
                if (time.time() - mtime) < 5.0:
                    return True
            except Exception:
                pass

        return False

    def get_state(self) -> Optional[Dict[str, Any]]:
        """Query current simulation telemetry and navigation state."""
        # 1. Try TCP Socket
        sock = self._create_connection()
        if sock:
            try:
                req = create_command(CommandType.GET_STATE)
                sock.sendall(req.encode("utf-8"))
                data = b""
                while b"\n" not in data:
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    data += chunk
                sock.close()
                if data:
                    line = data.decode("utf-8").split("\n")[0]
                    return parse_message(line)
            except Exception:
                try:
                    sock.close()
                except Exception:
                    pass

        # 2. Resilient Fallback to File-IPC (only if file was updated recently, < 4.0s)
        if TELEMETRY_FILE.exists():
            try:
                mtime = os.path.getmtime(TELEMETRY_FILE)
                if (time.time() - mtime) < 4.0:
                    with open(TELEMETRY_FILE, "r", encoding="utf-8") as f:
                        return json.load(f)
            except Exception:
                return None

        return None

    def send_command(self, command: CommandType, payload: Optional[Dict[str, Any]] = None) -> bool:
        """Send a navigation or environment control command to the simulation."""
        sent_socket = False

        # 1. Try TCP Socket
        sock = self._create_connection()
        if sock:
            try:
                req = create_command(command, payload)
                sock.sendall(req.encode("utf-8"))
                data = sock.recv(4096)
                sock.close()
                if data:
                    res = parse_message(data.decode("utf-8"))
                    if res and res.get("status") == "ACK":
                        sent_socket = True
            except Exception:
                try:
                    sock.close()
                except Exception:
                    pass

        # 2. Write to Command File (guarantees delivery if socket failed or wasn't used)
        if not sent_socket:
            try:
                cmd_dict = {
                    "command": command.value if isinstance(command, CommandType) else str(command),
                    "payload": payload or {},
                    "timestamp": time.time()
                }
                # Read existing pending commands so rapid clicks append rather than overwrite
                existing = []
                if COMMAND_FILE.exists():
                    try:
                        with open(COMMAND_FILE, "r", encoding="utf-8") as f:
                            c = json.load(f)
                            if isinstance(c, list):
                                existing = c
                            elif isinstance(c, dict):
                                existing = [c]
                    except Exception:
                        existing = []
                existing.append(cmd_dict)

                tmp_file = LOGS_DIR / "command.json.tmp"
                with open(tmp_file, "w", encoding="utf-8") as f:
                    json.dump(existing, f)
                os.replace(tmp_file, COMMAND_FILE)
                return True
            except Exception as e:
                logger.error(f"Failed to write command file: {e}")
                return False

        return True
