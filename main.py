"""
Master Project Runner for Autonomous Mobile Robot Navigation Simulation.
Can launch Webots Simulation, Streamlit Mission Control Dashboard, or both.
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
WORLDS_DIR = BASE_DIR / "worlds"
DASHBOARD_PATH = BASE_DIR / "dashboard" / "app.py"
WBT_FILE = WORLDS_DIR / "indoor_navigation.wbt"

WEBOTS_PATHS = [
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    r"C:\Program Files\Webots\webots.exe",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots\msys64\mingw64\bin\webots.exe"),
]


def find_webots_executable() -> str:
    """Find Webots executable path on Windows."""
    for path in WEBOTS_PATHS:
        if os.path.exists(path):
            return path
    # Try PATH
    import shutil
    webots_path = shutil.which("webots")
    if webots_path:
        return webots_path
    return ""


def launch_webots():
    """Launch Webots 3D simulation with indoor_navigation.wbt world."""
    webots_bin = find_webots_executable()
    if not webots_bin:
        print("[ERROR] Webots executable not found!")
        print("Please ensure Webots is installed at 'C:\\Program Files\\Webots'")
        return None

    if not WBT_FILE.exists():
        print(f"[ERROR] World file not found: {WBT_FILE}")
        return None

    print(f"[LAUNCHER] Starting Webots simulation: {WBT_FILE.name}")
    # Prepare environment with WEBOTS_HOME
    env = os.environ.copy()
    env["WEBOTS_HOME"] = str(Path(webots_bin).parents[3]) if "msys64" in webots_bin else str(Path(webots_bin).parent)

    proc = subprocess.Popen([webots_bin, str(WBT_FILE)], env=env)
    return proc


def launch_dashboard():
    """Launch Streamlit mission control station."""
    print("[LAUNCHER] Starting Streamlit Mission Control Station...")
    python_bin = sys.executable
    cmd = [python_bin, "-m", "streamlit", "run", str(DASHBOARD_PATH), "--server.headless=false"]
    proc = subprocess.Popen(cmd)
    return proc


def main():
    parser = argparse.ArgumentParser(description="Autonomous Mobile Robot Simulation Runner")
    parser.add_argument("--mode", choices=["all", "webots", "dashboard"], default="all",
                        help="Launch mode: 'all' (default), 'webots', or 'dashboard'")
    args = parser.parse_args()

    print("=" * 70)
    print("  AUTONOMOUS MOBILE ROBOT NAVIGATION & DYNAMIC AVOIDANCE")
    print("  B.Tech Major Engineering Project • Webots 3D & Streamlit")
    print("=" * 70)

    webots_proc = None
    dash_proc = None

    if args.mode in ["all", "webots"]:
        webots_proc = launch_webots()
        if args.mode == "all":
            time.sleep(2.0)  # Brief delay to allow Webots to initialize

    if args.mode in ["all", "dashboard"]:
        dash_proc = launch_dashboard()

    print("\n[LAUNCHER] Applications started successfully.")
    print("[LAUNCHER] Streamlit Dashboard will open in your browser at: http://localhost:8501")
    print("[LAUNCHER] Press Ctrl+C in this terminal to stop all processes.\n")

    try:
        while True:
            time.sleep(1.0)
            if webots_proc and webots_proc.poll() is not None:
                print("[LAUNCHER] Webots simulation closed.")
                break
            if dash_proc and dash_proc.poll() is not None:
                print("[LAUNCHER] Dashboard closed.")
                break
    except KeyboardInterrupt:
        print("\n[LAUNCHER] Terminating processes...")
    finally:
        if webots_proc and webots_proc.poll() is None:
            webots_proc.terminate()
        if dash_proc and dash_proc.poll() is None:
            dash_proc.terminate()
        print("[LAUNCHER] All processes stopped.")


if __name__ == "__main__":
    main()
