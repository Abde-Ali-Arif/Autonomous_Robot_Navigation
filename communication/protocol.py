"""
Communication Protocol Definition.
Standardizes JSON message structure between Webots Controller and Streamlit Dashboard.
"""

from enum import Enum
import json
from typing import Any, Dict, Optional


class RobotState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    REPLANNING = "REPLANNING"
    PAUSED = "PAUSED"
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"
    GOAL_REACHED = "GOAL_REACHED"
    BLOCKED = "BLOCKED"
    ERROR = "ERROR"


class CommandType(str, Enum):
    START_NAV = "START_NAV"
    PAUSE_NAV = "PAUSE_NAV"
    RESUME_NAV = "RESUME_NAV"
    STOP_NAV = "STOP_NAV"
    RESET = "RESET"
    SET_GOAL = "SET_GOAL"
    SET_START = "SET_START"
    ADD_OBSTACLE = "ADD_OBSTACLE"
    REMOVE_OBSTACLE = "REMOVE_OBSTACLE"
    GET_STATE = "GET_STATE"


def create_command(command: CommandType, payload: Optional[Dict[str, Any]] = None) -> str:
    """Pack a command into a newline-delimited JSON string."""
    msg = {
        "command": command.value if isinstance(command, CommandType) else str(command),
        "payload": payload or {}
    }
    return json.dumps(msg) + "\n"


def parse_message(raw_msg: str) -> Optional[Dict[str, Any]]:
    """Parse received JSON message string."""
    try:
        raw_msg = raw_msg.strip()
        if not raw_msg:
            return None
        return json.loads(raw_msg)
    except json.JSONDecodeError:
        return None
