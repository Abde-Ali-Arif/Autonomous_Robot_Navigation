"""
Robot State and Telemetry Tracker.
Maintains continuous odometry, total traveled distance, mission timing,
and real-time navigation metrics.
"""

import math
import time
from typing import Dict, Any, Tuple, Optional

from communication.protocol import RobotState
from config.settings import DEFAULT_START_POS, DEFAULT_GOAL_POS


class TelemetryTracker:
    def __init__(self,
                 start_x: float = DEFAULT_START_POS[0],
                 start_y: float = DEFAULT_START_POS[1],
                 goal_x: float = DEFAULT_GOAL_POS[0],
                 goal_y: float = DEFAULT_GOAL_POS[1]):
        self.x: float = start_x
        self.y: float = start_y
        self.theta: float = 0.0  # Radians

        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0
        self.left_wheel_speed: float = 0.0
        self.right_wheel_speed: float = 0.0

        self.start_pos: Tuple[float, float] = (start_x, start_y)
        self.goal_pos: Tuple[float, float] = (goal_x, goal_y)

        self.state: RobotState = RobotState.IDLE
        self.status_message: str = "Ready. Waiting for destination selection."

        # Navigation Metrics
        self.total_distance_travelled: float = 0.0
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None

        self.start_time: Optional[float] = None
        self.elapsed_time: float = 0.0
        self.navigation_completed: bool = False

    def update_pose(self, x: float, y: float, theta: float):
        """Update robot pose and accumulate traveled distance."""
        self.x = round(x, 3)
        self.y = round(y, 3)
        self.theta = round(theta, 3)

        if self.prev_x is not None and self.prev_y is not None:
            # Only accumulate if active navigation
            if self.state in [RobotState.NAVIGATING, RobotState.REPLANNING]:
                step_dist = math.hypot(x - self.prev_x, y - self.prev_y)
                if step_dist < 0.2:  # Filter teleport/reset jumps
                    self.total_distance_travelled += step_dist

        self.prev_x = x
        self.prev_y = y

        # Track elapsed time
        if self.start_time is not None and not self.navigation_completed:
            self.elapsed_time = round(time.time() - self.start_time, 1)

    def start_mission(self):
        """Mark start of autonomous navigation run."""
        self.start_time = time.time()
        self.navigation_completed = False
        self.total_distance_travelled = 0.0
        self.state = RobotState.NAVIGATING
        self.status_message = "Navigating along calculated A* route."

    def complete_mission(self):
        """Mark goal reached."""
        self.navigation_completed = True
        self.state = RobotState.GOAL_REACHED
        self.status_message = "Mission Completed! Reached target destination."

    def to_dict(self) -> Dict[str, Any]:
        """Serialize telemetry for IPC broadcast."""
        dist_to_goal = math.hypot(self.goal_pos[0] - self.x, self.goal_pos[1] - self.y)
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "theta_deg": round(math.degrees(self.theta), 1),
            "linear_velocity": round(self.linear_velocity, 3),
            "angular_velocity": round(self.angular_velocity, 3),
            "left_wheel_speed": round(self.left_wheel_speed, 3),
            "right_wheel_speed": round(self.right_wheel_speed, 3),
            "distance_to_goal": round(dist_to_goal, 3),
            "distance_travelled": round(self.total_distance_travelled, 2),
            "elapsed_time": self.elapsed_time,
            "state": self.state.value if isinstance(self.state, RobotState) else str(self.state),
            "status_message": self.status_message,
            "start_pos": list(self.start_pos),
            "goal_pos": list(self.goal_pos)
        }
