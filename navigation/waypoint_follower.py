"""
Waypoint Following and Differential Drive Kinematic Controller.
Computes wheel velocities (left & right) using proportional heading control,
smooth turn-in-place transitions, and destination tolerance checks.
"""

import math
from typing import List, Tuple, Dict, Any, Optional

from config.settings import (
    WHEEL_RADIUS, AXLE_LENGTH, MAX_WHEEL_SPEED,
    DEFAULT_CRUISE_SPEED, SLOW_CRUISE_SPEED,
    KP_HEADING, HEADING_TOLERANCE, WAYPOINT_TOLERANCE, GOAL_TOLERANCE
)


class WaypointFollower:
    def __init__(self,
                 wheel_radius: float = WHEEL_RADIUS,
                 axle_length: float = AXLE_LENGTH,
                 max_speed: float = MAX_WHEEL_SPEED,
                 kp: float = KP_HEADING):
        self.r = wheel_radius
        self.L = axle_length
        self.max_speed = max_speed
        self.kp = kp

        self.waypoints: List[Tuple[float, float]] = []
        self.current_idx: int = 0
        self.goal_reached: bool = False

    def set_path(self, waypoints: List[Tuple[float, float]]):
        """Set new path waypoints to track."""
        self.waypoints = waypoints
        self.current_idx = 0
        self.goal_reached = False

    def get_remaining_waypoints(self) -> List[Tuple[float, float]]:
        """Return path from current target to final destination."""
        if not self.waypoints or self.current_idx >= len(self.waypoints):
            return []
        return self.waypoints[self.current_idx:]

    def compute_velocities(self,
                           robot_x: float,
                           robot_y: float,
                           robot_theta: float,
                           slow_down: bool = False) -> Dict[str, Any]:
        """
        Compute required left and right motor speeds (rad/s) and navigation status.
        """
        if not self.waypoints or self.current_idx >= len(self.waypoints):
            self.goal_reached = True
            return {
                "left_speed": 0.0,
                "right_speed": 0.0,
                "linear_vel": 0.0,
                "angular_vel": 0.0,
                "goal_reached": True,
                "target_waypoint": None,
                "dist_to_goal": 0.0
            }

        final_wp = self.waypoints[-1]
        dist_to_goal = math.hypot(final_wp[0] - robot_x, final_wp[1] - robot_y)

        # 1. Global Goal Arrival Check
        if dist_to_goal <= GOAL_TOLERANCE:
            self.goal_reached = True
            return {
                "left_speed": 0.0,
                "right_speed": 0.0,
                "linear_vel": 0.0,
                "angular_vel": 0.0,
                "goal_reached": True,
                "target_waypoint": final_wp,
                "dist_to_goal": round(dist_to_goal, 3)
            }

        # 2. Advance through any already reached intermediate waypoints
        while self.current_idx < len(self.waypoints) - 1:
            curr_wp = self.waypoints[self.current_idx]
            dist_to_curr = math.hypot(curr_wp[0] - robot_x, curr_wp[1] - robot_y)
            if dist_to_curr <= WAYPOINT_TOLERANCE:
                self.current_idx += 1
            else:
                break

        target_wp = self.waypoints[self.current_idx]

        # 3. Heading error calculation
        target_heading = math.atan2(target_wp[1] - robot_y, target_wp[0] - robot_x)
        heading_err = target_heading - robot_theta
        # Normalize to [-pi, pi]
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # Speed scaling based on conditions
        nominal_wheel_speed = SLOW_CRUISE_SPEED if slow_down else DEFAULT_CRUISE_SPEED
        v_linear_target = nominal_wheel_speed * self.r  # Linear velocity m/s

        # Control Law:
        if abs(heading_err) > 0.45:  # ~25 degrees error: turn predominantly in place
            v_lin = 0.04 if abs(heading_err) < 0.8 else 0.0
            omega = self.kp * heading_err
        else:
            # Linear speed is scaled down smoothly if slightly misaligned
            v_lin = v_linear_target * math.cos(heading_err)
            omega = self.kp * heading_err

        # Clamp angular velocity
        max_omega = (self.max_speed * self.r) / (self.L / 2.0)
        omega = max(-max_omega, min(max_omega, omega))

        # Differential Drive Inverse Kinematics
        w_left = (v_lin - (omega * self.L / 2.0)) / self.r
        w_right = (v_lin + (omega * self.L / 2.0)) / self.r

        # Speed normalization if exceeding maximum allowable wheel velocity
        max_w = max(abs(w_left), abs(w_right))
        if max_w > self.max_speed:
            scale = self.max_speed / max_w
            w_left *= scale
            w_right *= scale

        return {
            "left_speed": round(w_left, 3),
            "right_speed": round(w_right, 3),
            "linear_vel": round(v_lin, 3),
            "angular_vel": round(omega, 3),
            "goal_reached": False,
            "target_waypoint": target_wp,
            "dist_to_goal": round(dist_to_goal, 3)
        }
