"""
LiDAR Obstacle Detection and Safety Sector Analysis.
Processes 360-degree range scans, calculates front-sector clearance,
detects sudden dynamic obstructions, and determines if planned route is blocked.
"""

import math
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from config.settings import (
    LIDAR_BEAMS, LIDAR_MIN_RANGE, LIDAR_MAX_RANGE,
    EMERGENCY_STOP_DISTANCE, SLOWDOWN_DISTANCE, OBSTACLE_DETECTION_FRONT_ARC,
    ROBOT_RADIUS
)


class ObstacleDetector:
    def __init__(self,
                 front_arc_deg: float = OBSTACLE_DETECTION_FRONT_ARC,
                 stop_dist: float = EMERGENCY_STOP_DISTANCE,
                 slow_dist: float = SLOWDOWN_DISTANCE):
        self.front_arc_rad = math.radians(front_arc_deg)
        self.stop_dist = stop_dist
        self.slow_dist = slow_dist
        self.min_range = LIDAR_MIN_RANGE
        self.max_range = LIDAR_MAX_RANGE

    def process_scan(self,
                     ranges: List[float],
                     robot_x: float,
                     robot_y: float,
                     robot_theta: float) -> Dict[str, Any]:
        """
        Process raw 360-degree LiDAR range measurements.
        Returns:
            - min_front_dist: Closest object within front cone
            - emergency_stop: True if collision imminent
            - slow_down: True if approaching obstacle
            - detected_points: List of world (x, y) coordinates of obstacles
            - front_points: Points directly ahead
        """
        if not ranges or len(ranges) == 0:
            return {
                "min_front_dist": float("inf"),
                "emergency_stop": False,
                "slow_down": False,
                "detected_points": [],
                "front_points": []
            }

        num_beams = len(ranges)
        angle_step = (2.0 * math.pi) / num_beams

        detected_points = []
        front_points = []
        min_front_dist = float("inf")

        half_front_arc = self.front_arc_rad / 2.0

        for i, dist in enumerate(ranges):
            # Filter non-valid range readings
            if math.isnan(dist) or math.isinf(dist) or dist < self.min_range or dist > self.max_range:
                continue

            # In Webots Lidar, beams are uniformly distributed across FOV [-pi, pi]
            beam_angle_rel = -math.pi + i * angle_step
            # Normalize to [-pi, pi]
            beam_angle_rel = (beam_angle_rel + math.pi) % (2.0 * math.pi) - math.pi

            # Global beam angle
            beam_angle_global = robot_theta + beam_angle_rel

            # World obstacle position
            obs_x = robot_x + dist * math.cos(beam_angle_global)
            obs_y = robot_y + dist * math.sin(beam_angle_global)
            point = (round(obs_x, 3), round(obs_y, 3))
            detected_points.append(point)

            # Check if beam lies within front sector
            if abs(beam_angle_rel) <= half_front_arc:
                if dist < min_front_dist:
                    min_front_dist = dist
                front_points.append(point)

        emergency_stop = (min_front_dist <= self.stop_dist)
        slow_down = (min_front_dist <= self.slow_dist)

        return {
            "min_front_dist": round(min_front_dist, 3),
            "emergency_stop": emergency_stop,
            "slow_down": slow_down,
            "detected_points": detected_points,
            "front_points": front_points
        }

    def is_route_blocked(self,
                         front_points: List[Tuple[float, float]],
                         remaining_waypoints: List[Tuple[float, float]],
                         check_radius: float = 0.35) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """
        Check if any detected front obstacle directly blocks the upcoming path corridor.
        Returns:
            - is_blocked: Boolean
            - blocking_point: (x, y) centroid of blocking obstacle or None
        """
        if not front_points or not remaining_waypoints or len(remaining_waypoints) < 1:
            return False, None

        # Check against first few upcoming waypoints (lookahead corridor)
        lookahead_pts = remaining_waypoints[:min(5, len(remaining_waypoints))]

        blocking_cluster = []

        for px, py in front_points:
            for wx, wy in lookahead_pts:
                dist = math.hypot(px - wx, py - wy)
                if dist <= check_radius:
                    blocking_cluster.append((px, py))
                    break

        if len(blocking_cluster) >= 2:  # Minimum 2 LiDAR points to confirm real obstacle
            avg_x = sum(p[0] for p in blocking_cluster) / len(blocking_cluster)
            avg_y = sum(p[1] for p in blocking_cluster) / len(blocking_cluster)
            return True, (round(avg_x, 3), round(avg_y, 3))

        return False, None
