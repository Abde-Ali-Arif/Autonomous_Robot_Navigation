"""
LiDAR Obstacle Detection and Safety Sector Analysis.
Processes 360-degree range scans, calculates front-sector clearance,
filters out known static map geometry, and identifies active corridor blockages.
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
                     robot_theta: float,
                     occupancy_grid: Optional[Any] = None) -> Dict[str, Any]:
        """
        Process raw 360-degree LiDAR range measurements.
        Filters out points belonging to known static obstacles so only new dynamic
        obstacles trigger path replanning.
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
            # Filter non-valid range readings (out-of-range or sensor limits)
            if math.isnan(dist) or math.isinf(dist) or dist < self.min_range or dist >= (self.max_range - 1e-4):
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

                # Map-based Filtering:
                # If an occupancy grid is provided, check if point belongs to known static obstacles.
                # If it is a known static obstacle, it does NOT get added to front_points as a dynamic blockage!
                if occupancy_grid is not None and occupancy_grid.is_static_obstacle(obs_x, obs_y):
                    continue

                front_points.append(point)

        front_dist_val = min_front_dist if not math.isinf(min_front_dist) else self.max_range
        emergency_stop = (front_dist_val <= self.stop_dist)
        slow_down = (front_dist_val <= self.slow_dist)

        return {
            "min_front_dist": round(front_dist_val, 3),
            "emergency_stop": emergency_stop,
            "slow_down": slow_down,
            "detected_points": detected_points,
            "front_points": front_points
        }

    @staticmethod
    def _dist_point_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate minimum Euclidean distance from point (px, py) to line segment (x1, y1)-(x2, y2)."""
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-6:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def is_route_blocked(self,
                         front_points: List[Tuple[float, float]],
                         remaining_waypoints: List[Tuple[float, float]],
                         check_radius: float = 0.25,
                         max_lookahead_dist: float = 1.40,
                         robot_pos: Optional[Tuple[float, float]] = None) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """
        Check if any detected front dynamic obstacle directly blocks the upcoming path corridor.
        Checks both discrete waypoints and intermediate continuous line segments within lookahead.
        """
        if not front_points or not remaining_waypoints or len(remaining_waypoints) < 1:
            return False, None

        # Lookahead: check first 4 segments
        num_segments = min(4, len(remaining_waypoints) - 1)
        blocking_cluster = []

        for px, py in front_points:
            # Lookahead distance filter: ignore obstacles far beyond the immediate navigation corridor
            if robot_pos is not None:
                dist_to_robot = math.hypot(px - robot_pos[0], py - robot_pos[1])
                if dist_to_robot > max_lookahead_dist:
                    continue

            blocked = False
            # 1. Check distance to upcoming discrete waypoints
            for wx, wy in remaining_waypoints[:min(5, len(remaining_waypoints))]:
                if math.hypot(px - wx, py - wy) <= check_radius:
                    blocking_cluster.append((px, py))
                    blocked = True
                    break

            if blocked:
                continue

            # 2. Check distance to continuous line segments between waypoints
            for s in range(num_segments):
                x1, y1 = remaining_waypoints[s]
                x2, y2 = remaining_waypoints[s + 1]
                if self._dist_point_to_segment(px, py, x1, y1, x2, y2) <= check_radius:
                    blocking_cluster.append((px, py))
                    break

        if len(blocking_cluster) >= 2:  # Minimum 2 LiDAR points to confirm real obstacle
            avg_x = sum(p[0] for p in blocking_cluster) / len(blocking_cluster)
            avg_y = sum(p[1] for p in blocking_cluster) / len(blocking_cluster)
            return True, (round(avg_x, 3), round(avg_y, 3))

        return False, None
