"""
Unit Tests for LiDAR Obstacle Detection and Safety Sectors.
"""

import math
import pytest
from navigation.obstacle_detection import ObstacleDetector


@pytest.fixture
def detector():
    return ObstacleDetector(front_arc_deg=60.0, stop_dist=0.28, slow_dist=0.50)


def test_clear_lidar_scan(detector):
    # Scan with no nearby obstacles (all beams return max distance 3.5m)
    ranges = [3.5] * 360
    res = detector.process_scan(ranges, robot_x=0.0, robot_y=0.0, robot_theta=0.0)

    assert not res["emergency_stop"]
    assert not res["slow_down"]
    assert res["min_front_dist"] >= 3.0


def test_emergency_stop_trigger(detector):
    # Scan with obstacle 0.20m directly in front
    ranges = [3.5] * 360
    # In 360 beams, 0 angle relative is at index 180 (center)
    ranges[180] = 0.20
    ranges[179] = 0.21
    ranges[181] = 0.21

    res = detector.process_scan(ranges, robot_x=0.0, robot_y=0.0, robot_theta=0.0)

    assert res["emergency_stop"], "Should trigger emergency stop when obstacle is <= 0.28m"
    assert res["slow_down"]
    assert res["min_front_dist"] <= 0.25


def test_route_blocked_detection(detector):
    # Obstacle located at (0.5, 0.0)
    front_points = [(0.5, 0.0), (0.52, 0.02), (0.48, -0.01)]
    # Waypoints traversing straight through (0.5, 0.0)
    waypoints = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (2.0, 0.0)]

    is_blocked, centroid = detector.is_route_blocked(front_points, waypoints, check_radius=0.35)

    assert is_blocked, "Detector must identify that front obstacle blocks planned waypoints."
    assert centroid is not None
    assert centroid[0] == pytest.approx(0.5, abs=0.1)
    assert centroid[1] == pytest.approx(0.0, abs=0.1)
