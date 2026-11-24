"""
End-to-End Simulation Integration Test.
Verifies the complete navigation lifecycle:
- Autonomous navigation from start (-2.0, -2.0) to goal (2.0, 2.0)
- Verification that proximity to static obstacles does NOT trigger false BLOCKED states
- Dynamic obstacle injection, LiDAR detection, and A* replanning around it
- Recovery from BLOCKED state when obstacle is cleared
- Multi-destination validation
"""

import math
import pytest
from unittest.mock import MagicMock

from communication.protocol import RobotState
from controllers.autonomous_controller.navigation_controller import NavigationController
from controllers.autonomous_controller.robot_state import TelemetryTracker


class MockMotors:
    def __init__(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.stopped = True

    def set_speeds(self, left: float, right: float):
        self.left_speed = left
        self.right_speed = right
        self.stopped = False

    def stop(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.stopped = True


class MockLidar:
    def __init__(self):
        from navigation.obstacle_detection import ObstacleDetector
        self.detector = ObstacleDetector()
        self.ranges = [3.5] * 360

    def set_obstacle_ahead(self, dist: float):
        self.ranges = [3.5] * 360
        # Beam 180 is 0 relative angle
        self.ranges[180] = dist
        self.ranges[179] = dist + 0.01
        self.ranges[181] = dist + 0.01

    def clear(self):
        self.ranges = [3.5] * 360

    def process(self, rx: float, ry: float, rtheta: float, occupancy_grid=None):
        return self.detector.process_scan(self.ranges, rx, ry, rtheta, occupancy_grid=occupancy_grid)


def test_full_navigation_lifecycle_and_avoidance():
    motors = MockMotors()
    lidar = MockLidar()
    telemetry = TelemetryTracker()
    nav = NavigationController(motors, lidar, telemetry)

    # 1. Set goal to (2.0, 2.0)
    assert nav.set_goal(2.0, 2.0)
    assert nav.start_navigation()
    assert nav.state == RobotState.NAVIGATING
    assert len(nav.follower.waypoints) > 5

    # 2. Simulate robot traversing along waypoints passing near static box (-1.2, -1.0)
    # At (-1.5, -1.5), static box 4 is ahead/right. Static obstacle filtering should prevent false blockage.
    nav.step(-1.5, -1.5, 0.785)
    assert nav.state == RobotState.NAVIGATING, "Must remain NAVIGATING near static obstacles"
    assert not motors.stopped

    # 3. Simulate robot at (-0.5, -0.5) moving towards (0.0, 0.0). Inject dynamic obstacle at (0.0, 0.0)
    # Lidar sees obstacle 0.7m ahead
    lidar.set_obstacle_ahead(0.70)
    nav.step(-0.5, -0.5, 0.785)

    # Should have triggered replan
    assert nav.planner.has_replanned, "Must replan when dynamic obstacle is detected"
    assert nav.planner.replan_count >= 1
    assert nav.state == RobotState.NAVIGATING, "Must return to NAVIGATING with new route"

    # Verify that the new path avoids (0.0, 0.0)
    for wp in nav.follower.waypoints:
        dist_to_dyn = math.hypot(wp[0] - 0.0, wp[1] - 0.0)
        assert dist_to_dyn > 0.25, f"Waypoint {wp} must maintain clearance from dynamic obstacle"


def test_recovery_from_blocked_state():
    motors = MockMotors()
    lidar = MockLidar()
    telemetry = TelemetryTracker()
    nav = NavigationController(motors, lidar, telemetry)

    nav.set_goal(2.0, 2.0)
    nav.start_navigation()

    # Manually place robot into BLOCKED state
    nav.state = RobotState.BLOCKED
    nav.telemetry.state = RobotState.BLOCKED

    # Robot at (0.0, 0.0) with clear lidar
    lidar.clear()
    nav.step(0.0, 0.0, 0.0)

    # Calling recover_from_blocked should restore NAVIGATING state
    success = nav.recover_from_blocked(0.0, 0.0)
    assert success
    assert nav.state == RobotState.NAVIGATING
    assert len(nav.follower.waypoints) > 0


@pytest.mark.parametrize("target", [
    (2.0, 2.0),
    (-2.0, 2.0),
    (2.2, 0.0),
    (0.0, 0.0),
    (2.0, -2.0)
])
def test_all_prescribed_destinations_reachability(target):
    motors = MockMotors()
    lidar = MockLidar()
    telemetry = TelemetryTracker()
    nav = NavigationController(motors, lidar, telemetry)

    assert nav.set_goal(target[0], target[1]), f"Destination {target} must be valid"
    assert nav.start_navigation()
    assert nav.state == RobotState.NAVIGATING
    assert len(nav.follower.waypoints) >= 2
    # Verify final waypoint matches destination within 0.15m
    assert nav.follower.waypoints[-1] == pytest.approx(target, abs=0.15)


def test_replan_cooldown_prevents_thrashing():
    motors = MockMotors()
    lidar = MockLidar()
    telemetry = TelemetryTracker()
    nav = NavigationController(motors, lidar, telemetry)

    # Position robot at (-1.0, -1.0)
    nav.telemetry.x = -1.0
    nav.telemetry.y = -1.0
    nav.telemetry.start_pos = (-1.0, -1.0)

    nav.set_goal(2.0, 2.0)
    nav.start_navigation()

    # First blockage trigger (obstacle directly ahead along path to 2, 2)
    lidar.set_obstacle_ahead(0.5)
    nav.step(-1.0, -1.0, 0.785)
    first_count = nav.planner.replan_count
    assert first_count >= 1

    # Immediate second blockage trigger within milliseconds
    nav.step(-1.0, -1.0, 0.785)
    assert nav.planner.replan_count == first_count, "Debounce cooldown must prevent rapid duplicate replanning"

