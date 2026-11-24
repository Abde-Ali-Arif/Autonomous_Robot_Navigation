"""
Unit Tests for Kinematics, Coordinate Transformations, and Waypoint Tracking.
"""

import math
import pytest
from navigation.occupancy_grid import OccupancyGrid
from navigation.waypoint_follower import WaypointFollower
from navigation.path_planner import PathPlanner


def test_coordinate_transformations():
    grid = OccupancyGrid()

    # Center of room (0.0, 0.0) should map to middle grid cell
    gx, gy = grid.world_to_grid(0.0, 0.0)
    assert gx == 30 and gy == 30

    # Converting back should give close to (0.0, 0.0)
    wx, wy = grid.grid_to_world(gx, gy)
    assert abs(wx - 0.05) < 0.1
    assert abs(wy - 0.05) < 0.1

    # World bounds check
    assert grid.is_inside(0, 0)
    assert grid.is_inside(59, 59)
    assert not grid.is_inside(60, 60)


def test_waypoint_follower_tracking():
    follower = WaypointFollower()
    waypoints = [(-2.0, -2.0), (0.0, 0.0), (2.0, 2.0)]
    follower.set_path(waypoints)

    assert len(follower.get_remaining_waypoints()) == 3

    # Robot at start position, facing positive X (0 radians), target is (0.0, 0.0)
    control = follower.compute_velocities(robot_x=-2.0, robot_y=-2.0, robot_theta=0.0)
    assert not control["goal_reached"]
    assert control["linear_vel"] >= 0.0
    # Desired heading to (0,0) is pi/4 (~0.78 rad), robot theta is 0 -> positive angular velocity
    assert control["angular_vel"] > 0.0

    # Move robot to final goal
    control_goal = follower.compute_velocities(robot_x=2.0, robot_y=2.0, robot_theta=math.pi / 4)
    assert control_goal["goal_reached"]
    assert control_goal["left_speed"] == 0.0
    assert control_goal["right_speed"] == 0.0


def test_path_planner_lifecycle():
    planner = PathPlanner()
    start = (-2.0, -2.0)
    goal = (2.0, 2.0)

    # Initial plan
    path = planner.plan_initial_path(start, goal)
    assert path is not None
    assert len(planner.original_path) > 0
    assert not planner.has_replanned

    # Trigger replan with dynamic obstacle
    new_path = planner.replan(current_pos=(-0.5, -0.5), blocking_obs=(0.0, 0.0))
    assert new_path is not None
    assert planner.has_replanned
    assert planner.replan_count == 1


def test_emergency_stop_rotation_recovery():
    follower = WaypointFollower()
    waypoints = [(0.0, 0.0), (0.0, 1.0)]  # Waypoint is to the left (+Y)
    follower.set_path(waypoints)

    # Robot facing forward along +X (theta = 0), waypoint is along +Y (theta_des = pi/2)
    # Under emergency_stop, linear velocity must be 0.0 while angular velocity > 0 to rotate
    control = follower.compute_velocities(robot_x=0.0, robot_y=0.0, robot_theta=0.0, emergency_stop=True)

    assert control["linear_vel"] == 0.0, "Linear velocity must be 0 during emergency stop"
    assert control["angular_vel"] > 0.0, "Angular velocity must be positive to turn toward +Y waypoint"
    assert control["left_speed"] < 0.0, "Left wheel should reverse during in-place counter-clockwise turn"
    assert control["right_speed"] > 0.0, "Right wheel should move forward during in-place counter-clockwise turn"


def test_navigation_controller_recover_from_blocked():
    from unittest.mock import MagicMock
    from communication.protocol import RobotState
    from controllers.autonomous_controller.navigation_controller import NavigationController
    from controllers.autonomous_controller.robot_state import TelemetryTracker

    mock_motors = MagicMock()
    mock_lidar = MagicMock()
    telemetry = TelemetryTracker()

    nav = NavigationController(mock_motors, mock_lidar, telemetry)
    nav.telemetry.goal_pos = (2.0, 2.0)
    nav.state = RobotState.BLOCKED

    # Attempt recovery from free location (0.0, 0.0)
    recovered = nav.recover_from_blocked(current_x=0.0, current_y=0.0)
    assert recovered, "Recovery should find a valid path from (0, 0) to (2, 2)"
    assert nav.state == RobotState.NAVIGATING
    assert len(nav.follower.waypoints) > 0


def test_navigation_controller_goal_validation():
    from unittest.mock import MagicMock
    from controllers.autonomous_controller.navigation_controller import NavigationController
    from controllers.autonomous_controller.robot_state import TelemetryTracker

    nav = NavigationController(MagicMock(), MagicMock(), TelemetryTracker())

    # Out of boundary
    assert not nav.set_goal(3.5, 0.0), "Goal outside arena should be rejected"
    assert not nav.set_goal(0.0, -3.2), "Goal outside arena should be rejected"

    # Overlapping static obstacle (obs_box_1 at -0.8, 0.0)
    assert not nav.set_goal(-0.8, 0.0), "Goal overlapping static obstacle should be rejected"

    # Valid goals
    assert nav.set_goal(2.0, 2.0), "Valid goal should be accepted"
    assert nav.set_goal(-2.0, 2.0), "Valid goal should be accepted"
