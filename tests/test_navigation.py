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
