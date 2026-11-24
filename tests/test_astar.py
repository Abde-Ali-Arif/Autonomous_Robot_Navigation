"""
Unit Tests for A* Pathfinding Algorithm.
"""

import pytest
from navigation.occupancy_grid import OccupancyGrid
from navigation.astar import AStarPlanner


@pytest.fixture
def planner():
    grid = OccupancyGrid()
    return AStarPlanner(grid)


def test_astar_finds_valid_path(planner):
    start = (-2.0, -2.0)
    goal = (2.0, 2.0)
    path = planner.plan(start, goal)

    assert path is not None, "A* should find a valid collision-free path."
    assert len(path) >= 2, "Path must contain at least start and goal waypoints."
    assert path[0] == pytest.approx(start, abs=0.15)
    assert path[-1] == pytest.approx(goal, abs=0.15)


def test_astar_avoids_static_obstacles(planner):
    start = (-2.0, -2.0)
    goal = (2.0, 2.0)
    path = planner.plan(start, goal)
    assert path is not None

    # Verify no waypoint falls inside an occupied cell
    for pt in path:
        gx, gy = planner.grid.world_to_grid(pt[0], pt[1])
        assert planner.grid.grid[gy, gx] < 100, f"Waypoint {pt} overlaps an obstacle!"


def test_astar_unreachable_goal(planner):
    # Goal placed inside a solid wall
    start = (-2.0, -2.0)
    goal = (3.05, 0.0)  # On the east boundary wall
    path = planner.plan(start, goal)
    assert path is None, "A* must return None when destination is inside an obstacle."


def test_astar_replan_with_dynamic_obstacle(planner):
    start = (-2.0, -2.0)
    goal = (2.0, 2.0)
    initial_path = planner.plan(start, goal)
    assert initial_path is not None

    # Introduce dynamic obstacle in the middle
    planner.grid.add_dynamic_obstacle(0.0, 0.0, 0.6, 0.6)
    new_path = planner.plan(start, goal)

    assert new_path is not None, "A* should successfully find an alternative route."
    # Ensure new path does not pass through the dynamic obstacle center
    for pt in new_path:
        dist_to_dyn = ((pt[0] - 0.0)**2 + (pt[1] - 0.0)**2)**0.5
        assert dist_to_dyn > 0.25, "Replanned path must avoid the dynamic obstacle!"


@pytest.mark.parametrize("destination", [
    (2.0, 2.0),
    (-2.0, 2.0),
    (2.2, 0.0),
    (0.0, 0.0),
    (2.0, -2.0)
])
def test_astar_multiple_destinations_reachable(planner, destination):
    start = (-2.0, -2.0)
    path = planner.plan(start, destination)
    assert path is not None, f"Destination {destination} should be reachable from {start}."
    assert len(path) >= 2
    assert path[-1] == pytest.approx(destination, abs=0.15)


def test_astar_soft_start_and_goal_recovery(planner):
    # Place start slightly inside inflated boundary margin
    start_near_wall = (-2.65, -2.0)  # Near west wall
    goal = (2.0, 2.0)
    path = planner.plan(start_near_wall, goal)
    assert path is not None, "A* should find nearest free cell and successfully plan path."
    assert path[-1] == pytest.approx(goal, abs=0.15)
