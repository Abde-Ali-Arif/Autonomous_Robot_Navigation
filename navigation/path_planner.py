"""
High-Level Path Planner Orchestrator.
Manages initial route planning, dynamic obstacle injection into occupancy grid,
dynamic replanning triggers, and path history preservation.
"""

from typing import List, Tuple, Optional, Dict, Any

from navigation.occupancy_grid import OccupancyGrid
from navigation.astar import AStarPlanner


class PathPlanner:
    def __init__(self):
        self.occupancy_grid = OccupancyGrid()
        self.astar = AStarPlanner(self.occupancy_grid)

        self.start_pos: Optional[Tuple[float, float]] = None
        self.goal_pos: Optional[Tuple[float, float]] = None

        self.original_path: List[Tuple[float, float]] = []
        self.current_path: List[Tuple[float, float]] = []
        self.replan_count: int = 0
        self.has_replanned: bool = False

    def plan_initial_path(self,
                          start: Tuple[float, float],
                          goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Compute initial global path before simulation starts."""
        self.start_pos = start
        self.goal_pos = goal
        self.replan_count = 0
        self.has_replanned = False

        path = self.astar.plan(start, goal)
        if path:
            self.original_path = list(path)
            self.current_path = list(path)
            return path
        return None

    def replan(self,
               current_pos: Tuple[float, float],
               blocking_obs: Optional[Tuple[float, float]] = None) -> Optional[List[Tuple[float, float]]]:
        """
        Trigger dynamic replan when LiDAR detects a blocked path.
        Updates occupancy grid with new obstacle and calculates alternative route.
        """
        if not self.goal_pos:
            return None

        if blocking_obs:
            self.occupancy_grid.remove_dynamic_obstacle()
            self.occupancy_grid.add_dynamic_obstacle(blocking_obs[0], blocking_obs[1])

        new_path = self.astar.plan(current_pos, self.goal_pos)
        if new_path:
            self.current_path = list(new_path)
            self.replan_count += 1
            self.has_replanned = True
            return new_path
        return None

    def reset(self):
        """Reset path planner and clear dynamic obstacles."""
        self.occupancy_grid.remove_dynamic_obstacle()
        self.original_path = []
        self.current_path = []
        self.replan_count = 0
        self.has_replanned = False
