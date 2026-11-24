"""
A* (A-Star) Pathfinding Algorithm for 2D Grid Representation.
Features 8-connectivity, diagonal corner-cutting prevention, Euclidean heuristic,
obstacle clearance cost penalties, soft start/goal recovery, and continuous waypoint generation.
"""

import heapq
import math
from typing import List, Tuple, Optional, Dict
import numpy as np

from navigation.occupancy_grid import OccupancyGrid
from config.settings import (
    FREE_CELL_VALUE, OCCUPIED_CELL_VALUE, INFLATED_OBSTACLE_VALUE, DYNAMIC_OBSTACLE_VALUE
)


class AStarPlanner:
    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # 8-direction movements: (dx, dy, cost)
        self.motions = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0)),
        ]

    def _heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Euclidean distance heuristic."""
        dx = node[0] - goal[0]
        dy = node[1] - goal[1]
        return math.hypot(dx, dy)

    def plan(self,
             start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Compute optimal shortest collision-free path from start to goal.
        Returns list of (x, y) world coordinates in meters, or None if no valid path exists.
        """
        raw_start_node = self.grid.world_to_grid(start_world[0], start_world[1])
        raw_goal_node = self.grid.world_to_grid(goal_world[0], goal_world[1])

        # Validate start and goal boundaries
        if not self.grid.is_inside(*raw_start_node):
            return None
        if not self.grid.is_inside(*raw_goal_node):
            return None

        # Soft Start & Soft Goal Recovery:
        # If start or goal node falls inside an obstacle/inflation, snap to nearest traversable cell
        start_node = self.grid.find_nearest_free_cell(*raw_start_node, max_radius=4)
        goal_node = self.grid.find_nearest_free_cell(*raw_goal_node, max_radius=4)

        # Check if goal is truly trapped deep inside a hard obstacle
        if self.grid.grid[goal_node[1], goal_node[0]] >= OCCUPIED_CELL_VALUE:
            return None

        # Check if already at goal
        if start_node == goal_node:
            return [start_world, goal_world]

        # Priority queue stores tuples: (f_score, tie_breaker, (gx, gy))
        open_set = []
        counter = 0
        h_start = self._heuristic(start_node, goal_node)
        heapq.heappush(open_set, (h_start, counter, start_node))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_node: 0.0}
        closed_set = set()

        while open_set:
            current_f, _, current = heapq.heappop(open_set)

            if current == goal_node:
                # Reconstruct path
                grid_path = self._reconstruct_path(came_from, current)
                world_path = self._to_world_path(grid_path)
                # Ensure original goal is preserved at path end
                if world_path and (world_path[-1][0] != goal_world[0] or world_path[-1][1] != goal_world[1]):
                    world_path.append(goal_world)
                return world_path

            if current in closed_set:
                continue
            closed_set.add(current)

            cx, cy = current

            for dx, dy, move_cost in self.motions:
                neighbor = (cx + dx, cy + dy)
                nx, ny = neighbor

                if not self.grid.is_inside(nx, ny):
                    continue

                cell_val = self.grid.grid[ny, nx]

                # Hard obstacles cannot be traversed
                if cell_val >= OCCUPIED_CELL_VALUE:
                    continue

                # Diagonal movement safety: prevent cutting blocked corners
                if dx != 0 and dy != 0:
                    adj1_val = self.grid.grid[cy, cx + dx]
                    adj2_val = self.grid.grid[cy + dy, cx]
                    if adj1_val >= OCCUPIED_CELL_VALUE or adj2_val >= OCCUPIED_CELL_VALUE:
                        continue

                # Proximity penalty for inflated safety cells to encourage open center paths
                inflation_penalty = 1.6 if cell_val >= INFLATED_OBSTACLE_VALUE else 0.0
                tentative_g = g_score[current] + move_cost + inflation_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal_node)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        # No path found
        return None

    def _reconstruct_path(self, came_from: Dict[Tuple[int, int], Tuple[int, int]],
                          current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Backtrack from goal to start to produce ordered grid cells."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _to_world_path(self, grid_path: List[Tuple[int, int]]) -> List[Tuple[float, float]]:
        """Convert grid cells to world coordinates and simplify redundant collinear waypoints."""
        if not grid_path:
            return []

        raw_world_pts = [self.grid.grid_to_world(gx, gy) for gx, gy in grid_path]
        if len(raw_world_pts) <= 2:
            return raw_world_pts

        simplified = [raw_world_pts[0]]
        for i in range(1, len(raw_world_pts) - 1):
            p_prev = simplified[-1]
            p_curr = raw_world_pts[i]
            p_next = raw_world_pts[i + 1]

            # Vector direction check
            v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])

            cross_product = v1[0] * v2[1] - v1[1] * v2[0]
            if abs(cross_product) > 1e-4:
                simplified.append(p_curr)

        simplified.append(raw_world_pts[-1])
        return simplified
