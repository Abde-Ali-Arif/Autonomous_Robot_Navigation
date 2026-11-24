"""
Occupancy Grid Mapping for Indoor Navigation.
Discretizes continuous 2D space into a grid matrix and handles coordinate
transformations, static obstacle projection, and safety inflation boundaries.
"""

from typing import List, Tuple, Dict, Any, Optional
import numpy as np

from config.settings import (
    WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX,
    GRID_RESOLUTION, GRID_WIDTH, GRID_HEIGHT,
    FREE_CELL_VALUE, OCCUPIED_CELL_VALUE, INFLATED_OBSTACLE_VALUE, DYNAMIC_OBSTACLE_VALUE,
    INFLATION_CELLS, STATIC_OBSTACLES
)


class OccupancyGrid:
    def __init__(self,
                 x_min: float = WORLD_X_MIN,
                 x_max: float = WORLD_X_MAX,
                 y_min: float = WORLD_Y_MIN,
                 y_max: float = WORLD_Y_MAX,
                 resolution: float = GRID_RESOLUTION):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution

        self.width = int(round((x_max - x_min) / resolution))
        self.height = int(round((y_max - y_min) / resolution))

        # 2D Grid: [height, width] = [row (Y), col (X)]
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.static_mask = np.zeros((self.height, self.width), dtype=bool)
        self.dynamic_mask = np.zeros((self.height, self.width), dtype=bool)

        # Initialize with static obstacles
        self.load_static_obstacles(STATIC_OBSTACLES)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert continuous world coordinates (meters) to discrete (col, row) = (gx, gy)."""
        gx = int(np.floor((x - self.x_min) / self.resolution))
        gy = int(np.floor((y - self.y_min) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert discrete grid coordinates (col, row) to world coordinates (meters)."""
        x = self.x_min + (gx + 0.5) * self.resolution
        y = self.y_min + (gy + 0.5) * self.resolution
        return round(x, 3), round(y, 3)

    def is_inside(self, gx: int, gy: int) -> bool:
        """Check if grid index is within arena boundaries."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx: int, gy: int) -> bool:
        """A cell is free if within bounds and value == FREE_CELL_VALUE."""
        if not self.is_inside(gx, gy):
            return False
        return self.grid[gy, gx] == FREE_CELL_VALUE

    def is_traversable(self, gx: int, gy: int) -> bool:
        """A cell is traversable if not a hard physical obstacle."""
        if not self.is_inside(gx, gy):
            return False
        val = self.grid[gy, gx]
        return val < OCCUPIED_CELL_VALUE

    def is_static_obstacle(self, x: float, y: float, margin_cells: int = 1) -> bool:
        """
        Check if world coordinate (x, y) belongs to known static walls or static obstacles.
        Used to filter LiDAR detections so static geometry never triggers false dynamic replanning.
        """
        gx, gy = self.world_to_grid(x, y)
        if not self.is_inside(gx, gy):
            return True  # Outside bounds is static perimeter wall

        # Check the cell and immediate margin for sensor tolerance
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                nx, ny = gx + dx, gy + dy
                if self.is_inside(nx, ny) and self.static_mask[ny, nx]:
                    return True
        return False

    def find_nearest_free_cell(self, gx: int, gy: int, max_radius: int = 4) -> Tuple[int, int]:
        """
        Find closest traversable cell if a coordinate falls slightly inside an obstacle boundary.
        Ensures A* never fails due to 1-pixel contact with an inflated boundary.
        """
        if self.is_inside(gx, gy) and self.grid[gy, gx] < OCCUPIED_CELL_VALUE:
            return gx, gy

        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self.is_inside(nx, ny) and self.grid[ny, nx] < OCCUPIED_CELL_VALUE:
                            return nx, ny
        return gx, gy

    def load_static_obstacles(self, obstacles: List[Dict[str, Any]]):
        """Burn static boundary walls and indoor obstacle boxes into the grid."""
        self.grid.fill(FREE_CELL_VALUE)
        self.static_mask.fill(False)

        # 1. Outer boundary walls (thickness ~1 cell)
        self.static_mask[0, :] = True
        self.static_mask[-1, :] = True
        self.static_mask[:, 0] = True
        self.static_mask[:, -1] = True

        # 2. Rectangular Static Obstacles
        for obs in obstacles:
            ox, oy = obs["x"], obs["y"]
            sx, sy = obs["size_x"], obs["size_y"]

            min_x, max_x = ox - sx / 2.0, ox + sx / 2.0
            min_y, max_y = oy - sy / 2.0, oy + sy / 2.0

            gx1, gy1 = self.world_to_grid(min_x, min_y)
            gx2, gy2 = self.world_to_grid(max_x, max_y)

            gx1 = max(0, min(self.width - 1, gx1))
            gx2 = max(0, min(self.width - 1, gx2))
            gy1 = max(0, min(self.height - 1, gy1))
            gy2 = max(0, min(self.height - 1, gy2))

            self.static_mask[gy1:gy2 + 1, gx1:gx2 + 1] = True

        # Set occupied values
        self.grid[self.static_mask] = OCCUPIED_CELL_VALUE

        # Inflate obstacles to keep robot body from grazing walls
        self._apply_inflation()

    def _apply_inflation(self, inflation_cells: int = INFLATION_CELLS):
        """Expand obstacle footprint by inflation_cells to ensure robot clearance."""
        hard_obstacles = (self.static_mask | self.dynamic_mask)
        inflated = np.zeros_like(self.grid, dtype=bool)

        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                if dx * dx + dy * dy <= inflation_cells * inflation_cells:
                    shifted = np.roll(np.roll(hard_obstacles, dy, axis=0), dx, axis=1)
                    # Clear boundary roll-overs
                    if dy > 0:
                        shifted[:dy, :] = False
                    elif dy < 0:
                        shifted[dy:, :] = False
                    if dx > 0:
                        shifted[:, :dx] = False
                    elif dx < 0:
                        shifted[:, dx:] = False

                    inflated |= shifted

        # Assign inflated cost to non-hard-obstacle cells
        inflated_only = inflated & (~hard_obstacles)
        self.grid[inflated_only] = INFLATED_OBSTACLE_VALUE
        self.grid[hard_obstacles] = OCCUPIED_CELL_VALUE

    def add_dynamic_obstacle(self, x: float, y: float, size_x: float = 0.45, size_y: float = 0.45):
        """Mark a new dynamic obstacle detected or injected in the environment."""
        # Clamp coordinates to safe arena boundaries
        cx = max(self.x_min + 0.4, min(self.x_max - 0.4, x))
        cy = max(self.y_min + 0.4, min(self.y_max - 0.4, y))

        min_x, max_x = cx - size_x / 2.0, cx + size_x / 2.0
        min_y, max_y = cy - size_y / 2.0, cy + size_y / 2.0

        gx1, gy1 = self.world_to_grid(min_x, min_y)
        gx2, gy2 = self.world_to_grid(max_x, max_y)

        gx1 = max(0, min(self.width - 1, gx1))
        gx2 = max(0, min(self.width - 1, gx2))
        gy1 = max(0, min(self.height - 1, gy1))
        gy2 = max(0, min(self.height - 1, gy2))

        self.dynamic_mask[gy1:gy2 + 1, gx1:gx2 + 1] = True
        self._recompute_grid()

    def remove_dynamic_obstacle(self):
        """Clear dynamic obstacles from the occupancy grid."""
        self.dynamic_mask.fill(False)
        self._recompute_grid()

    def _recompute_grid(self):
        """Recalculate entire grid layers with static and dynamic obstacles."""
        self.grid.fill(FREE_CELL_VALUE)
        self.grid[self.static_mask] = OCCUPIED_CELL_VALUE
        self.grid[self.dynamic_mask] = DYNAMIC_OBSTACLE_VALUE
        self._apply_inflation()

    def is_world_pos_valid(self, x: float, y: float) -> bool:
        """Validate if a continuous world coordinate is safe (free of static and dynamic obstacles)."""
        # Strict boundary check (stay inside walls with clearance)
        if not (-2.60 <= x <= 2.60 and -2.60 <= y <= 2.60):
            return False

        gx, gy = self.world_to_grid(x, y)
        if not self.is_inside(gx, gy):
            return False
        return self.grid[gy, gx] < OCCUPIED_CELL_VALUE
