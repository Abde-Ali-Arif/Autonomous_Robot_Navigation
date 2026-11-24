"""
Autonomous Mobile Robot Navigation Configuration Settings.
Defines environment parameters, robot physical dimensions, sensor properties,
occupancy grid mapping, communication socket ports, and navigation thresholds.
"""

import os
from pathlib import Path

# Base Paths
BASE_DIR = Path(__file__).resolve().parent.parent
CONFIG_DIR = BASE_DIR / "config"
LOGS_DIR = BASE_DIR / "logs"
WORLDS_DIR = BASE_DIR / "worlds"
CONTROLLERS_DIR = BASE_DIR / "controllers"

# Ensure logs directory exists
os.makedirs(LOGS_DIR, exist_ok=True)

# ---------------------------------------------------------
# 1. Environment & World Geometry (in meters)
# Webots World uses standard ENU (X: East, Y: North, Z: Up)
# ---------------------------------------------------------
WORLD_X_MIN = -3.0
WORLD_X_MAX = 3.0
WORLD_Y_MIN = -3.0
WORLD_Y_MAX = 3.0

ROOM_WIDTH = WORLD_X_MAX - WORLD_X_MIN   # 6.0 meters
ROOM_HEIGHT = WORLD_Y_MAX - WORLD_Y_MIN  # 6.0 meters

# ---------------------------------------------------------
# 2. Occupancy Grid Parameters
# ---------------------------------------------------------
GRID_RESOLUTION = 0.10  # 10 cm per cell (60 x 60 grid for 6m x 6m room)
GRID_WIDTH = int(ROOM_WIDTH / GRID_RESOLUTION)   # 60 cells
GRID_HEIGHT = int(ROOM_HEIGHT / GRID_RESOLUTION) # 60 cells

# Cost values for grid cells
FREE_CELL_VALUE = 0
OCCUPIED_CELL_VALUE = 100
INFLATED_OBSTACLE_VALUE = 75
DYNAMIC_OBSTACLE_VALUE = 90

# Obstacle Inflation / Robot Clearance (safety padding)
ROBOT_RADIUS = 0.18  # Actual physical radius ~16cm + safety margin
INFLATION_RADIUS = 0.25  # In meters, distance around obstacles considered high cost
INFLATION_CELLS = int(INFLATION_RADIUS / GRID_RESOLUTION)  # ~3 cells

# ---------------------------------------------------------
# 3. Differential-Drive Robot Parameters
# ---------------------------------------------------------
WHEEL_RADIUS = 0.04       # 4 cm radius
AXLE_LENGTH = 0.16        # 16 cm wheel-to-wheel distance
MAX_WHEEL_SPEED = 6.28    # Max rotational speed (rad/s), ~0.25 m/s linear velocity
DEFAULT_CRUISE_SPEED = 4.0 # Nominal wheel speed (rad/s)
SLOW_CRUISE_SPEED = 2.0    # Reduced speed when near obstacles (rad/s)

# Control gains (Proportional Waypoint Following)
KP_HEADING = 3.2          # Proportional gain for heading error
HEADING_TOLERANCE = 0.12  # Radians (~7 degrees) to switch to straight drive
WAYPOINT_TOLERANCE = 0.15 # Distance threshold (meters) to consider waypoint reached
GOAL_TOLERANCE = 0.12     # Final destination tolerance (meters)

# ---------------------------------------------------------
# 4. Virtual LiDAR Sensor Configuration
# ---------------------------------------------------------
LIDAR_NAME = "lidar"
LIDAR_TIME_STEP = 32      # ms
LIDAR_MIN_RANGE = 0.06    # meters
LIDAR_MAX_RANGE = 3.50    # meters
LIDAR_FOV = 6.2831853     # 360 degrees (2 * PI)
LIDAR_BEAMS = 360         # 1 sample per degree

# Safety Distances
EMERGENCY_STOP_DISTANCE = 0.28   # Halt immediately if obstacle within 28 cm in front
SLOWDOWN_DISTANCE = 0.50         # Slow down if obstacle within 50 cm
OBSTACLE_DETECTION_FRONT_ARC = 60 # Degrees (+/- 30 degrees ahead)

# ---------------------------------------------------------
# 5. Default Start and Goal Positions (in meters)
# ---------------------------------------------------------
DEFAULT_START_POS = [-2.0, -2.0]
DEFAULT_GOAL_POS = [2.0, 2.0]

# Predefined Static Obstacles (Center X, Center Y, Width, Height) in world meters
STATIC_OBSTACLES = [
    {"id": "obs_box_1", "x": -0.8, "y": 0.0, "size_x": 0.6, "size_y": 0.6},
    {"id": "obs_box_2", "x": 0.8, "y": -0.5, "size_x": 0.6, "size_y": 0.6},
    {"id": "obs_box_3", "x": 0.0, "y": 1.2, "size_x": 0.8, "size_y": 0.5},
    {"id": "obs_box_4", "x": -1.2, "y": -1.0, "size_x": 0.5, "size_y": 0.5},
]

# Dynamic Obstacle defaults (for user injection)
DEFAULT_DYNAMIC_OBSTACLE = {
    "id": "dynamic_obs_1",
    "x": 0.2,
    "y": 0.2,
    "size_x": 0.45,
    "size_y": 0.45,
    "height": 0.5
}

# ---------------------------------------------------------
# 6. Communication (IPC Socket Server)
# ---------------------------------------------------------
SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 5050
SOCKET_BUFFER_SIZE = 65536
SOCKET_TIMEOUT = 0.5

# ---------------------------------------------------------
# 7. Simulation Control Time Step
# ---------------------------------------------------------
SIMULATION_TIME_STEP = 32  # ms (Matches Webots physics step)
