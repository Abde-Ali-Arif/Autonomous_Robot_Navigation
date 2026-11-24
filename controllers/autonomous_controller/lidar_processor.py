"""
LiDAR Sensor Processor for Webots Simulation.
Interfaces with Webots Lidar device, fetches raw range scans,
and delivers processed obstacle data to the navigation system.
"""

import logging
from typing import List, Dict, Any, Optional
from navigation.obstacle_detection import ObstacleDetector
from config.settings import SIMULATION_TIME_STEP, LIDAR_NAME

logger = logging.getLogger("LidarProcessor")


class LidarProcessor:
    def __init__(self, robot_device, time_step: int = SIMULATION_TIME_STEP):
        self.robot = robot_device
        self.time_step = time_step
        self.lidar = None
        self.detector = ObstacleDetector()

        self.last_ranges: List[float] = []
        self._init_lidar()

    def _init_lidar(self):
        """Retrieve and enable Webots Lidar device."""
        try:
            self.lidar = self.robot.getDevice(LIDAR_NAME)
            if self.lidar:
                self.lidar.enable(self.time_step)
                logger.info("Virtual LiDAR sensor enabled successfully.")
            else:
                logger.warning(f"Lidar device '{LIDAR_NAME}' not found.")
        except Exception as e:
            logger.error(f"Error initializing Lidar: {e}")

    def get_ranges(self) -> List[float]:
        """Fetch latest raw range measurements from Webots."""
        if not self.lidar:
            return []
        try:
            # getRangeImage() returns float array of size horizontalResolution (360)
            raw = self.lidar.getRangeImage()
            if raw:
                self.last_ranges = list(raw)
                return self.last_ranges
        except Exception as e:
            logger.debug(f"Error fetching range image: {e}")
        return self.last_ranges

    def process(self, robot_x: float, robot_y: float, robot_theta: float) -> Dict[str, Any]:
        """Run obstacle detection pipeline on latest scan."""
        ranges = self.get_ranges()
        return self.detector.process_scan(ranges, robot_x, robot_y, robot_theta)
