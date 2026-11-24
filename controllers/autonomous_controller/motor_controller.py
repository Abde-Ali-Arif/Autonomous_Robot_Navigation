"""
Motor Controller for Differential-Drive Robot.
Interacts with Webots RotationalMotor devices for velocity control.
"""

import logging
from typing import Optional
from config.settings import MAX_WHEEL_SPEED

logger = logging.getLogger("MotorController")


class MotorController:
    def __init__(self, robot_device):
        self.robot = robot_device
        self.left_motor = None
        self.right_motor = None
        self.max_speed = MAX_WHEEL_SPEED

        self._init_motors()

    def _init_motors(self):
        """Retrieve and configure Webots wheel motors."""
        try:
            self.left_motor = self.robot.getDevice("left_wheel_motor")
            self.right_motor = self.robot.getDevice("right_wheel_motor")

            if self.left_motor and self.right_motor:
                # Set position to infinity to enable continuous velocity mode
                self.left_motor.setPosition(float("inf"))
                self.right_motor.setPosition(float("inf"))
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                logger.info("Differential wheel motors initialized successfully.")
            else:
                logger.warning("Wheel motors not found in robot device tree.")
        except Exception as e:
            logger.error(f"Error initializing wheel motors: {e}")

    def set_speeds(self, left_speed: float, right_speed: float):
        """Command angular speeds (rad/s) to left and right motors."""
        if not self.left_motor or not self.right_motor:
            return

        # Clamp to physical motor limits
        left_clamped = max(-self.max_speed, min(self.max_speed, left_speed))
        right_clamped = max(-self.max_speed, min(self.max_speed, right_speed))

        try:
            self.left_motor.setVelocity(left_clamped)
            self.right_motor.setVelocity(right_clamped)
        except Exception as e:
            logger.error(f"Failed to command motor speeds: {e}")

    def stop(self):
        """Bring both drive wheels to an immediate halt."""
        self.set_speeds(0.0, 0.0)
