"""
Master Autonomous Mobile Robot Webots Supervisor Controller.
Manages physical simulation loop, sensor acquisition, motor actuation,
dynamic 3D obstacle manipulation, and real-time IPC telemetry broadcast.
"""

import math
import os
import sys
import time
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller import Supervisor

from config.settings import (
    SIMULATION_TIME_STEP, DEFAULT_START_POS, DEFAULT_GOAL_POS,
    DEFAULT_DYNAMIC_OBSTACLE
)
from communication.server import SimulationServer
from communication.protocol import CommandType, RobotState
from controllers.autonomous_controller.robot_state import TelemetryTracker
from controllers.autonomous_controller.motor_controller import MotorController
from controllers.autonomous_controller.lidar_processor import LidarProcessor
from controllers.autonomous_controller.navigation_controller import NavigationController


def main():
    print("[Robot Controller] Initializing Autonomous Navigation System...")

    # Initialize Webots Supervisor
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())
    if time_step <= 0:
        time_step = SIMULATION_TIME_STEP

    # 1. Initialize Subsystems
    motors = MotorController(robot)
    lidar = LidarProcessor(robot, time_step)
    telemetry = TelemetryTracker()

    # 2. Initialize Sensors (GPS and InertialUnit)
    gps = robot.getDevice("gps")
    if gps:
        gps.enable(time_step)

    imu = robot.getDevice("inertial_unit")
    if imu:
        imu.enable(time_step)

    # 3. Retrieve Dynamic Obstacle and Robot Nodes via Supervisor API
    dynamic_obs_node = robot.getFromDef("DYNAMIC_OBSTACLE")
    dynamic_obs_trans_field = dynamic_obs_node.getField("translation") if dynamic_obs_node else None

    robot_node = robot.getSelf()
    robot_trans_field = robot_node.getField("translation") if robot_node else None
    robot_rot_field = robot_node.getField("rotation") if robot_node else None

    dynamic_obstacle_active = False
    current_dynamic_obs_coords = None

    # 4. Navigation Controller
    nav_controller = NavigationController(motors, lidar, telemetry)
    # Pre-plan initial route
    nav_controller.planner.plan_initial_path(
        (DEFAULT_START_POS[0], DEFAULT_START_POS[1]),
        (DEFAULT_GOAL_POS[0], DEFAULT_GOAL_POS[1])
    )
    if nav_controller.planner.original_path:
        nav_controller.follower.set_path(nav_controller.planner.original_path)

    # 5. Start Communication Server (Dual-channel: Socket + File Sync)
    server = SimulationServer()
    server.start()

    print("[Robot Controller] System fully online. Ready for mission commands.")

    # 6. Main Webots Physics Control Loop
    try:
        while robot.step(time_step) != -1:
            # A. Read Robot Pose
            current_x, current_y = DEFAULT_START_POS[0], DEFAULT_START_POS[1]
            current_theta = 0.0

            if gps:
                gps_vals = gps.getValues()
                if not any(math.isnan(v) for v in gps_vals):
                    current_x, current_y = gps_vals[0], gps_vals[1]

            if imu:
                rpy = imu.getRollPitchYaw()
                if not any(math.isnan(v) for v in rpy):
                    current_theta = rpy[2]  # Yaw angle in ENU is Z rotation
            elif robot_rot_field:
                # Fallback to supervisor rotation
                rot_vals = robot_rot_field.getSFRotation()
                current_theta = rot_vals[3] * rot_vals[2]

            # B. Consume Incoming Dashboard Commands
            commands = server.get_pending_commands()
            for cmd_obj in commands:
                cmd_type = cmd_obj.get("command")
                payload = cmd_obj.get("payload", {})

                if cmd_type == CommandType.START_NAV.value or cmd_type == "START_NAV":
                    nav_controller.start_navigation()

                elif cmd_type == CommandType.PAUSE_NAV.value or cmd_type == "PAUSE_NAV":
                    nav_controller.pause_navigation()

                elif cmd_type == CommandType.RESUME_NAV.value or cmd_type == "RESUME_NAV":
                    nav_controller.resume_navigation()

                elif cmd_type == CommandType.STOP_NAV.value or cmd_type == "STOP_NAV":
                    nav_controller.stop_navigation()

                elif cmd_type == CommandType.SET_GOAL.value or cmd_type == "SET_GOAL":
                    goal = payload.get("goal", DEFAULT_GOAL_POS)
                    nav_controller.set_goal(float(goal[0]), float(goal[1]))

                elif cmd_type == CommandType.ADD_OBSTACLE.value or cmd_type == "ADD_OBSTACLE":
                    # Determine dynamic obstacle location
                    ox = payload.get("x", DEFAULT_DYNAMIC_OBSTACLE["x"])
                    oy = payload.get("y", DEFAULT_DYNAMIC_OBSTACLE["y"])

                    # If obstacle position not specified, place it 0.7m ahead along robot's current heading
                    if "x" not in payload and "y" not in payload and nav_controller.follower.waypoints:
                        rem_wps = nav_controller.follower.get_remaining_waypoints()
                        if len(rem_wps) >= 2:
                            target_wp = rem_wps[1]
                            ox = round(target_wp[0], 2)
                            oy = round(target_wp[1], 2)
                        else:
                            ox = round(current_x + 0.7 * math.cos(current_theta), 2)
                            oy = round(current_y + 0.7 * math.sin(current_theta), 2)

                    # Physically translate obstacle into the 3D Webots arena floor!
                    if dynamic_obs_trans_field:
                        dynamic_obs_trans_field.setSFVec3f([ox, oy, 0.225])
                        dynamic_obstacle_active = True
                        current_dynamic_obs_coords = [ox, oy]
                        print(f"[Supervisor] Injected 3D dynamic obstacle at ({ox}, {oy})")

                elif cmd_type == CommandType.REMOVE_OBSTACLE.value or cmd_type == "REMOVE_OBSTACLE":
                    if dynamic_obs_trans_field:
                        # Translate below arena floor
                        dynamic_obs_trans_field.setSFVec3f([0.0, 0.0, -5.0])
                        dynamic_obstacle_active = False
                        current_dynamic_obs_coords = None
                        nav_controller.planner.occupancy_grid.remove_dynamic_obstacle()
                        print("[Supervisor] Removed dynamic obstacle from arena.")

                elif cmd_type == CommandType.RESET.value or cmd_type == "RESET":
                    # 1. Reset dynamic obstacle below floor
                    if dynamic_obs_trans_field:
                        dynamic_obs_trans_field.setSFVec3f([0.0, 0.0, -5.0])
                        dynamic_obstacle_active = False
                        current_dynamic_obs_coords = None

                    # 2. Reset robot position in Webots world
                    if robot_trans_field:
                        robot_trans_field.setSFVec3f([DEFAULT_START_POS[0], DEFAULT_START_POS[1], 0.04])
                    if robot_rot_field:
                        robot_rot_field.setSFRotation([0.0, 0.0, 1.0, 0.0])

                    # 3. Reset physics velocity
                    robot_node.resetPhysics()

                    # 4. Reset navigation stack
                    nav_controller.reset(
                        (DEFAULT_START_POS[0], DEFAULT_START_POS[1]),
                        (DEFAULT_GOAL_POS[0], DEFAULT_GOAL_POS[1])
                    )
                    print("[Supervisor] Simulation environment reset successfully.")

            # C. Step Navigation System
            nav_controller.step(current_x, current_y, current_theta)

            # D. Broadcast Comprehensive Real-Time Telemetry
            telemetry_data = telemetry.to_dict()
            lidar_ranges = lidar.last_ranges if lidar.last_ranges else []

            # Compute min distance for dashboard gauge
            valid_ranges = [r for r in lidar_ranges if not math.isnan(r) and not math.isinf(r) and r >= 0.06]
            min_lidar_dist = min(valid_ranges) if valid_ranges else 3.5

            state_payload = {
                "connected": True,
                "telemetry": telemetry_data,
                "navigation": {
                    "state": nav_controller.state.value if isinstance(nav_controller.state, RobotState) else str(nav_controller.state),
                    "original_path": nav_controller.planner.original_path,
                    "current_path": nav_controller.planner.current_path,
                    "has_replanned": nav_controller.planner.has_replanned,
                    "replan_count": nav_controller.planner.replan_count,
                    "detected_obstacles_count": nav_controller.detected_obstacles_count,
                    "min_lidar_distance": round(min_lidar_dist, 2),
                    "dynamic_obstacle_active": dynamic_obstacle_active,
                    "dynamic_obstacle_pos": current_dynamic_obs_coords,
                    "status_message": telemetry.status_message
                },
                "lidar_samples": lidar_ranges[::5] if lidar_ranges else []  # Subsampled for efficient JSON streaming
            }

            server.update_state(state_payload)

    except KeyboardInterrupt:
        print("[Robot Controller] Shutting down...")
    finally:
        motors.stop()
        server.stop()


if __name__ == "__main__":
    main()
