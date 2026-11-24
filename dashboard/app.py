"""
Interactive Streamlit Robotics Mission Control Station.
Autonomous Mobile Robot Navigation and Dynamic Obstacle Avoidance Using 3D Simulation.
"""

import math
import sys
import time
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import streamlit as st

from communication.client import SimulationClient
from communication.protocol import CommandType, RobotState
from config.settings import (
    DEFAULT_START_POS, DEFAULT_GOAL_POS,
    WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX,
    STATIC_OBSTACLES, DEFAULT_DYNAMIC_OBSTACLE
)
from dashboard.components import apply_custom_css, render_metric_card
from dashboard.visualization import create_navigation_plot, create_lidar_polar_plot

# Configure Streamlit Page
st.set_page_config(
    page_title="AMR Mission Control | 3D Robotics",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

apply_custom_css()

# Initialize Client
if "client" not in st.session_state:
    st.session_state.client = SimulationClient()

if "trajectory_history" not in st.session_state:
    st.session_state.trajectory_history = []

client = st.session_state.client

# Query Latest Telemetry
state_data = client.get_state()
is_connected = (state_data is not None and state_data.get("connected", False))

# Fallback default structures if simulation is not yet running
telemetry = state_data.get("telemetry", {}) if is_connected else {
    "x": DEFAULT_START_POS[0],
    "y": DEFAULT_START_POS[1],
    "theta": 0.0,
    "theta_deg": 0.0,
    "linear_velocity": 0.0,
    "angular_velocity": 0.0,
    "distance_to_goal": round(math.hypot(DEFAULT_GOAL_POS[0] - DEFAULT_START_POS[0], DEFAULT_GOAL_POS[1] - DEFAULT_START_POS[1]), 2),
    "distance_travelled": 0.0,
    "elapsed_time": 0.0,
    "state": RobotState.IDLE.value,
    "status_message": "Simulation offline. Start Webots to begin mission.",
    "start_pos": DEFAULT_START_POS,
    "goal_pos": DEFAULT_GOAL_POS
}

navigation = state_data.get("navigation", {}) if is_connected else {
    "state": RobotState.IDLE.value,
    "original_path": [DEFAULT_START_POS, DEFAULT_GOAL_POS],
    "current_path": [DEFAULT_START_POS, DEFAULT_GOAL_POS],
    "has_replanned": False,
    "replan_count": 0,
    "detected_obstacles_count": 0,
    "min_lidar_distance": 3.5,
    "dynamic_obstacle_active": False,
    "dynamic_obstacle_pos": None,
    "status_message": "Awaiting connection to Webots simulation."
}

lidar_samples = state_data.get("lidar_samples", []) if is_connected else []

# Accumulate breadcrumbs
rx, ry = telemetry.get("x", DEFAULT_START_POS[0]), telemetry.get("y", DEFAULT_START_POS[1])
if is_connected and (len(st.session_state.trajectory_history) == 0 or
   math.hypot(rx - st.session_state.trajectory_history[-1][0], ry - st.session_state.trajectory_history[-1][1]) > 0.05):
    st.session_state.trajectory_history.append([rx, ry])
    # Keep last 250 points
    if len(st.session_state.trajectory_history) > 250:
        st.session_state.trajectory_history.pop(0)

# -------------------------------------------------------------
# SIDEBAR CONTROLS & ENVIRONMENT CONFIGURATION
# -------------------------------------------------------------
with st.sidebar:
    st.markdown("### 🤖 Robot Control Station")
    st.markdown("---")

    # Connection Status Badge
    if is_connected:
        st.markdown('<span class="badge-online">● WEBOTS CONNECTED</span>', unsafe_allow_html=True)
    else:
        st.markdown('<span class="badge-offline">○ WEBOTS DISCONNECTED</span>', unsafe_allow_html=True)
        st.caption("Launch Webots simulation world to begin telemetry streaming.")

    st.markdown("#### 🎮 Mission Navigation")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶ Start Mission", use_container_width=True, type="primary"):
            client.send_command(CommandType.START_NAV)
            st.rerun()

    with c2:
        if st.button("⏹ Stop", use_container_width=True):
            client.send_command(CommandType.STOP_NAV)
            st.rerun()

    c3, c4 = st.columns(2)
    with c3:
        if st.button("⏸ Pause", use_container_width=True):
            client.send_command(CommandType.PAUSE_NAV)
            st.rerun()
    with c4:
        if st.button("▶ Resume", use_container_width=True):
            client.send_command(CommandType.RESUME_NAV)
            st.rerun()

    if st.button("🔄 Reset Environment", use_container_width=True):
        client.send_command(CommandType.RESET)
        st.session_state.trajectory_history = []
        st.rerun()

    st.markdown("---")
    st.markdown("#### 🚨 Dynamic Obstacle Control")
    st.caption("Inject a real 3D physical obstacle into the robot's active navigation path.")

    col_dyn1, col_dyn2 = st.columns(2)
    with col_dyn1:
        if st.button("➕ Add Obstacle", use_container_width=True, help="Places dynamic obstacle directly in front of moving robot"):
            client.send_command(CommandType.ADD_OBSTACLE)
            st.rerun()

    with col_dyn2:
        if st.button("🧹 Clear Obstacle", use_container_width=True):
            client.send_command(CommandType.REMOVE_OBSTACLE)
            st.rerun()

    st.markdown("---")
    st.markdown("#### 🎯 Destination Selection")

    dest_preset = st.selectbox(
        "Destination Preset:",
        ["Top Right (+2.0, +2.0)", "Top Left (-2.0, +2.0)", "Center Right (+2.2, 0.0)", "Bottom Right (+2.0, -2.0)", "Custom Coordinates"]
    )

    preset_map = {
        "Top Right (+2.0, +2.0)": (2.0, 2.0),
        "Top Left (-2.0, +2.0)": (-2.0, 2.0),
        "Center Right (+2.2, 0.0)": (2.2, 0.0),
        "Bottom Right (+2.0, -2.0)": (2.0, -2.0),
    }

    if dest_preset == "Custom Coordinates":
        target_x = st.number_input("Target X (-2.7 to 2.7):", -2.7, 2.7, float(telemetry.get("goal_pos", [2.0, 2.0])[0]), 0.1)
        target_y = st.number_input("Target Y (-2.7 to 2.7):", -2.7, 2.7, float(telemetry.get("goal_pos", [2.0, 2.0])[1]), 0.1)
    else:
        target_x, target_y = preset_map[dest_preset]

    if st.button("📍 Set Target Destination", use_container_width=True):
        client.send_command(CommandType.SET_GOAL, {"goal": [target_x, target_y]})
        st.toast(f"New destination targeted: ({target_x}, {target_y})")
        st.rerun()

# -------------------------------------------------------------
# MAIN DASHBOARD CONTENT
# -------------------------------------------------------------

# Header
title_col, status_col = st.columns([3, 1])
with title_col:
    st.markdown("## 🤖 Autonomous Mobile Robot Mission Control Station")
    st.caption("B.Tech Major Project • Real-Time 3D Simulation • A* Path Planning • Dynamic LiDAR Obstacle Avoidance")

with status_col:
    nav_state = navigation.get("state", RobotState.IDLE.value)
    badge_class = {
        "NAVIGATING": "badge-navigating",
        "REPLANNING": "badge-replan",
        "GOAL_REACHED": "badge-completed",
        "BLOCKED": "badge-offline",
        "IDLE": "badge-online"
    }.get(nav_state, "badge-online")
    st.markdown(f'<div style="text-align: right; margin-top: 15px;"><span class="{badge_class}">STATE: {nav_state}</span></div>', unsafe_allow_html=True)

st.markdown("---")

# Key Metrics Row
m1, m2, m3, m4, m5, m6, m7 = st.columns(7)
with m1:
    render_metric_card("Robot Pose", f"({telemetry.get('x', 0):.2f}, {telemetry.get('y', 0):.2f})", f"{telemetry.get('theta_deg', 0):.1f}°")
with m2:
    render_metric_card("Linear Velocity", f"{telemetry.get('linear_velocity', 0):.2f} m/s", f"ω: {telemetry.get('angular_velocity', 0):.2f} rad/s")
with m3:
    render_metric_card("Dist to Goal", f"{telemetry.get('distance_to_goal', 0):.2f} m", "A* Euclidean")
with m4:
    render_metric_card("Traveled", f"{telemetry.get('distance_travelled', 0):.2f} m", f"Time: {telemetry.get('elapsed_time', 0):.1f} s")
with m5:
    render_metric_card("Replans", f"{navigation.get('replan_count', 0)}", "Dynamic Events")
with m6:
    render_metric_card("Obstacles Seen", f"{navigation.get('detected_obstacles_count', 0)}", "LiDAR Hits")
with m7:
    min_dist = navigation.get("min_lidar_distance", 3.5)
    alert_text = "Safe" if min_dist > 0.5 else ("CAUTION" if min_dist > 0.28 else "EMERGENCY")
    render_metric_card("LiDAR Clearance", f"{min_dist:.2f} m", alert_text)

# System Message Box
msg = navigation.get("status_message", "System standby.")
if "Emergency" in msg or "BLOCKED" in nav_state:
    st.error(f"⚠️ {msg}")
elif "Replanning" in msg or "Replaced" in msg or nav_state == "REPLANNING":
    st.warning(f"🔄 {msg}")
elif nav_state == "GOAL_REACHED":
    st.success(f"🎉 {msg}")
else:
    st.info(f"ℹ️ {msg}")

# Visualizations Layout
col_map, col_lidar = st.columns([1.5, 1])

with col_map:
    fig_map = create_navigation_plot(telemetry, navigation, st.session_state.trajectory_history)
    st.plotly_chart(fig_map, use_container_width=True)

with col_lidar:
    fig_lidar = create_lidar_polar_plot(lidar_samples, navigation.get("min_lidar_distance", 3.5))
    st.plotly_chart(fig_lidar, use_container_width=True)

    # Dynamic Obstacle Details Card
    st.markdown("##### 📍 Dynamic Obstacle Telemetry")
    if navigation.get("dynamic_obstacle_active", False):
        obs_coord = navigation.get("dynamic_obstacle_pos", [0, 0])
        st.markdown(f"**Status:** 🔴 Active in Arena | **Position:** `({obs_coord[0]}, {obs_coord[1]})`")
        st.markdown("**Replanning Status:** ⚡ A* Path Recalculated around Obstacle")
    else:
        st.markdown("**Status:** ⚪ Inactive (Staged below arena floor)")
        st.caption("Click 'Add Obstacle' in the sidebar while the robot is navigating to trigger live avoidance demonstration.")

# Auto-refresh loop (polls every 1.2 seconds for live telemetry)
time.sleep(1.2)
st.rerun()
