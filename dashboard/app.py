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

# Initialize Client and State
if "client" not in st.session_state:
    st.session_state.client = SimulationClient()

if "trajectory_history" not in st.session_state:
    st.session_state.trajectory_history = []

client = st.session_state.client

def is_valid_destination(tx: float, ty: float) -> tuple[bool, str]:
    """Validate destination coordinates against arena walls and static obstacles."""
    if not (-2.50 <= tx <= 2.50 and -2.50 <= ty <= 2.50):
        return False, f"Coordinates ({tx:.2f}, {ty:.2f}) are outside safe arena bounds [-2.5, +2.5] m."

    margin = 0.30  # Robot radius + safety clearance
    for obs in STATIC_OBSTACLES:
        ox, oy = obs["x"], obs["y"]
        sx, sy = obs["size_x"], obs["size_y"]
        if (ox - sx / 2.0 - margin) <= tx <= (ox + sx / 2.0 + margin) and \
           (oy - sy / 2.0 - margin) <= ty <= (oy + sy / 2.0 + margin):
            return False, f"Coordinates ({tx:.2f}, {ty:.2f}) overlap static obstacle '{obs.get('id', 'box')}'!"
    return True, "Valid"


# -------------------------------------------------------------
# SIDEBAR CONTROLS & ENVIRONMENT CONFIGURATION
# -------------------------------------------------------------
with st.sidebar:
    st.markdown("### 🤖 Robot Control Station")
    st.markdown("---")

    # Connection Status indicator (Refreshed on user interaction)
    currently_connected = client.is_connected()
    if currently_connected:
        st.markdown('<span class="badge-online">● WEBOTS CONNECTED</span>', unsafe_allow_html=True)
    else:
        st.markdown('<span class="badge-offline">○ WEBOTS DISCONNECTED</span>', unsafe_allow_html=True)
        st.caption("Start Webots simulation world to begin live telemetry streaming.")

    st.markdown("#### 🎮 Mission Navigation")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶ Start Mission", use_container_width=True, type="primary"):
            success = client.send_command(CommandType.START_NAV)
            if success:
                st.toast("Command Sent: Start Mission")
            st.rerun()

    with c2:
        if st.button("⏹ Stop", use_container_width=True):
            client.send_command(CommandType.STOP_NAV)
            st.toast("Command Sent: Stop")
            st.rerun()

    c3, c4 = st.columns(2)
    with c3:
        if st.button("⏸ Pause", use_container_width=True):
            client.send_command(CommandType.PAUSE_NAV)
            st.toast("Command Sent: Pause")
            st.rerun()
    with c4:
        if st.button("▶ Resume", use_container_width=True):
            client.send_command(CommandType.RESUME_NAV)
            st.toast("Command Sent: Resume")
            st.rerun()

    if st.button("🔄 Reset Environment", use_container_width=True):
        client.send_command(CommandType.RESET)
        st.session_state.trajectory_history = []
        st.toast("Environment reset to initial state.")
        st.rerun()

    st.markdown("---")
    st.markdown("#### 🚨 Dynamic Obstacle Control")
    st.caption("Inject a real 3D physical obstacle into the robot's active navigation path.")

    obs_mode = st.radio(
        "Obstacle Placement Mode:",
        ["Automatic (Ahead on Path)", "Custom Coordinates"],
        index=0,
        horizontal=True
    )

    obs_x, obs_y = 0.2, 0.2
    if obs_mode == "Custom Coordinates":
        col_ox, col_oy = st.columns(2)
        with col_ox:
            obs_x = st.number_input("Obstacle X (-2.4 to +2.4):", min_value=-2.4, max_value=2.4, value=0.2, step=0.1)
        with col_oy:
            obs_y = st.number_input("Obstacle Y (-2.4 to +2.4):", min_value=-2.4, max_value=2.4, value=0.2, step=0.1)

    col_dyn1, col_dyn2 = st.columns(2)
    with col_dyn1:
        if st.button("➕ Add Obstacle", use_container_width=True, help="Injects dynamic obstacle into simulation"):
            if obs_mode == "Custom Coordinates":
                client.send_command(CommandType.ADD_OBSTACLE, {"x": round(obs_x, 2), "y": round(obs_y, 2)})
                st.toast(f"Dynamic obstacle injected at ({obs_x:.2f}, {obs_y:.2f})!")
            else:
                client.send_command(CommandType.ADD_OBSTACLE)
                st.toast("Dynamic obstacle placed ahead along robot route!")
            st.rerun()

    with col_dyn2:
        if st.button("🧹 Clear Obstacle", use_container_width=True):
            client.send_command(CommandType.REMOVE_OBSTACLE)
            st.toast("Dynamic Obstacle removed.")
            st.rerun()

    st.markdown("---")
    st.markdown("#### 🎯 Destination Selection")

    dest_preset = st.selectbox(
        "Destination Preset:",
        ["Top Right (+2.0, +2.0)", "Top Left (-2.0, +2.0)", "Center Right (+2.2, 0.0)", "Center Room (0.0, 0.0)", "Bottom Right (+2.0, -2.0)", "Custom Coordinates"]
    )

    preset_map = {
        "Top Right (+2.0, +2.0)": (2.0, 2.0),
        "Top Left (-2.0, +2.0)": (-2.0, 2.0),
        "Center Right (+2.2, 0.0)": (2.2, 0.0),
        "Center Room (0.0, 0.0)": (0.0, 0.0),
        "Bottom Right (+2.0, -2.0)": (2.0, -2.0),
    }

    if dest_preset == "Custom Coordinates":
        target_x = st.number_input("Target X (-2.5 to +2.5):", min_value=-2.5, max_value=2.5, value=2.0, step=0.1)
        target_y = st.number_input("Target Y (-2.5 to +2.5):", min_value=-2.5, max_value=2.5, value=2.0, step=0.1)
    else:
        target_x, target_y = preset_map[dest_preset]

    if st.button("📍 Set Target Destination", use_container_width=True):
        valid, reason = is_valid_destination(target_x, target_y)
        if not valid:
            st.error(f"❌ Cannot set destination: {reason}")
        else:
            client.send_command(CommandType.SET_GOAL, {"goal": [round(target_x, 2), round(target_y, 2)]})
            st.toast(f"New destination targeted: ({target_x:.2f}, {target_y:.2f})")
            st.rerun()

# -------------------------------------------------------------
# MAIN DASHBOARD CONTENT (HIGH-PERFORMANCE STREAMLIT FRAGMENT)
# -------------------------------------------------------------

@st.fragment(run_every=1.0)
def render_live_telemetry():
    """
    Asynchronously queries Webots simulation telemetry and renders
    real-time charts and KPI metrics without reloading the whole page.
    """
    state_data = client.get_state()
    is_conn = (state_data is not None and state_data.get("connected", False))

    telemetry = state_data.get("telemetry", {}) if is_conn else {
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

    navigation = state_data.get("navigation", {}) if is_conn else {
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

    lidar_samples = state_data.get("lidar_samples", []) if is_conn else []

    # Update breadcrumb trajectory
    rx, ry = telemetry.get("x", DEFAULT_START_POS[0]), telemetry.get("y", DEFAULT_START_POS[1])
    if is_conn and (len(st.session_state.trajectory_history) == 0 or
       math.hypot(rx - st.session_state.trajectory_history[-1][0], ry - st.session_state.trajectory_history[-1][1]) > 0.04):
        st.session_state.trajectory_history.append([rx, ry])
        if len(st.session_state.trajectory_history) > 300:
            st.session_state.trajectory_history.pop(0)

    # Header Row
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

    # Metrics Row
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

    # Status / Alert Notification Banner
    msg = navigation.get("status_message", "System standby.")
    if "Emergency" in msg or "BLOCKED" in nav_state:
        st.error(f"⚠️ {msg}")
    elif "Replanning" in msg or "Rotating" in msg or nav_state == "REPLANNING":
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


# Execute live telemetry fragment
render_live_telemetry()
