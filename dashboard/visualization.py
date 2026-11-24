"""
Plotly Visualizations for Robot Environment, Navigation Routes, and LiDAR Scans.
"""

import math
from typing import Dict, Any, List, Optional
import numpy as np
import plotly.graph_objects as go

from config.settings import (
    WORLD_X_MIN, WORLD_X_MAX, WORLD_Y_MIN, WORLD_Y_MAX,
    STATIC_OBSTACLES, EMERGENCY_STOP_DISTANCE, SLOWDOWN_DISTANCE
)


def create_navigation_plot(telemetry: Dict[str, Any],
                           navigation: Dict[str, Any],
                           trajectory_history: List[List[float]]) -> go.Figure:
    """
    Generate an interactive 2D map showing arena boundaries, static obstacles,
    dynamic obstacle, planned routes (original vs replanned), and robot pose.
    """
    fig = go.Figure()

    # 1. Outer Arena Boundary Walls
    fig.add_shape(
        type="rect",
        x0=WORLD_X_MIN, y0=WORLD_Y_MIN,
        x1=WORLD_X_MAX, y1=WORLD_Y_MAX,
        line=dict(color="#4a5568", width=3),
        fillcolor="rgba(15, 23, 42, 0.6)"
    )

    # 2. Static Obstacles
    for obs in STATIC_OBSTACLES:
        ox, oy = obs["x"], obs["y"]
        sx, sy = obs["size_x"], obs["size_y"]
        fig.add_shape(
            type="rect",
            x0=ox - sx / 2.0, y0=oy - sy / 2.0,
            x1=ox + sx / 2.0, y1=oy + sy / 2.0,
            line=dict(color="#2563eb", width=2),
            fillcolor="rgba(37, 99, 235, 0.45)"
        )
        fig.add_annotation(
            x=ox, y=oy,
            text=obs["id"].replace("obs_box_", "Box "),
            showarrow=False,
            font=dict(color="#93c5fd", size=9)
        )

    # 3. Dynamic Obstacle (if active in simulation)
    dyn_pos = navigation.get("dynamic_obstacle_pos")
    if navigation.get("dynamic_obstacle_active", False) and dyn_pos:
        d_x, d_y = dyn_pos[0], dyn_pos[1]
        fig.add_shape(
            type="rect",
            x0=d_x - 0.225, y0=d_y - 0.225,
            x1=d_x + 0.225, y1=d_y + 0.225,
            line=dict(color="#ef4444", width=3),
            fillcolor="rgba(239, 68, 68, 0.75)"
        )
        fig.add_annotation(
            x=d_x, y=d_y,
            text="DYNAMIC OBSTACLE",
            showarrow=True,
            arrowhead=2,
            arrowcolor="#ef4444",
            ax=0, ay=-30,
            font=dict(color="#fca5a5", size=10, family="sans-serif")
        )

    # 4. Trajectory Breadcrumb History
    if trajectory_history and len(trajectory_history) > 1:
        tx = [p[0] for p in trajectory_history]
        ty = [p[1] for p in trajectory_history]
        fig.add_trace(go.Scatter(
            x=tx, y=ty,
            mode="lines",
            line=dict(color="rgba(74, 222, 128, 0.4)", width=2, dash="dot"),
            name="Traveled Path",
            hoverinfo="skip"
        ))

    # 5. Original Planned A* Route (Blue dashed line)
    orig_path = navigation.get("original_path", [])
    if orig_path and len(orig_path) > 1:
        ox = [p[0] for p in orig_path]
        oy = [p[1] for p in orig_path]
        fig.add_trace(go.Scatter(
            x=ox, y=oy,
            mode="lines+markers",
            line=dict(color="#38bdf8", width=2, dash="dash"),
            marker=dict(size=5, color="#38bdf8"),
            name="Original Route",
            hoverinfo="text",
            hovertext=[f"Waypoint {i}: ({round(p[0], 2)}, {round(p[1], 2)})" for i, p in enumerate(orig_path)]
        ))

    # 6. Updated Replanned A* Route (Gold/Orange solid line)
    curr_path = navigation.get("current_path", [])
    if navigation.get("has_replanned", False) and curr_path and len(curr_path) > 1:
        cx = [p[0] for p in curr_path]
        cy = [p[1] for p in curr_path]
        fig.add_trace(go.Scatter(
            x=cx, y=cy,
            mode="lines+markers",
            line=dict(color="#f59e0b", width=3.5),
            marker=dict(size=6, color="#f59e0b"),
            name="Replanned Route",
            hoverinfo="text",
            hovertext=[f"Replanned WP {i}: ({round(p[0], 2)}, {round(p[1], 2)})" for i, p in enumerate(curr_path)]
        ))

    # 7. Start and Goal Markers
    start_pos = telemetry.get("start_pos", [-2.0, -2.0])
    goal_pos = telemetry.get("goal_pos", [2.0, 2.0])

    fig.add_trace(go.Scatter(
        x=[start_pos[0]], y=[start_pos[1]],
        mode="markers+text",
        marker=dict(size=14, color="#10b981", symbol="circle"),
        text=["START"],
        textposition="bottom center",
        name="Start Point",
        hoverinfo="text",
        hovertext=f"Start: ({start_pos[0]}, {start_pos[1]})"
    ))

    fig.add_trace(go.Scatter(
        x=[goal_pos[0]], y=[goal_pos[1]],
        mode="markers+text",
        marker=dict(size=16, color="#fbbf24", symbol="star"),
        text=["GOAL"],
        textposition="top center",
        name="Destination",
        hoverinfo="text",
        hovertext=f"Goal: ({goal_pos[0]}, {goal_pos[1]})"
    ))

    # 8. Robot Current Pose (Marker with directional orientation line)
    rx, ry = telemetry.get("x", start_pos[0]), telemetry.get("y", start_pos[1])
    theta = telemetry.get("theta", 0.0)

    # Robot body
    fig.add_trace(go.Scatter(
        x=[rx], y=[ry],
        mode="markers",
        marker=dict(size=18, color="#22c55e", line=dict(color="#ffffff", width=2)),
        name="Robot Pose",
        hoverinfo="text",
        hovertext=f"Robot: ({rx}, {ry}) | Heading: {round(math.degrees(theta), 1)}°"
    ))

    # Heading Arrow tip
    arrow_len = 0.35
    ax_end = rx + arrow_len * math.cos(theta)
    ay_end = ry + arrow_len * math.sin(theta)
    fig.add_trace(go.Scatter(
        x=[rx, ax_end], y=[ry, ay_end],
        mode="lines",
        line=dict(color="#ffffff", width=3),
        showlegend=False,
        hoverinfo="skip"
    ))

    # Layout styling
    fig.update_layout(
        title="Indoor Arena Occupancy Grid & Navigation Trajectory",
        title_font=dict(color="#f8fafc", size=14),
        paper_bgcolor="#0f172a",
        plot_bgcolor="#0b0f19",
        xaxis=dict(
            range=[WORLD_X_MIN - 0.3, WORLD_X_MAX + 0.3],
            zeroline=False,
            gridcolor="#1e293b",
            title="X (East) [meters]",
            color="#94a3b8"
        ),
        yaxis=dict(
            range=[WORLD_Y_MIN - 0.3, WORLD_Y_MAX + 0.3],
            zeroline=False,
            gridcolor="#1e293b",
            title="Y (North) [meters]",
            color="#94a3b8",
            scaleanchor="x",
            scaleratio=1
        ),
        legend=dict(
            bgcolor="rgba(15, 23, 42, 0.8)",
            bordercolor="#334155",
            borderwidth=1,
            font=dict(color="#cbd5e1", size=11),
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        ),
        margin=dict(l=40, r=40, t=60, b=40),
        height=520
    )

    return fig


def create_lidar_polar_plot(ranges: List[float], min_dist: float) -> go.Figure:
    """
    Generate an interactive 360-degree LiDAR radar polar scan visualization.
    """
    fig = go.Figure()

    if not ranges or len(ranges) == 0:
        # Default empty circle
        theta_angles = np.linspace(0, 360, 72)
        r_vals = [3.5] * 72
    else:
        num_beams = len(ranges)
        theta_angles = np.linspace(-180, 180, num_beams, endpoint=False)
        r_vals = [min(3.5, max(0.06, r)) if not (math.isnan(r) or math.isinf(r)) else 3.5 for r in ranges]

    # LiDAR range trace
    fig.add_trace(go.Scatterpolar(
        r=r_vals,
        theta=theta_angles,
        mode="markers",
        marker=dict(size=3.5, color=r_vals, colorscale="Viridis", cmin=0.1, cmax=3.5),
        name="LiDAR Beam",
        hoverinfo="text",
        hovertext=[f"Angle: {round(a, 1)}° | Dist: {round(d, 2)}m" for a, d in zip(theta_angles, r_vals)]
    ))

    # Safety Zone Circles
    fig.add_trace(go.Scatterpolar(
        r=[EMERGENCY_STOP_DISTANCE] * 60,
        theta=np.linspace(0, 360, 60),
        mode="lines",
        line=dict(color="#ef4444", width=1.5, dash="dash"),
        name="Stop Limit (0.28m)",
        hoverinfo="skip"
    ))

    fig.add_trace(go.Scatterpolar(
        r=[SLOWDOWN_DISTANCE] * 60,
        theta=np.linspace(0, 360, 60),
        mode="lines",
        line=dict(color="#f59e0b", width=1.5, dash="dot"),
        name="Slow Limit (0.50m)",
        hoverinfo="skip"
    ))

    fig.update_layout(
        title="360° LiDAR Real-Time Range Scan",
        title_font=dict(color="#f8fafc", size=14),
        polar=dict(
            radialaxis=dict(
                visible=True,
                range=[0, 3.5],
                gridcolor="#1e293b",
                linecolor="#334155",
                color="#94a3b8"
            ),
            angularaxis=dict(
                gridcolor="#1e293b",
                linecolor="#334155",
                color="#94a3b8",
                direction="clockwise",
                rotation=90
            ),
            bgcolor="#0b0f19"
        ),
        paper_bgcolor="#0f172a",
        font=dict(color="#cbd5e1"),
        legend=dict(
            bgcolor="rgba(15, 23, 42, 0.8)",
            font=dict(color="#cbd5e1", size=10),
            orientation="h",
            yanchor="bottom",
            y=-0.2,
            xanchor="center",
            x=0.5
        ),
        margin=dict(l=30, r=30, t=50, b=50),
        height=380
    )

    return fig
