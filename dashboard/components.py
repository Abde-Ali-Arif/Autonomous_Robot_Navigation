"""
UI Components and Styling for Streamlit Robotics Mission Control Station.
"""

import streamlit as st


def apply_custom_css():
    """Apply modern dark-themed robotics control station CSS styling."""
    st.markdown("""
        <style>
            /* Main background styling */
            .main {
                background-color: #0e1117;
            }
            /* Metric Card styling */
            .metric-card {
                background-color: #1a1e27;
                border-radius: 8px;
                padding: 12px 16px;
                border: 1px solid #2a313d;
                box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
                margin-bottom: 10px;
            }
            .metric-title {
                color: #8b949e;
                font-size: 11px;
                font-weight: 600;
                text-transform: uppercase;
                letter-spacing: 0.5px;
            }
            .metric-value {
                color: #ffffff;
                font-size: 22px;
                font-weight: 700;
                margin-top: 4px;
            }
            /* Status badges */
            .badge-online {
                background-color: #1b4332;
                color: #40916c;
                padding: 4px 10px;
                border-radius: 12px;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #2d6a4f;
            }
            .badge-offline {
                background-color: #49111c;
                color: #f72585;
                padding: 4px 10px;
                border-radius: 12px;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #800f2f;
            }
            .badge-navigating {
                background-color: #003049;
                color: #669bbc;
                padding: 4px 10px;
                border-radius: 12px;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #00507a;
            }
            .badge-replan {
                background-color: #663d00;
                color: #ffb703;
                padding: 4px 10px;
                border-radius: 12px;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #995c00;
            }
            .badge-completed {
                background-color: #143601;
                color: #55a630;
                padding: 4px 10px;
                border-radius: 12px;
                font-weight: 600;
                font-size: 12px;
                border: 1px solid #2d6a4f;
            }
        </style>
    """, unsafe_allow_html=True)


def render_metric_card(title: str, value: str, subvalue: str = ""):
    """Render a styled robotics telemetry card."""
    sub_html = f"<div style='color: #52b788; font-size: 11px; margin-top: 2px;'>{subvalue}</div>" if subvalue else ""
    st.markdown(f"""
        <div class="metric-card">
            <div class="metric-title">{title}</div>
            <div class="metric-value">{value}</div>
            {sub_html}
        </div>
    """, unsafe_allow_html=True)
