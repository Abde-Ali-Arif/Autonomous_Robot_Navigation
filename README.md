# Autonomous Mobile Robot Navigation and Dynamic Obstacle Avoidance Using 3D Simulation

**B.Tech Major Engineering Project** • Software-Only Robotics Implementation

An autonomous mobile robot simulation system featuring 3D indoor physics, virtual 360° LiDAR scanning, A* global path planning, dynamic real-time obstacle avoidance, and an interactive Streamlit Mission Control Station.

---

## 🌟 Key Features

- **Pure Software Implementation:** Zero hardware dependency (no physical microcontrollers, motors, or sensors needed).
- **Realistic 3D Simulation:** Powered by Cyberbotics Webots with differential-drive kinematics, mass properties, and 360° planar LiDAR rays.
- **A\* Pathfinding Algorithm:** 8-connectivity grid planning with diagonal corner-cutting prevention and Euclidean distance heuristics.
- **Obstacle Inflation Layer:** Dynamic safety boundary calculation ensuring the robot body (diameter 0.32 m) never grazes walls or obstacles.
- **True Physical Dynamic Obstacle Avoidance:** User injects a real 3D physical object into the arena via the Webots Supervisor API. The robot's simulated LiDAR rays physically detect the obstruction, halt the robot, update the occupancy grid, and execute real-time A* replanning.
- **Modern Mission Control Dashboard:** Built with Streamlit and Plotly, displaying live robot odometry, 360° radar scans, route comparisons (original vs. replanned), and mission analytics.
- **Dual-Channel IPC Communication:** High-speed TCP sockets (`127.0.0.1:5050`) coupled with an atomic JSON file mirror (`logs/telemetry.json`) for 100% resilience against Windows Defender firewall blocks.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│               STREAMLIT MISSION CONTROL                 │
│         (http://localhost:8501 • dashboard/app.py)      │
│  - Real-time Odometry (X, Y, Theta, Velocities)         │
│  - Interactive 2D Grid Map & Trajectory Plot            │
│  - 360° Polar LiDAR Scan Radar Visualizer               │
│  - Controls: Start, Pause, Resume, Reset, Add Obstacle   │
└───────────────────────────┬─────────────────────────────┘
                            │
               [ Dual-Channel IPC Layer ]
        TCP Socket (5050) + Atomic File Mirror
                            │
┌───────────────────────────▼─────────────────────────────┐
│                 WEBOTS ROBOT CONTROLLER                 │
│      (controllers/autonomous_controller/main.py)        │
│                                                         │
│   ┌─────────────────────────────────────────────────┐   │
│   │                 NAVIGATION CORE                 │   │
│   │  - Occupancy Grid Mapping (60x60, 0.10m res)    │   │
│   │  - A* Planner (8-connectivity, Corner Safe)     │   │
│   │  - LiDAR Sector Analyzer & Front Cone Filter    │   │
│   │  - Differential Proportional Waypoint Follower  │   │
│   └───────────────────────┬─────────────────────────┘   │
│                           │                             │
│   ┌───────────────────────▼─────────────────────────┐   │
│   │            PHYSICAL SENSORS & ACTUATORS         │   │
│   │  - Left & Right Wheel Motors (RotationalMotor)  │   │
│   │  - 360° Planar Virtual LiDAR (360 beams)        │   │
│   │  - Supervisor Dynamic Obstacle Injector         │   │
│   └─────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────┐
│              3D SIMULATION ARENA (Webots)               │
│  - 6m x 6m Enclosed Indoor Arena (ENU Coordinates)      │
│  - Differential Mobile Robot (Drive Wheels + Casters)   │
│  - Static Obstacles (Desks/Cabinets) & Dynamic Box      │
└─────────────────────────────────────────────────────────┘
```

---

## 💻 Tech Stack & Requirements

- **Operating System:** Windows 10/11 (64-bit)
- **Simulator:** Cyberbotics Webots R2023b or R2024a
- **Python:** Python 3.12 (64-bit)
- **Libraries:**
  - `numpy` (Occupancy grid matrices)
  - `matplotlib` & `plotly` (Visualizations)
  - `streamlit` (Mission Control Dashboard)
  - `pytest` (Automated unit testing)

---

## 🚀 Quick Start Guide (Windows)

### 1. Prerequisites Check
Ensure Webots is installed at `C:\Program Files\Webots`.

### 2. Configure Webots Python Runtime
1. Open Webots.
2. Navigate to **Tools** → **Preferences** → **General**.
3. In **Python Command**, type:
   ```text
   C:\Users\Hp\AppData\Local\Programs\Python\Python312\python.exe
   ```
4. Click **OK**.

### 3. Run the Application

#### Option A: One-Click PowerShell Launcher (Recommended)
Open PowerShell in the project directory and execute:
```powershell
Set-Location "d:\Project Hub\Major Project"
.\launch.ps1
```

#### Option B: Python Master Runner
```powershell
Set-Location "d:\Project Hub\Major Project"
.\.venv\Scripts\Activate.ps1
python main.py
```

#### Option C: Run Components Separately
1. **Launch Webots World:**
   Double-click `worlds/indoor_navigation.wbt` or run:
   ```powershell
   & "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" "d:\Project Hub\Major Project\worlds\indoor_navigation.wbt"
   ```
2. **Launch Streamlit Dashboard:**
   In a separate terminal:
   ```powershell
   Set-Location "d:\Project Hub\Major Project"
   .\.venv\Scripts\Activate.ps1
   streamlit run dashboard/app.py
   ```

---

## 🎓 College Live Demonstration Walkthrough

Follow these steps for a live presentation to examiners and faculty:

1. **Initial State Display:**
   - Launch the project. Point out the Webots 3D simulation showing the 6m x 6m room and differential-drive robot at `(-2.0, -2.0)`.
   - Show the Streamlit Dashboard at `http://localhost:8501`. Note the status `● WEBOTS CONNECTED` and the initial blue dashed A* path to destination `(2.0, 2.0)`.
2. **Start Autonomous Mission:**
   - Click **`▶ Start Mission`** on the dashboard sidebar.
   - Observe the robot rotate toward the first waypoint and begin smooth differential drive movement in Webots.
   - Point out the real-time odometry updates (X, Y, Heading, Velocity, LiDAR clearance) updating live on the dashboard.
3. **Dynamic Obstacle Injection (Key Project Demonstration):**
   - While the robot is moving along its route, click **`➕ Add Obstacle`** on the dashboard.
   - Explain: *"A real 3D physical box has now been injected into the Webots arena directly along the robot's active path."*
   - Observe the virtual LiDAR beam hit the obstacle in Webots.
   - The robot halts immediately before collision (safety boundary clearance).
   - The status updates to `🔄 REPLANNING`.
   - The occupancy grid updates, and A* calculates a new collision-free detour.
   - The dashboard updates with a new orange solid route line and increments the **Replans** counter.
4. **Resumed Autonomous Navigation:**
   - The robot steers around the obstacle without human intervention.
   - Show the traveled breadcrumb line and the 360° LiDAR radar scan reflecting the obstacle's surface.
5. **Mission Completion:**
   - The robot reaches destination `(2.0, 2.0)`.
   - Both motors stop cleanly.
   - Dashboard displays `STATE: GOAL_REACHED` along with mission statistics: total traveled distance, elapsed time, replanning count, and obstacle detections.

---

## 🧪 Running Automated Unit Tests

All navigation, pathfinding, kinematic, and sensor processing modules include unit tests:
```powershell
Set-Location "d:\Project Hub\Major Project"
.\.venv\Scripts\pytest.exe -v
```

All 10 tests will execute and confirm:
- A* path generation and obstacle avoidance
- Diagonal corner-cutting prevention
- Unreachable goal detection
- LiDAR range filtering and emergency stop triggers
- Waypoint tracking kinematics and arrival tolerance

---

## 📁 Repository Structure

```
autonomous-robot-navigation/
│
├── config/
│   ├── __init__.py
│   └── settings.py              # Environment dimensions, speeds, LiDAR parameters
│
├── communication/
│   ├── __init__.py
│   ├── protocol.py              # JSON protocol, RobotState & CommandType definitions
│   ├── server.py                # Dual-channel TCP + File IPC server
│   └── client.py                # Resilient client for dashboard
│
├── navigation/
│   ├── __init__.py
│   ├── occupancy_grid.py        # 2D Grid mapping, coordinate transforms, inflation
│   ├── astar.py                 # A* pathfinder with corner clearance
│   ├── obstacle_detection.py    # LiDAR scan filtering & path corridor checking
│   ├── waypoint_follower.py     # Proportional differential drive controller
│   └── path_planner.py          # High-level route planning and replanning
│
├── controllers/
│   └── autonomous_controller/
│       ├── autonomous_controller.py  # Webots Supervisor main controller
│       ├── motor_controller.py       # Differential wheel motor actuation
│       ├── lidar_processor.py        # Webots LiDAR sensor interface
│       ├── navigation_controller.py  # Finite state machine
│       └── robot_state.py            # Telemetry and metrics tracker
│
├── worlds/
│   └── indoor_navigation.wbt    # 3D Webots indoor arena and robot model
│
├── dashboard/
│   ├── app.py                   # Streamlit mission control application
│   ├── components.py            # Custom CSS cards and KPI widgets
│   └── visualization.py         # Plotly 2D arena map and 360° LiDAR polar chart
│
├── tests/
│   ├── test_astar.py            # Pathfinding validation tests
│   ├── test_obstacle_detection.py # LiDAR sensor and safety zone tests
│   └── test_navigation.py       # Kinematics and coordinate transform tests
│
├── launch.ps1                   # One-click Windows PowerShell launcher
├── main.py                      # Python cross-component runner
├── requirements.txt             # Pinned Python package dependencies
├── pytest.ini                   # Pytest test discovery configuration
└── README.md                    # Project documentation
```
