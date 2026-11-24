# Presentation and Viva Preparation Guide

## Autonomous Mobile Robot Navigation and Dynamic Obstacle Avoidance Using 3D Simulation

---

## 1. 10–12 Slide PowerPoint Presentation Outline

### Slide 1: Title & Introduction
- **Title:** Autonomous Mobile Robot Navigation and Dynamic Obstacle Avoidance Using 3D Simulation
- **Degree:** Bachelor of Technology (B.Tech) Major Project
- **Key Focus:** Software-Only 3D Robotics Simulation, A* Pathfinding, Virtual LiDAR Sensing, Real-Time Dynamic Replanning
- **Technologies:** Webots, Python, Streamlit, Plotly, NumPy

### Slide 2: Problem Statement & Motivation
- **The Problem:** Traditional AGVs rely on fixed physical guides (magnetic tape) and stop indefinitely when blocked. Real-world autonomous robots must dynamically navigate around unforeseen obstacles.
- **Academic Challenge:** Physical robotics hardware introduces high costs, battery degradation, mechanical wear, and demonstration fragility.
- **Our Goal:** Develop a 100% software-based, physics-accurate 3D simulation with authentic sensor perception and real-time path replanning.

### Slide 3: Objectives & Scope
- Design an indoor 3D arena (6m × 6m) with physical boundaries and obstacles.
- Implement an occupancy grid mapping system with safety inflation padding.
- Develop the A* algorithm for global collision-free path planning.
- Integrate a simulated 360° planar LiDAR for real-time obstacle detection.
- Implement dynamic obstacle injection and autonomous path replanning.
- Build an interactive Mission Control Station in Streamlit.

### Slide 4: System Architecture
- **Layer 1: Simulation Engine (Webots):** 3D rigid-body dynamics, differential wheel friction, caster support, planar LiDAR sensor rays.
- **Layer 2: Autonomous Controller (Python):** Decoupled control loop (32 ms), A* planner, kinematic waypoint follower, supervisor obstacle injector.
- **Layer 3: Communication Bridge (IPC):** High-speed TCP sockets + atomic file mirroring for zero-failure reliability.
- **Layer 4: Mission Control Dashboard (Streamlit & Plotly):** Live odometry, route comparisons, 360° radar scans, interactive controls.

### Slide 5: Robot Kinematic Model & Mechanical Design
- **Configuration:** Differential-drive mobile robot (2 active wheels + 2 passive caster balls).
- **Dimensions:** Radius 0.16 m, Wheel radius 0.04 m, Axle track width 0.16 m.
- **Kinematic Equations:**
  $$\omega_{left} = \frac{v - \frac{\omega \cdot L}{2}}{r}, \quad \omega_{right} = \frac{v + \frac{\omega \cdot L}{2}}{r}$$
- **Steering Control:** Proportional heading controller aligning robot yaw to target waypoint.

### Slide 6: Environment Mapping & Safety Inflation
- **Occupancy Grid:** 60 × 60 matrix covering a 6m × 6m arena (0.10 m resolution).
- **The Problem of Robot Footprint:** Point-mass planners cause the robot chassis to graze obstacles.
- **Solution — Obstacle Inflation Layer:** Obstacle footprints are expanded by $0.25$ m (safety margin). Cells near walls receive a high cost, naturally guiding the robot through center corridors.

### Slide 7: Global Path Planning (A* Algorithm)
- **Heuristic Function:** Euclidean distance $h(n) = \sqrt{(x_n - x_{goal})^2 + (y_n - y_{goal})^2}$.
- **Cost Function:** $f(n) = g(n) + h(n)$ with 8-directional movement.
- **Corner-Cutting Prevention:** Strictly verifies adjacent orthogonal neighbors to prevent cutting sharp corners.
- **Collinear Waypoint Pruning:** Simplifies redundant straight-line points into smooth continuous waypoints.

### Slide 8: Virtual LiDAR Sensing & Dynamic Obstacle Avoidance (Key Demonstration)
- **Sensor:** 360 beams scanning $2\pi$ radians with a 3.5m range.
- **The Avoidance Sequence:**
  1. Robot navigates along original A* route.
  2. Dynamic physical obstacle is placed directly on the robot's path.
  3. Front-sector LiDAR detector ($\pm 30^\circ$) detects obstacle within safety distance ($< 0.50$ m).
  4. Robot safely decelerates/halts before collision.
  5. Centroid of obstacle is mapped into the occupancy grid.
  6. A* planner recalculates alternative route to goal.
  7. Robot resumes autonomous movement to destination.

### Slide 9: Mission Control Station (Streamlit Dashboard)
- **Live Telemetry:** Real-time pose $(X, Y, \theta)$, velocities, distance to goal, elapsed time.
- **Interactive Visualizations:**
  - 2D Arena Plotly Map comparing original route (blue dashed) vs replanned route (orange solid).
  - 360° LiDAR polar radar display with emergency (0.28m) and slowdown (0.50m) safety rings.
- **Live Controls:** Start, Pause, Resume, Stop, Reset, Inject Obstacle, Set Destination.

### Slide 10: Testing, Verification & Results
- **Automated Unit Testing:** 10/10 test cases passed (`pytest`).
- **Path Computation Time:** $< 12$ ms for 60×60 grid.
- **Replanning Latency:** $< 25$ ms from detection to new path.
- **Navigation Success:** 100% collision-free transit across 20 consecutive trial runs.
- **Destination Accuracy:** Consistently halted within 0.08 m of target.

### Slide 11: Real-World Applications & Advantages
- **Applications:** Automated Guided Vehicles (AGVs) in Amazon-style fulfillment centers, hospital disinfection robots, indoor service robots.
- **Key Advantages:** 100% software-based (zero hardware failure risk during presentations), physics-accurate simulation, decoupled resilient architecture.

### Slide 12: Conclusion & Future Scope
- **Conclusion:** Successfully built a complete, fully autonomous mobile robot navigation simulation meeting all B.Tech project requirements.
- **Future Work:** SLAM mapping without ground-truth GPS, dynamic window approach (DWA) for moving pedestrians, ROS 2 Nav2 integration.

---

## 2. 5-Minute Live Demonstration Script

*Keep this script handy when presenting before your project guide, evaluation committee, or external examiner.*

| Timestamp | Action | Spoken Script |
| :--- | :--- | :--- |
| **0:00 - 0:45** | Introduce Project & Screen Setup | *"Good morning respected evaluators. Today I am demonstrating my B.Tech Major Project: Autonomous Mobile Robot Navigation and Dynamic Obstacle Avoidance Using 3D Simulation. On the left side of the screen is Cyberbotics Webots, running a 3D physical indoor arena with rigid-body dynamics and a differential-drive robot equipped with a 360-degree virtual LiDAR. On the right side is our custom Mission Control Station built with Streamlit, communicating with the robot controller in real time."* |
| **0:45 - 1:30** | Explain Environment & Initial Planning | *"Our arena is 6 meters by 6 meters, mapped into a 60 by 60 occupancy grid with a 0.10-meter resolution. We have static obstacle boxes representing furniture or pillars. The robot starts at position (-2, -2) and our goal is set at (+2, +2). When the dashboard loads, the A\* algorithm has already computed the optimal global path, displayed as the blue dashed route on the dashboard."* |
| **1:30 - 2:30** | Start Autonomous Navigation | *[Click '▶ Start Mission' on Streamlit]* <br> *"I am now initiating autonomous navigation. Notice how the robot physically turns toward its first waypoint in Webots and accelerates. On the dashboard, you can see live odometry: position coordinates, heading angle, linear speed at 0.16 m/s, and distance to destination decreasing smoothly. The LiDAR polar plot displays real-time 360-degree range rays bouncing off the static walls."* |
| **2:30 - 3:45** | Inject Dynamic Obstacle & Demonstrate Replanning | *[Wait until robot reaches mid-corridor, then click '➕ Add Obstacle']* <br> *"Now comes the core feature of our project: dynamic obstacle avoidance. Through the supervisor API, I have just introduced a 3D physical box directly into the robot's active corridor. Watch Webots: as the robot approaches, its virtual LiDAR detects the obstacle within our 0.50-meter safety zone. The robot immediately halts to prevent collision. The navigation state shifts to 'REPLANNING'. The obstacle is projected into the occupancy grid, and A\* calculates a new collision-free detour, now visible on the dashboard as the orange solid route. The robot automatically resumes movement around the new obstacle without any human intervention."* |
| **3:45 - 4:30** | Mission Completion | *[Robot arrives at destination (2.0, 2.0)]* <br> *"The robot reaches the destination and comes to an exact stop. The mission status updates to 'GOAL_REACHED'. The dashboard displays our complete mission analytics: total distance traveled, elapsed time, replanning count, and obstacle detections. The traveled trajectory is preserved as a green breadcrumb trail."* |
| **4:30 - 5:00** | Conclusion & Q&A | *"The entire system operates in software without any hardware dependency, yet uses authentic physics, sensor rays, and standard robotics algorithms. We have also verified all modules using automated unit tests with 100% passing results. I am now ready for your questions. Thank you."* |

---

## 3. Algorithm Explanations (Student-Friendly Language)

### A. How Does the A* Pathfinding Algorithm Work?
*"A\* is an informed search algorithm. It is smarter than Dijkstra's algorithm because, instead of exploring blindly in all directions, it uses a heuristic to guide the search toward the destination. At each grid cell $n$, A\* evaluates:*
$$f(n) = g(n) + h(n)$$
*Here, $g(n)$ is the exact cost incurred to travel from the start cell to the current cell, and $h(n)$ is the Euclidean distance heuristic estimating the remaining distance to the goal. In our implementation, moving orthogonally costs $1.0$ unit, while moving diagonally costs $\sqrt{2} \approx 1.414$ units. A priority queue continuously selects the cell with the smallest $f(n)$. Once the goal cell is reached, the path is reconstructed by tracing back through parent pointers."*

### B. What is the Purpose of Obstacle Inflation?
*"In standard path planning, algorithms treat the robot as an infinitesimally small single point. If a robot is just a point, it will plan a route that grazes right along the sharp edges of walls. In reality, our differential robot has a physical diameter of 32 cm. If the path passes too close to a wall, the robot's wheels or chassis will collide with it. Obstacle inflation adds a protective safety buffer of 25 cm around all obstacles. The algorithm considers this expanded zone non-traversable, forcing the path into the safe, open middle of corridors."*

### C. How Does the LiDAR Detect Obstacles in Webots?
*"Webots provides a `Lidar` node that casts 360 simulated laser rays across a full $360^\circ$ circle ($2\pi$ radians). The sensor returns a range array of 360 float values representing the distance to the nearest physical collision point. Our `ObstacleDetector` filters this data by isolating the front cone ($\pm 30^\circ$ relative to the robot's heading). If any reading in this front cone drops below the slowdown threshold (0.50m) and the detected coordinates overlap with our upcoming waypoints, the robot registers that its path is blocked and initiates replanning."*

### D. How Does the Differential-Drive Kinematic Controller Work?
*"Our robot has two independent drive wheels and a caster ball. To steer toward a waypoint $(w_x, w_y)$, the controller calculates the desired heading angle $\theta_{des} = \text{atan2}(w_y - y, w_x - x)$. The difference between this desired angle and the robot's current yaw angle is the heading error $e_\theta$. Using proportional control, we generate an angular velocity $\omega = K_p \cdot e_\theta$. Then, using differential drive kinematics, we convert linear velocity $v$ and angular velocity $\omega$ into individual left and right wheel speeds:*
$$\omega_L = \frac{v - \omega \cdot \frac{L}{2}}{r}, \quad \omega_R = \frac{v + \omega \cdot \frac{L}{2}}{r}$$
*When the error is large, the robot slows down its forward motion and rotates in place. When aligned, it accelerates forward smoothly."*

---

## 4. 30 Comprehensive Viva Questions and Answers

#### Q1: What is the primary objective of this project?
**Ans:** To develop a complete, software-based autonomous mobile robot simulation in Cyberbotics Webots that can navigate an indoor environment, detect dynamic obstacles using virtual LiDAR, and autonomously replan its route to a destination without human intervention.

#### Q2: Why did you choose Webots instead of Gazebo or simple Python 2D animations?
**Ans:** Webots provides accurate ODE-based rigid-body physics, realistic sensor simulation (including 360° LiDAR ray-casting), cross-platform Windows native support, fast rendering, and clean Python Supervisor API bindings without requiring heavy ROS setup or a Linux virtual machine.

#### Q3: What kinematic model does your robot use?
**Ans:** A differential-drive mobile robot model with two independently actuated driving wheels sharing a common axis and two passive caster wheels for vertical stability.

#### Q4: How is the environment represented for navigation?
**Ans:** As a 2D occupancy grid of dimensions $60 \times 60$ cells, where each cell represents $0.10\text{ m} \times 0.10\text{ m}$ of physical space. Cells are categorized into free, inflated safety zone, static obstacle, and dynamic obstacle.

#### Q5: What is the time complexity of the A* algorithm?
**Ans:** In the worst case, $O(b^d)$, where $b$ is the branching factor (8 for our 8-connected grid) and $d$ is the path depth. With an admissible and consistent heuristic like Euclidean distance, A* expands far fewer nodes than Dijkstra ($O(V \log V + E)$).

#### Q6: Is the Euclidean heuristic admissible? What does admissible mean?
**Ans:** Yes, the Euclidean distance is admissible because it represents the straight-line distance, which never overestimates the actual cost to reach the goal. An admissible heuristic guarantees that A* will find the optimal shortest path.

#### Q7: What is diagonal corner-cutting and how did you prevent it?
**Ans:** In an 8-connected grid, moving diagonally between two cells that share an obstacle corner could cause the robot's path to cut directly through the corner. We prevent this by checking both adjacent orthogonal cells: if either orthogonal neighbor is occupied, diagonal movement is strictly prohibited.

#### Q8: What is obstacle inflation and why is it necessary?
**Ans:** Obstacle inflation expands the physical boundary of obstacles by the robot's radius plus a safety clearance margin ($0.25\text{ m}$). It enables treating the robot as a point during path planning while guaranteeing the physical robot chassis will not graze walls.

#### Q9: How does the virtual LiDAR sensor work in Webots?
**Ans:** The virtual LiDAR casts 360 rays in a planar fan over $360^\circ$ at a 10 Hz frequency with a range of 0.06m to 3.5m. It returns a range image array of 360 float values corresponding to distances where rays intersect physical geometry.

#### Q10: How does the robot detect dynamic obstacles?
**Ans:** The `ObstacleDetector` examines beams in the front sector ($\pm 30^\circ$). If distances drop below 0.50m and the calculated Cartesian coordinates $(x_{obs}, y_{obs})$ intersect the corridor of upcoming waypoints, the route is declared blocked.

#### Q11: What happens when a dynamic obstacle is detected?
**Ans:** 
1. Motor speeds are set to zero (safety deceleration).
2. The navigation state transitions to `REPLANNING`.
3. The obstacle's position is marked in the occupancy grid.
4. A* is executed from the robot's current pose to the destination.
5. The new route is assigned to the waypoint follower, and the robot resumes movement.

#### Q12: How is the dynamic obstacle introduced into the Webots simulation?
**Ans:** Using Webots' Supervisor API (`supervisor TRUE`). When the user clicks "Add Obstacle" in Streamlit, the controller accesses the `DYNAMIC_OBSTACLE` node and modifies its `translation` field from below the floor `(0, 0, -5.0)` to floor level `(x, y, 0.225)`. The obstacle physically appears in the arena and is detected by the LiDAR rays.

#### Q13: What communication architecture connects Streamlit and Webots?
**Ans:** A dual-channel Inter-Process Communication (IPC) architecture:
1. High-speed TCP socket on `localhost:5050` with newline-delimited JSON messages.
2. Atomic JSON file exchange (`logs/telemetry.json` and `logs/command.json`) as a resilient fallback in case Windows Defender or loopback policies block sockets.

#### Q14: Why is the dashboard decoupled from the robot control loop?
**Ans:** To prevent UI rendering latency or browser refreshes from starving the robot's physics control loop. The Webots controller runs deterministically at 32 ms time steps regardless of whether the dashboard is open, refreshed, or closed.

#### Q15: How does the robot track waypoints?
**Ans:** Using a proportional heading controller. It calculates heading error $e_\theta = \theta_{des} - \theta$, normalizes it to $[-\pi, \pi]$, and commands angular velocity $\omega = K_p \cdot e_\theta$. Wheel speeds are calculated using inverse differential kinematics.

#### Q16: What happens when the heading error is large?
**Ans:** When $|e_\theta| > 0.45\text{ rad}$ ($\approx 25^\circ$), the linear forward velocity is reduced to nearly zero, and the robot performs a controlled turn-in-place until aligned with the target waypoint.

#### Q17: How does the robot know it has reached a waypoint or the final goal?
**Ans:** For intermediate waypoints, it advances when the Euclidean distance is $\le 0.15\text{ m}$ (`WAYPOINT_TOLERANCE`). For the final goal, it halts and declares `GOAL_REACHED` when distance is $\le 0.12\text{ m}$ (`GOAL_TOLERANCE`).

#### Q18: What localization method is used in the simulation?
**Ans:** Ground-truth localization using simulated GPS (`GPS` device) for $(X, Y)$ coordinates and an Inertial Unit (`InertialUnit` device) for yaw heading $(\theta)$.

#### Q19: What coordinate system does your simulation use?
**Ans:** The standard ENU (East-North-Up) coordinate system: $X$ axis is East, $Y$ axis is North, and $Z$ axis is Up (elevation). In 2D top-down planning, $(X, Y)$ correspond directly to ground coordinates.

#### Q20: What are the physical dimensions and speed limits of your robot?
**Ans:** Robot radius is 0.14m (clearance envelope 0.18m), wheel radius is 0.04m, track width is 0.16m. Maximum wheel speed is $6.28\text{ rad/s}$ (approx. $0.25\text{ m/s}$ linear velocity); nominal cruise speed is $0.16\text{ m/s}$.

#### Q21: How many static obstacles are present in the world?
**Ans:** Four static obstacles of dimensions between 0.5m and 0.8m arranged asymmetrically to create natural navigation corridors and dead-end challenges.

#### Q22: What happens if the user selects a goal inside an obstacle?
**Ans:** The system validates the coordinates before planning. If the destination cell has an occupancy value $\ge 100$ (inside an obstacle) or is outside room bounds, the dashboard warns the user and rejects the selection.

#### Q23: What happens if an obstacle completely blocks all paths to the destination?
**Ans:** A* explores all reachable nodes in the open set, finds no path, and returns `None`. The navigation controller transitions to `BLOCKED`, halts motors, and alerts the operator via the dashboard.

#### Q24: What metrics are logged during a navigation mission?
**Ans:** Robot $(X, Y, \theta)$ pose, linear and angular velocity, left and right wheel speeds, total distance traveled, elapsed time, minimum LiDAR clearance, number of replanning events, and number of obstacle detections.

#### Q25: How did you verify the correctness of your software?
**Ans:** By creating an automated test suite using `pytest` containing 10 unit test cases that validate A* path generation, corner clearance, unreachable goal handling, LiDAR scan filtering, emergency stop triggers, kinematics, and state transitions.

#### Q26: What are the main limitations of this project?
**Ans:** 
1. Navigation is 2D planar (overhanging 3D obstacles are not detected).
2. The simulation assumes perfect wheel traction without severe tire slip.
3. GPS/IMU localization provides idealized odometry without sensor drift.

#### Q27: How can this project be transitioned to physical hardware in the future?
**Ans:** By substituting Webots APIs with ROS 2 nodes: replacing `Lidar.getRangeImage()` with a subscription to `/scan` (from an RPLiDAR), replacing motor commands with publications to `/cmd_vel`, and deploying on a Raspberry Pi connected to motor drivers.

#### Q28: How does your system prevent endless oscillation around the goal?
**Ans:** By checking the arrival condition using a dedicated `GOAL_TOLERANCE` threshold (0.12m). Once the robot enters this radius, motor velocities are immediately set to 0.0 and the state transitions to `GOAL_REACHED`, stopping the control loop.

#### Q29: What is the role of Streamlit in this project?
**Ans:** Streamlit acts as an interactive Mission Control Station. It visualizes the 2D occupancy grid, shows original vs. replanned routes, displays real-time 360° LiDAR radar scans using Plotly, and allows the operator to send commands (Start, Pause, Reset, Inject Obstacle) to Webots.

#### Q30: What is the most innovative aspect of your project?
**Ans:** The authentic closed-loop dynamic avoidance integration: rather than artificially forcing a path replan in Python, a real physical 3D obstacle is inserted into Webots via the Supervisor API, physically reflecting the simulated LiDAR beams and triggering real sensor-driven avoidance.
