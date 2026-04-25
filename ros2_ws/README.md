# AquaSweep — Autonomous Pool Cleaning Robot (ROS 2)

This repository contains the full software stack for an autonomous pool cleaning robot built on ROS 2 Humble. It includes synthetic sensing for simulation, RPLIDAR C1 support for real hardware, SLAM Toolbox mapping (hardware) and log-odds occupancy grid mapping (simulation), boustrophedon coverage planning, collision-aware local control, a service-driven mission state machine, a hardware driver for the Yahboom ESC/IMU board, and a mobile-first web UI for remote operation.

All scan-consuming nodes accept a configurable `scan_topic` parameter (default `/scan`), so the same pipeline works with both the simulated scan publisher and the real RPLIDAR C1.

## Workspace Layout

```
fypTemp/                                # Project root
├── TODO.md                             # Priority roadmap
│
├── ros2_ws/                            # ROS 2 Humble workspace
│   ├── Dockerfile                      # ros:humble-ros-base + build tools + drivers
│   ├── docker-compose.yml              # Two services: ros2 (interactive) + rosbridge (ws:9090)
│   ├── docker/
│   │   ├── cyclonedds.xml              # CycloneDDS configuration
│   │   └── ros_entrypoint.sh           # Container entrypoint wrapper
│   ├── log_manual_control.py           # Historical logging utility
│   │
│   └── src/                            # 10 ROS 2 Python packages (ament_python)
│       │                               # + sllidar_ros2 (C++, external dep, cloned into src/)
│       │  ── Sensing ──────────────────────────────────────────
│       ├── sim_scan_publisher/         # Synthetic laser scan → /scan (sim only, 5 Hz)
│       ├── odom_integrator/            # Dead-reckoning odometry with drift & noise (30 Hz)
│       │
│       │  ── Mapping ──────────────────────────────────────────
│       ├── polar_to_grid_mapper/       # Log-odds occupancy grid from /scan (2 Hz)
│       │                               #   sim: publishes /map
│       │                               #   hw:  publishes /map_static (SLAM Toolbox owns /map)
│       │
│       │  ── Planning ─────────────────────────────────────────
│       ├── coverage_planner/           # Boustrophedon coverage over /map
│       ├── collision_costmap/          # Forward-sector costmap + velocity scaling
│       ├── local_planner/              # Pure pursuit path follower (10 Hz)
│       │
│       │  ── Mission & Control ────────────────────────────────
│       ├── mission_executor/           # State machine: Idle / Preset / Gen Path / Manual / Return
│       ├── teleop_node/                # Keyboard teleoperation → /cmd_vel
│       │
│       │  ── Hardware ─────────────────────────────────────────
│       ├── yahboom_driver/             # Serial driver for YB-ERF01-V3.0 board (UART 115200)
│       │                               # Differential ESC control + ICM20948 IMU readout
│       │                               # RPLIDAR C1 via sllidar_ros2 (external dep)
│       │
│       │  ── Bringup ─────────────────────────────────────────
│       └── sim_bringup/                # Launch files (sim + hw) and RViz / SLAM config
│           ├── launch/
│           │   ├── sim_bringup.launch.py
│           │   └── hw_bringup.launch.py
│           ├── rviz/pool_sim.rviz
│           └── config/
│               └── slam_toolbox_online_async.yaml
│
└── user_interface/                     # Mobile-first web UI (vanilla JS + roslibjs)
    ├── prototype/
    │   ├── index.html                  # SPA: 7 screens (Dashboard, Preset, Gen Path, Manual, …)
    │   ├── script.js                   # ROS bridge, joystick, service calls, live map canvas
    │   └── styles.css                  # Dark/light themes, glassmorphism, responsive
    ├── copywriting.md                  # Production UI copy strings
    ├── demo_storyboard.md              # 30-second demo script
    ├── ux_flow.md                      # Mermaid state diagram for screen transitions
    └── README.md                       # UI deliverables guide
```

All ROS 2 nodes are implemented in Python. Each package installs with `ament_python` and exposes a console entry point.

## Quick Start

1. **Install ROS 2 Humble (recommended)** and source the ROS environment.
2. Install Python dependencies used by the teleop and tf utilities (usually included with a ROS 2 desktop installation):
   ```bash
   sudo apt install python3-tf-transformations ros-humble-tf2-ros ros-humble-rviz2
   ```
3. **Build the workspace**:
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```
4. **Launch the full simulation stack** (RViz, synthetic sensors, planners, mission logic):
   ```bash
   ros2 launch sim_bringup sim_bringup.launch.py scenario:=rect_pool
   ```
   - `scenario` supports `rect_pool`, `l_pool`, `island_pool` for varied obstacle layouts.
   - Pass `teleop:=false` to skip starting the keyboard teleop node (e.g., when using a joystick node).

## Running on Raspberry Pi (Hardware Mode)

The stack runs inside Docker on the Raspberry Pi. The Dockerfile uses `ros:humble-ros-base` which has native arm64 support.

### Prerequisites

- Docker installed on the Pi (`curl -fsSL https://get.docker.com | sudo sh`)
- Yahboom board connected via USB (`/dev/ttyUSB0`)
- RPLIDAR C1 connected via USB (`/dev/ttyUSB1`)
- Phone/PC on the same network as the Pi

### Step-by-step startup

1. **Build the Docker image** (one-time):
   ```bash
   cd ros2_ws
   sudo docker compose build
   ```

2. **Build the ROS workspace** (one-time, or after code changes):
   ```bash
   sudo docker compose run --rm ros2 bash -c \
     "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
   ```

3. **Start rosbridge + ros2 containers**:
   ```bash
   cd ros2_ws
   sudo docker compose up rosbridge -d
   ```
   This starts both the `ros2` and `rosbridge` containers. Rosbridge listens on port **9090**.

4. **Install pyserial and launch the hardware stack**:
   ```bash
   sudo docker compose exec ros2 bash -c \
     "apt-get update -qq && apt-get install -y -qq python3-serial && \
      source /opt/ros/humble/setup.bash && \
      source /workspace/ros2_ws/install/setup.bash && \
      ros2 launch sim_bringup hw_bringup.launch.py serial_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1"
   ```
   Wait for `Yahboom driver ready` and `sllidar_node` startup messages in the output.

   > **Dry-run mode** (no physical hardware): pass `dry_run:=true` to log serial commands without opening the port — useful for testing the full stack on a desktop.

5. **Serve the web UI**:
   ```bash
   cd ../user_interface/prototype
   python3 -m http.server 8080
   ```

6. **Open the web app** on your phone/PC browser:
   ```
   http://<PI_IP>:8080?rosbridge=ws://<PI_IP>:9090
   ```
   Find the Pi's IP with `hostname -I`. The `rosbridge` URL is saved in localStorage after the first visit.

### Stopping everything

```bash
cd ros2_ws
sudo docker compose down
sudo kill $(lsof -t -i:8080) 2>/dev/null
```

### Field deployment (no external WiFi)

For environments without a stable network (e.g. a public pool), the Pi can act as its own WiFi hotspot. Your phone connects directly to it — no router needed.

```
Phone → Pi hotspot (AquaSweep) → web app (8080) + rosbridge (9090) → ROS 2 → motors
```

**Setup using NetworkManager:**

```bash
# Create a hotspot on wlan0 (ethernet stays available for SSH)
sudo nmcli device wifi hotspot ifname wlan0 ssid AquaSweep password aquasweep123

# The Pi gets a static IP of 10.42.0.1 by default
# On your phone, connect to the "AquaSweep" WiFi, then open:
#   http://10.42.0.1:8080?rosbridge=ws://10.42.0.1:9090
```

To make the hotspot start automatically on boot:

```bash
# Find the connection name
nmcli con show | grep Hotspot

# Set it to auto-connect
sudo nmcli con modify Hotspot autoconnect yes
```

## Running Inside Docker (macOS / Desktop)

Docker keeps the workspace reproducible when ROS 2 cannot be installed natively (e.g. on Apple Silicon).

1. **Prerequisites**
   - Install [Docker Desktop](https://www.docker.com/products/docker-desktop/).
   - For RViz or GUI tools on macOS, install [XQuartz](https://www.xquartz.org/) and enable "Allow connections from network clients." Restart your machine or run `xhost + 127.0.0.1` after starting XQuartz.

2. **Build the image** (from the repository root):
   ```bash
   cd ros2_ws
   docker compose build
   ```

3. **Start an interactive container** (mounts the workspace for live edits):
   ```bash
   docker compose run --rm ros2
   ```

4. **Inside the container**
   ```bash
   # resolve dependencies the first time
   rosdep install --from-paths src --ignore-src -y

   colcon build --symlink-install
   source install/setup.bash
   ros2 launch sim_bringup sim_bringup.launch.py scenario:=rect_pool
   ```

5. **Expose ROS for the web UI**:
   ```bash
   docker compose up rosbridge -d
   ```
   Rosbridge will be available at `ws://localhost:9090`. The UI prototype in `user_interface/prototype/` connects via roslibjs.

6. **Tweaking DDS discovery**
   - Override `ROS_DOMAIN_ID` in `docker-compose.yml` to isolate networks or troubleshoot discovery.

7. **Rebuild after changes**
   - Source code lives on your host. Re-run `colcon build` inside the container whenever you modify packages. Artifacts persist via the mounted `build/`, `install/`, and `log/` directories.

## Nodes Overview

| Node | Publishes | Subscribes | Summary |
|------|-----------|------------|---------|
| `sim_scan_publisher` | `/scan` (LaserScan) | `/odom` | Synthetic laser scan with noise, dropouts, and multiple pool scenarios (sim only, 5 Hz). |
| `odom_integrator` | `/odom`, `/pose`, TF `odom→base_link` | `/cmd_vel` | Dead-reckoning odometry with configurable drift and noise (30 Hz). Noise/drift disabled in hw mode. |
| `polar_to_grid_mapper` | `/map` (sim) or `/map_static` (hw) (OccupancyGrid) | `/scan`, `/odom` | Log-odds Bayesian grid mapping via Bresenham ray-casting (200×200 @ 0.15 m in sim; 200×200 @ 0.05 m in hw). |
| `slam_toolbox` | `/map` (OccupancyGrid), TF `map→odom` | `/scan`, `/odom` | SLAM Toolbox online async mapper (hw only). Publishes authoritative `/map` and corrects odometry drift via loop closure. |
| `coverage_planner` | `/coverage_plan` (Path) | `/map` | Boustrophedon lane planner with adjustable spacing and overlap. |
| `collision_costmap` | `/local_costmap` (OccupancyGrid), `/collision_velocity_scale` (Float32) | `/scan` | 120° forward-sector costmap; scales velocity linearly 0.4–1.2 m. |
| `local_planner` | `/cmd_vel` | `/global_path`, `/odom`, `/collision_velocity_scale` | Pure pursuit follower (lookahead 0.8 m, 10 Hz). |
| `mission_executor` | `/global_path`, `/mission_state`, `/manual_trace` | `/coverage_plan`, `/odom` | State machine (IDLE → PRESET / AUTO_GEN / MANUAL / RETURN) with six `Trigger` services under `/mission/*`. |
| `yahboom_driver` | `/esc_values` (Float32MultiArray), `/imu/data_raw` (Imu), `/imu/mag` (MagneticField) | `/cmd_vel` | Serial driver for YB-ERF01-V3.0 board; differential kinematics, auto-arming, safety timeout, ICM20948 IMU readout. |
| `teleop_node` | `/cmd_vel` | — | WASD keyboard teleoperation (0.3 m/s linear, 0.7 rad/s angular). |
| `sllidar_node` | `/scan` (LaserScan) | — | RPLIDAR C1 driver (hardware only, via `sllidar_ros2`). |
| `sim_bringup` | — | — | Launch files: `sim_bringup.launch.py` (simulation) and `hw_bringup.launch.py` (hardware + RPLIDAR C1 + SLAM Toolbox). |

### Data Flow

**Simulation mode** (`sim_bringup.launch.py`):

```
┌─ Sensing ──────────────────────────────────────────────────────┐
│  sim_scan_publisher ──→ /scan                                   │
│  odom_integrator    ──→ /odom, TF (odom→base_link)             │
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Mapping ──────────────────────────────────────────────────────┐
│  polar_to_grid_mapper ──→ /map                                 │
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Planning ─────────────────────────────────────────────────────┐
│  coverage_planner ──→ /coverage_plan                           │
│  collision_costmap ──→ /local_costmap, /collision_velocity_scale│
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Mission & Execution ─────────────────────────────────────────┐
│  mission_executor ──→ /global_path, /mission_state             │
│  local_planner    ──→ /cmd_vel                                 │
│  teleop_node      ──→ /cmd_vel (manual override)               │
└────────────────────────────────────────────────────────────────┘
```

**Hardware mode** (`hw_bringup.launch.py`):

```
┌─ Sensing ──────────────────────────────────────────────────────┐
│  sllidar_node    ──→ /scan  (RPLIDAR C1, /dev/ttyUSB1)         │
│  odom_integrator ──→ /odom, TF (odom→base_link)  [no noise]   │
│  yahboom_driver  ──→ /imu/data_raw, /imu/mag  (ICM20948)       │
└────────────────────────────────────────────────────────────────┘
        │                        │
        ▼                        ▼
┌─ Mapping ──────────────────────────────────────────────────────┐
│  slam_toolbox         ──→ /map (primary), TF (map→odom)        │
│  polar_to_grid_mapper ──→ /map_static (fallback/UI toggle)     │
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Planning ─────────────────────────────────────────────────────┐
│  coverage_planner ──→ /coverage_plan                           │
│  collision_costmap ──→ /local_costmap, /collision_velocity_scale│
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Mission & Execution ─────────────────────────────────────────┐
│  mission_executor ──→ /global_path, /mission_state             │
│  local_planner    ──→ /cmd_vel                                 │
└────────────────────────────────────────────────────────────────┘
        │
        ▼
┌─ Hardware ─────────────────────────────────────────────────────┐
│  yahboom_driver ──→ UART /dev/ttyUSB0 ──→ ESC motors           │
└────────────────────────────────────────────────────────────────┘
```

Static TFs published by `hw_bringup`: `base_link → laser` (0, 0, 0.05 m) and `base_link → imu_link`.

## Manual Path Recording Workflow

1. Call `/mission/manual_on` (robot stops following autonomous path).
2. Drive manually using the teleop node. The executor records `/odom` poses.
3. Call `/mission/manual_off` when finished.
4. Store the trace with `/mission/save_manual_path`.
5. Later, trigger `/mission/start_preset` to replay the recorded path.

## Coverage + Return Demonstration Script

A typical demo sequence in RViz:
1. Launch stack with `scenario:=rect_pool`.
2. Once mapping stabilizes, call `/mission/gen_path` to send the planner output to `/global_path` and watch the robot track lanes.
3. During execution, issue `/mission/end_and_return` to cancel coverage and generate a direct path back to the dock origin.
4. Engage manual mode, nudge around obstacles, save the path, and replay as preset.

## Web UI

The UI (`user_interface/prototype/`) is a mobile-first single-page app connecting to ROS via rosbridge WebSocket (`ws://<host>:9090`).

**Implemented:**
- Dashboard with live `/mission_state` status indicator
- Manual joystick control sending `/cmd_vel`
- Live occupancy grid map canvas (subscribes to `/map` or `/map_static`; toggle between SLAM and static mapper)
- Robot pose overlay on map via TF (`map → odom → base_link`)
- Real-time `/esc_values` feedback display

**Pending:**
- Mission service calls from UI (`/mission/start_preset`, `/mission/gen_path`, `/mission/manual_on`, `/mission/manual_off`, `/mission/save_manual_path`, `/mission/end_and_return`)

## Next Steps

1. **Hardware validation** — Test ESC wiring and motor response on real hardware; validate propeller directions; set safety limits.
2. **RPLIDAR C1 field testing** — Verify `/scan` quality poolside; tune `collision_costmap` thresholds (`stop_distance`, `slow_distance`) for real data.
3. **Web UI mission service calls** — Wire the six `/mission/*` service calls in `script.js` (screens: Preset, Gen Path, Manual, Return).
4. **SLAM Toolbox evaluation** — Test SLAM Toolbox performance on real poolside data; pool walls are feature-poor so tune `min_laser_range` / `max_laser_range` and loop closure parameters as needed.
5. **Evaluation scripts** — Coverage %, min obstacle distance, and run-time metrics.
