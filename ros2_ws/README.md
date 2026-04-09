# Pool Cleaning Simulation Stack (ROS 2)

This ROS 2 workspace provides a simulation-first stack for the autonomous pool cleaning MVP. It includes synthetic sensing, mapping, coverage planning, local collision-aware control, mission execution, and teleoperation helpers.

## Workspace Layout

```
ros2_ws/
  ├── src/
  │   ├── mock_sonar_sweep          # Synthetic polar sonar publisher
  │   ├── polar_to_grid_mapper      # Ray-casting occupancy grid mapper
  │   ├── mock_state_estimator      # Odometry integrator with drift/noise
  │   ├── coverage_planner          # Boustrophedon planner over /map
  │   ├── collision_costmap         # Forward-sector costmap + velocity scaling
  │   ├── local_planner             # Pure pursuit follower using /global_path
  │   ├── mission_executor          # State machine for Preset/Gen Path/Manual/Return
  │   ├── teleop_node               # Keyboard teleoperation (optional)
  │   └── sim_bringup               # Launch files and RViz configuration
```

All nodes are implemented in Python for rapid iteration. Each package installs with `ament_python` and exposes a console entry point.

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
      ros2 launch sim_bringup hw_bringup.launch.py serial_port:=/dev/ttyUSB0"
   ```
   Wait for `Yahboom driver ready` in the output.

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
   - For RViz or GUI tools on macOS, install [XQuartz](https://www.xquartz.org/) and enable “Allow connections from network clients.” Restart your machine or run `xhost + 127.0.0.1` after starting XQuartz.

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

- `mock_sonar_sweep`: Emits `sensor_msgs/LaserScan` on `/sonar/polar_scan` with configurable noise, dropouts, and scenarios.
- `polar_to_grid_mapper`: Integrates scans into `/map` (`nav_msgs/OccupancyGrid`) using log-odds updates.
- `mock_state_estimator`: Subscribes to `/cmd_vel`, injects drift/noise, and publishes `/odom`, `/pose`, plus TF from `odom` → `base_link`.
- `coverage_planner`: Produces `/coverage_plan` (`nav_msgs/Path`) with adjustable lane spacing/overlap once the map has enough free cells.
- `local_planner`: Follows `/global_path` via pure pursuit while respecting `/collision_velocity_scale` from the collision module.
- `collision_costmap`: Converts the latest scan into `/local_costmap` (`OccupancyGrid`) and a velocity scaling factor.
- `mission_executor`: Service-driven state machine exposing:
  - `/mission/start_preset`
  - `/mission/gen_path`
  - `/mission/manual_on` / `/mission/manual_off`
  - `/mission/save_manual_path`
  - `/mission/end_and_return`
  It republishes paths to `/global_path` and advertises the active mode on `/mission_state` (`std_msgs/String`).
- `teleop_node`: Minimal keyboard teleoperation publishing `/cmd_vel`.

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

## Next Steps

- Hook a joystick/GUI teleop and replace keyboard control.
- Connect bag recording scripts to capture synthetic datasets.
- Add evaluation scripts (coverage %, min obstacle distance) and integrate with the mission executor.
- Transition packages to C++ or optimized Python as performance demands increase.

Feel free to extend parameters or launch arguments to match your hardware-in-the-loop requirements.
