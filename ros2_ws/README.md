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

## Running Inside Docker (macOS/ARM Friendly)

Docker keeps the workspace reproducible when ROS 2 cannot be installed natively (for example on Apple Silicon). Everything you need lives under `ros2_ws/`.

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

5. **Expose ROS for the web UI**
   - Option A (single shell): after sourcing `install/setup.bash`, run
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml
     ```
   - Option B (Docker service): from the host run
     ```bash
     docker compose up rosbridge
     ```
     This launches a dedicated container that exposes rosbridge on port `9090`.
   - The UI prototype in `user_interface/prototype/` can connect with `roslibjs` at `ws://localhost:9090` and publish/subscribe to topics/services.

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
