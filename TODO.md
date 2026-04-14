# AquaSweep — Project TODO

**Last updated**: April 13, 2026

---

## ✅ Recently Completed

- [x] **RPLIDAR C1 Integration** — `sllidar_ros2` driver integrated in `hw_bringup.launch.py`, publishing to `/scan`. All scan-consuming nodes now use configurable `scan_topic` parameter.
- [x] **Package Renames** — Removed all "mock" and "sonar" references for clarity:
  - `mock_sonar_sweep` → `sim_scan_publisher` (simulation-only synthetic laser scan)
  - `mock_state_estimator` → `odom_integrator` (dead-reckoning odometry, used in sim + hw)
  - Updated all launch files, README, and package dependencies
- [x] **Unified Scan Topic** — Both simulation and hardware use `/scan`, making the pipeline hardware-agnostic

---

## 🔴 Priority 1: Hardware Validation & Field Testing

### ESC ↔ Yahboom Board Integration
**Status**: Driver code exists (`yahboom_driver`), but untested on real hardware.

- [ ] Verify physical wiring (ESC → Yahboom board pins, power/ground)
- [ ] Test `test_esc.py` on hardware to validate servo protocol (0xFF 0xFC frame format)
- [ ] Launch `hw_bringup.launch.py` and send `/cmd_vel` via teleop or web UI
- [ ] Validate motor response: forward, reverse, stop, full throttle range
- [ ] **Fix propeller direction mismatch** if one motor spins backward:
  - Add `left_motor_direction` / `right_motor_direction` parameters (1 or -1)
  - Add per-motor `left_motor_scale` / `right_motor_scale` for RPM trim
- [ ] Set hardware safety limits (max throttle, timeout behavior)
- [ ] Test emergency stop and timeout safety (no cmd_vel → motors stop)

### RPLIDAR C1 Field Testing
**Status**: Integrated in software, needs real-world validation.

- [ ] Connect RPLIDAR C1 to Pi via `/dev/ttyUSB1`, launch `hw_bringup.launch.py`
- [ ] Verify `/scan` data quality in RViz (range, angle, noise characteristics)
- [ ] Test in water/poolside to check for reflections, dropouts, or interference
- [ ] Tune `polar_to_grid_mapper` parameters for real LIDAR data:
  - Adjust `hit_log_odds`, `miss_log_odds` if mapping is too aggressive/conservative
  - Verify 0.15m resolution is appropriate for pool-scale obstacles
- [ ] Tune `collision_costmap` thresholds (`stop_distance`, `slow_distance`)
- [ ] Validate obstacle detection with real pool walls and test objects

---

## 🟡 Priority 2: Web UI Enhancements

### Mission Service Integration
**Status**: Services exist in `mission_executor`, but not called from web UI.

- [ ] Wire `/mission/start_preset` service call in `script.js` (Preset screen)
- [ ] Wire `/mission/gen_path` service call (Gen Path screen)
- [ ] Wire `/mission/manual_on` / `/mission/manual_off` (Manual screen recording toggle)
- [ ] Wire `/mission/save_manual_path` (save recorded path as preset)
- [ ] Wire `/mission/end_and_return` (End & Return button)
- [ ] Display service call feedback (success/failure toasts)

### Map Visualization
**Status**: `/map` topic exists, but not displayed in UI.

- [ ] Add `<canvas>` element to Map & History screen
- [ ] Subscribe to `/map` (nav_msgs/OccupancyGrid) via ROSLIB.Topic
- [ ] Render occupancy grid to canvas (grey=unknown, white=free, black=occupied)
- [ ] Overlay robot position from `/odom` or `/pose`
- [ ] Overlay coverage path from `/coverage_plan` or `/global_path`
- [ ] Add pan/zoom controls for the map canvas

### Status Monitoring
- [ ] Subscribe to `/mission_state` and update Dashboard mode indicator
- [ ] Display live `/esc_values` feedback (already subscribed, but enhance visualization)
- [ ] Add connectivity status indicator (rosbridge connected/disconnected)
- [ ] Battery level integration (requires hardware sensor — future)

---

## 🟢 Priority 3: Optimization & Evaluation

### Coverage Optimization
- [ ] Add hardcoded 50m × 25m preset pattern for known pool geometry (bypass SLAM)
- [ ] Implement Dubins-style semicircular turns at lane ends (smoother for real robot)
- [ ] Test coverage completeness with simulation scenarios (`rect_pool`, `l_pool`, `island_pool`)

### Evaluation & Metrics
- [ ] Implement coverage % calculator (free cells visited / total free cells)
- [ ] Log min obstacle distance during runs (safety metric)
- [ ] Track run duration and path length
- [ ] Export metrics to `/mission_state` or a dedicated diagnostics topic

### Simulation Testing
- [ ] Run full demo sequence in simulation (gen path → execute → manual → return)
- [ ] Test all 3 pool scenarios with the new renamed packages
- [ ] Validate collision avoidance with `collision_velocity_scale` feedback

---

## 🔵 Priority 4: Nice-to-Haves & Future Work

### SLAM Improvements
- [ ] **Consider SLAM Toolbox** instead of `polar_to_grid_mapper`:
  - Native ROS 2, actively maintained
  - Loop closure detection for drift correction
  - `apt install ros-humble-slam-toolbox`
  - Caveat: Pool environment is feature-poor (smooth walls) — may not improve much
- [ ] Test SLAM Toolbox vs. current dead-reckoning + grid mapping
- [ ] For production: Consider **visual odometry** (camera + lidar fusion) for better pose estimation

### Documentation
- [ ] Add architecture diagram (sensing → mapping → planning → control → hardware)
- [ ] Document all ROS 2 parameters for each node
- [ ] Write deployment guide for Pi hotspot setup (currently in README, needs expansion)
- [ ] Video demo of full system (sim + hardware)

### Code Quality
- [ ] Add unit tests for differential drive kinematics (`yahboom_driver`)
- [ ] Add integration tests for mission state machine transitions
- [ ] Add linting/formatting (black, ruff) to CI/CD
- [ ] Type hints cleanup (some nodes use `| None`, others use `Optional`)

---

## 📝 Notes & Technical Decisions

### Why not Hector SLAM?
- ROS 1 only (no official ROS 2 port)
- Pool environment is too featureless (smooth walls) for scan-matching SLAM
- Current `polar_to_grid_mapper` + dead-reckoning is sufficient for rectangular pools

### Why rename packages?
- "mock" and "sonar" were confusing — packages are used in both sim and hardware modes
- `sim_scan_publisher` clearly indicates sim-only synthetic data
- `odom_integrator` is hardware-agnostic dead-reckoning, used everywhere

### Key Architecture Choices
- **Unified `/scan` topic**: Both `sim_scan_publisher` (sim) and `sllidar_node` (hw) publish to `/scan`
- **Hardware-agnostic pipeline**: Mapping, planning, and control nodes work identically in sim and hw
- **Dead-reckoning odometry**: `odom_integrator` used until real IMU/encoders are integrated
- **Rosbridge for UI**: Web app connects via WebSocket (ws://…:9090), no custom backend needed