# TODO

## 1. [PRIORITY 1] ESC <-> Yahboom Control Board — App Motor Control

The most critical task: get the ESC talking to the Yahboom control board so the app can drive the motors.

- [ ] Identify the communication protocol between the ESC and the Yahboom board (PWM, UART, CAN, etc.)
- [ ] Wire up the ESC to the correct pins/port on the Yahboom board and verify power/ground connections
- [ ] Flash or configure any firmware needed on the Yahboom board to output motor commands
- [ ] Write/test a minimal script or ROS 2 node that sends a command from the app through to the ESC
- [ ] Confirm motor responds to forward, reverse, and stop commands from the app
- [ ] Validate full-range throttle control and ensure safe hardware limits are set (prevent runaway)

---

## 2. [PRIORITY 2] RPLIDAR C1 Integration

Once motor control is working, bring in the RPLIDAR C1 for sensing.

- [ ] Connect the RPLIDAR C1 via USB/UART and verify device is detected by the host (e.g., /dev/ttyUSB0)
- [ ] Install and configure the rplidar_ros (or equivalent ROS 2) driver package
- [ ] Launch the LIDAR node and confirm scan data is publishing on /scan
- [ ] Visualize scan data in RViz to validate correct range/angle readings
- [ ] Integrate /scan into the existing ROS 2 stack (replace or supplement mock sonar sweep)
- [ ] Update polar_to_grid_mapper or equivalent node to consume real LIDAR data
- [ ] Test obstacle detection and collision costmap with live RPLIDAR data

-------------------------------
Here's my analysis addressing all four of your questions:

---

## 1. Coverage Pattern for a 50m × 25m Standard Swimming Pool

Your existing `coverage_planner` in planner_node.py already implements a **boustrophedon (lawnmower) pattern** — this is the correct base pattern for a rectangular pool. However, it currently derives lanes from the occupancy grid bounds, which is generic. For a known 50m × 25m pool you should consider:

**Recommended pattern parameters:**
- **Lane spacing**: Depends on your cleaning/scanning width. With `lane_spacing = 0.75m` and `overlap = 0.15m` (your current defaults), you get an effective 0.6m lane width → ~42 lanes across 25m, covering the pool in long 50m sweeps.
- **Orientation**: Sweep along the **long axis (50m)** to minimize the number of turns (42 turns vs. 83 turns if you swept the short way). Currently your planner sweeps rows along X with lane steps in Y — so the pool's long axis should align with X.
- **Turn handling**: The current planner generates sharp U-turns (just two poses per lane). For a real robot with turning radius constraints, you may want to add **Dubins-style semicircular turn waypoints** at each end.

**Preset pattern idea**: For the "Preset" mode in your web UI, you could hardcode a 50×25m boustrophedon path as a known pattern (bypassing the occupancy grid entirely) since the pool geometry is fixed:

```
Lane count = ceil(25.0 / effective_lane_width)
Each lane = two waypoints at (x_start, y_lane) and (x_end, y_lane)
Alternate direction each lane
```

This avoids depending on SLAM having fully explored the pool before cleaning starts.

---

## 2. Hector SLAM for Pool Mapping & Collision Detection

**Short answer: Hector SLAM is a reasonable choice but has significant caveats for your scenario.**

### What Hector SLAM does well
- **No odometry required** — it aligns scans purely via scan-matching. This is ideal since your `odom_integrator` is still dead-reckoning with drift/noise.
- **Works with RPLIDAR C1** — it consumes standard `/scan` (LaserScan) messages.
- **Publishes an `OccupancyGrid` on `/map`** — directly compatible with your existing `coverage_planner` and `collision_costmap`.

### Major issues for your project

| Concern | Detail |
|---------|--------|
| **ROS 1 only** | `hector_slam` on GitHub is ROS 1 (Noetic). There is no official ROS 2 port. You'd need to either: (a) use `ros1_bridge` in your Docker setup, or (b) find a community ROS 2 fork (several exist on GitHub but are unofficial). |
| **Featureless environment** | A swimming pool has **smooth, uniform walls** with very few distinct scan features. Hector SLAM's scan-matching will likely **degenerate** in long featureless corridors (the 50m sides). You may get drift or complete scan-matching failure. |
| **2D only** | Hector SLAM is 2D. Fine for a surface robot, but you get no depth information. |
| **Static map assumption** | Pool walls don't move, so this is fine. People/objects in the pool would appear as transient obstacles. |

### Better alternatives to consider

1. **Keep your current `polar_to_grid_mapper`** — it already does log-odds occupancy grid mapping from `/scan` + `/odom`. For a known rectangular pool, this is arguably sufficient. You just need good-enough odometry.
2. **SLAM Toolbox** (`slam_toolbox`) — the de-facto ROS 2 SLAM solution. It's well-maintained, works with Humble, supports online/offline mapping, and handles loop closures. Install: `apt install ros-humble-slam-toolbox`.
3. **For collision detection** — you already have `collision_costmap` doing forward-sector scanning from `/scan` with velocity scaling. This works independently of whichever SLAM you use. Hector SLAM doesn't add collision detection — it only provides mapping.

**Recommendation**: Use **SLAM Toolbox** instead of Hector SLAM. It's native ROS 2, actively maintained, and would slot directly into your existing pipeline as a drop-in replacement for `polar_to_grid_mapper` (publishing the same `/map` topic).

---

## 3. Visual Mapping on the Web App

### RViz
Your sim_bringup.launch.py and hw_bringup.launch.py already launch RViz with your pool_sim config. RViz can display `/map`, `/scan`, `/coverage_plan`, `/local_costmap`, and the robot's TF tree — great for **development/debugging** on a desktop, but not suitable for the phone-based web UI.

### Streaming the map to the web app
Your web app connects via **rosbridge (ws://…:9090)**. You can absolutely display the map in the browser. Here's how:

**Option A — Subscribe to `/map` OccupancyGrid directly in JS:**
```js
const mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/map',
    messageType: 'nav_msgs/msg/OccupancyGrid',
});
mapTopic.subscribe(msg => {
    // msg.info.width, msg.info.height, msg.info.resolution
    // msg.data = flat array of occupancy values (-1=unknown, 0-100=probability)
    // Render to a <canvas> element
    drawOccupancyGrid(msg);
});
```
Then render the grid data onto an HTML `<canvas>`, mapping cell values to colours (grey=unknown, white=free, black=occupied). Overlay the robot position from `/odom` and the planned path from `/coverage_plan`.

**Option B — Use `ros2djs` or `nav2djs`** (RobotWebTools libraries):
- [`ros2d`](https://github.com/RobotWebTools/ros2djs) + [`nav2d`](https://github.com/RobotWebTools/nav2djs) can render OccupancyGrid maps, paths, and robot poses directly in the browser.
- These are purpose-built for exactly this use case.

**Option C — `rosboard`**:
- A standalone web dashboard that can visualize topics including OccupancyGrid, LaserScan, Images. Runs alongside your stack. Less customizable than building it into your existing UI.

**Recommendation**: Option A is simplest and fits your existing script.js architecture — you already have the rosbridge connection, topic subscriptions, and canvas-like UI elements. Add a `<canvas>` to your map screen and subscribe to `/map`.

---

## 4. Propeller Direction/RPM Mismatch on Forward Joystick Input

This is the most immediately actionable issue. Looking at the code path:

**Joystick** (script.js) → `publishCmdVel(linearX=maxSpeed, angularZ=0)` → rosbridge → `/cmd_vel`

**Yahboom driver** (yahboom_driver_node.py):
```python
left_vel  = linear_x - angular_z * (wheel_base / 2.0)   # = linear_x when angular=0
right_vel = linear_x + angular_z * (wheel_base / 2.0)   # = linear_x when angular=0

left_angle  = 90 + ratio * 90   # e.g. 90 + 0.3*90 = 117
right_angle = 90 + ratio * 90   # e.g. 90 + 0.3*90 = 117
```

When you push the joystick straight forward, both servos get the **same angle** (e.g., 117). If the propellers spin in different directions or at different speeds, the problem is one of:

### Likely causes

1. **Physical ESC wiring is reversed on one motor** — If one ESC has its motor leads swapped relative to the other, angle `117` means "forward" on one and "backward" on the other. **This is the most common cause.**

2. **ESCs are calibrated differently** — Bidirectional ESCs need to be calibrated so they agree on what angle = neutral. If one ESC thinks neutral is 88 and the other thinks it's 92, they'll respond differently to the same angle.

3. **Different ESC models or firmware** — Even same-model ESCs can have slightly different response curves.

### Fixes

**Fix for reversed direction** — Add a per-motor direction multiplier parameter in the driver:

In yahboom_driver_node.py, add parameters:
```python
self.declare_parameter('left_motor_direction', 1)   # 1 or -1
self.declare_parameter('right_motor_direction', 1)   # 1 or -1
```

Then in `_send_command`, apply direction before computing the angle:
```python
left_dir = int(self.get_parameter('left_motor_direction').value)
right_dir = int(self.get_parameter('right_motor_direction').value)
left_angle  = int(_ESC_NEUTRAL + left_ratio * left_dir * _ESC_NEUTRAL)
right_angle = int(_ESC_NEUTRAL + right_ratio * right_dir * _ESC_NEUTRAL)
```

If one propeller spins backward when both should go forward, set that motor's direction to `-1`.

**Fix for RPM mismatch** — Add per-motor trim/scale parameters:
```python
self.declare_parameter('left_motor_scale', 1.0)
self.declare_parameter('right_motor_scale', 1.0)
```

Then scale each ratio independently before converting to angle.

**Diagnostic step** — Use your existing test_esc.py to send the **same** angle to both servos and observe: do both motors spin the same direction at the same speed? If not, it's a hardware calibration issue. If they do match in test_esc.py but not through the ROS stack, the bug is in the differential-drive math.

---

## Summary Action Plan

| Priority | Task | Effort |
|----------|------|--------|
| **P0** | Fix propeller direction — add `left_motor_direction` / `right_motor_direction` params to `yahboom_driver` | Small code change |
| **P1** | Add per-motor scale/trim params for RPM matching | Small code change |
| **P2** | Add a hardcoded 50×25m boustrophedon preset to `coverage_planner` (skip SLAM dependency for known pool) | Medium |
| **P3** | Integrate **SLAM Toolbox** instead of Hector SLAM (native ROS 2, better maintained) | Medium — `apt install` + launch file config |
| **P4** | Add `/map` canvas rendering to web UI via rosbridge subscription | Medium — JS canvas work |
| **P5** | Overlay robot pose + coverage path on the map canvas | Medium |