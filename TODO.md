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
