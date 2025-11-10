What to do right now

1) Lock the MVP scope and metrics (half day)
- Features: preset path, auto-gen path, manual control, one-button return, collision prevention.
- KPIs: mapping time ≤25 min (sim), coverage ≥90–95%, zero simulated collisions, successful “End & Return” ≥99% in tests.
- Risks: slow scan rates, localization drift (no DVL), glossy tiles. Define mitigations you’ll validate in sim.

1) Build a simulation-first stack (ROS 2) (3–5 days)
- Create a ROS 2 workspace with these nodes/topics:
  - mock_sonar_sweep: publishes synthetic polar scans of pool boundaries/obstacles.
    - Topics: /sonar/polar_scan (bearing[], range[], intensity[])
  - polar_to_grid_mapper: converts polar scans to an occupancy grid.
    - Publishes: /map (nav_msgs/OccupancyGrid)
  - mock_state_estimator: integrates idealized odometry with noise (simulates IMU/flow).
    - Publishes: /odom (nav_msgs/Odometry)
  - coverage_planner: boustrophedon coverage over /map, with overlap and lane width params.
    - Publishes: /global_path (nav_msgs/Path)
  - local_planner: pure pursuit or TEB-like follower with curvature/velocity limits; consumes a short-horizon costmap for obstacle avoidance.
    - Publishes: /cmd_vel (geometry_msgs/Twist)
  - collision_costmap: creates a forward-sector costmap from the latest polar scan; implements slow-down/stop.
    - Publishes: /local_costmap (grid or custom)
  - mission_executor: modes (Preset, Gen Path, Manual, End & Return) as a state machine.
    - Services/Actions: start_preset, gen_path, manual_on/off, end_and_return
  - teleop_node: keyboard/joystick to command /cmd_vel in Manual mode.
- Visualization:
  - RViz: show /map, robot pose, planned path, local costmap, and sonar sector overlay.
- Scenarios:
  - Rectangular pool, L-shaped pool, pool with island/steps (vary obstacles and reflectivity).

1) Generate mock datasets (1–2 days)
- Write a simple script to produce “recorded” sonar sweeps along a path; save as rosbag.
- Produce versions with noise, missing returns (to mimic turbidity), and spurious reflections (to mimic glossy tiles).
- Use these to benchmark mapping robustness and collision prevention behavior.

1) Deliver a working demo video (1 day)
- Record screen while:
  - Running Gen Path: synthetic mapping builds → coverage path appears → robot follows.
  - Trigger End & Return: current task cancels and robot returns to “dock.”
  - Switch to Manual, nudge around, save “Recorded Manual Path” as a preset, then play it.
- Include overlays: coverage percentage over time; minimum distance to obstacles vs. time.

1) Design and documentation (parallel, small blocks each day)
- System block diagram: compute, motor drivers, future sensors (DVL optional), comms, power.
- Software architecture diagram: nodes, topics, actions, parameters, state machine.
- Test plan: pass/fail criteria for mapping, coverage, collision, return-to-dock.
- UX wireframes: 4 big buttons (Preset, Gen Path, Manual, End & Return) + status panel.
- Risk register: with validation steps in sim for each risk.

1) Procurement and budget planning (0.5–1 day)
- Prepare a tiered BOM (student MVP vs. stretch):
  - MVP sensors: IMU (BNO085), pressure (Bar30), optical flow (PMW3901), Ping1D + waterproof pan servo, leak sensor, bumper switch.
  - Compute: Raspberry Pi 4/CM4 + STM32 for motor/safety.
- Create a purchase timeline and lead times; identify used-market or edu-discount options.
- Draft a funding ask with justification tied to KPIs and sim results.

1) Mechanical and integration prep (0.5–1 day)
- Rough CAD sketch or hand drawings of the hull, sensor placements, and pan head window.
- Cable glands, sealing approach, and maintenance access (filter basket, brush).

What to show in your report

- Executive summary
  - Project goal, why “setup-once, any pool” matters, and MVP scope.
- Achievements
  - Simulated mapping and auto-gen coverage path (screenshots).
  - Working state machine for modes; End & Return demo.
  - Manual teleop with collision prevention in sim.
- Evidence
  - GitHub link to ROS 2 repo; list of nodes and launch files.
  - Demo video/gifs; coverage vs. time chart; min obstacle distance plot.
- Plan and timeline
  - Hardware orders next; first wet test target date; milestones for pool trials.
- Budget
  - Tiered BOM with costs and alternatives.
- Risks and mitigations
  - How sim validated mitigations; what remains to be proven on hardware.
- Asks
  - Funding, access to a test pool, mentors for ROS 2 review, 3D printing support.

Practical shortcuts to move fast

- Use Python nodes first for speed; optimize later.
- Reuse open-source coverage planners or pure-pursuit controllers if available.
- Keep the sonar sector narrow (e.g., 120–150°) in sim to mimic realistic refresh rates and force your avoidance logic to be conservative.
- Add a “noise dial” to your mock sonar to stress-test collision prevention and End & Return.

If you want, I can provide:
- A minimal ROS 2 package skeleton (topics, message types, and launch files).
- A one-page report template you can fill with your results.
- Sample code to generate synthetic polar scans for rectangular and L-shaped pools.