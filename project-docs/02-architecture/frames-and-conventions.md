# Frames and conventions

- **`odom`**: starts at the rear axle's initial pose, X forward. Dead
  reckoning; drifts slowly.
- **`base_link`**: rear-axle centre, X forward, Y left.
- **Perception grids** arrive in `camera_ground` (under the camera). They're
  placed in `odom` at the camera pose (`rear pose + camera_x` along the axis)
  **at the frame's capture time**.
- **World** (step 5): the simulator's world frame (or a campus map frame
  later); the route and GPS live there. `WorldAligner` provides odom → world.
- **Steering sign:** + = right on every topic. Internally the planner and
  odometry use curvature with + = **left** (REP 103);
  `vehicle.steer_to_curvature` converts (`κ = −tan(δ)/L`). Keep the
  conversion at that boundary only.
- **Units:** `/cmd_ackermann.speed` km/h; everything internal m/s, m, rad.
- **Time:** all nodes run on sim time in the sim. Sensor stamps from Webots
  are wall-clock; `ClockMap` maps them using a 3 s window of (wall, sim)
  pairs.
