# Hardware and environment

| | Sim (tesla_sim) | Real cart |
| --- | --- | --- |
| Body | Tesla Model 3: wheelbase 2.94 m, width 1.85 m, overhangs 0.85 / 0.95 m | **unknown**; `cart` profile has placeholders (1.7, 1.2, 0.4, 0.4) |
| Camera | 1.67 m ahead of the rear axle, 1.52 m high, 9° down | 1.52 m high, 9° down, centred (from the bag); position along the body unknown |
| Steering | relay model, 12.86 °/s, ±15° (same class as the cart) | relay via vehicle_bridge; lock not measured |
| Speed | throttle lag 1 s, coasting 0.4 m/s² | throttle open-loop and uncalibrated; coast rate unknown |
| Brake | `/vehicle/emergency_brake`, instant | same topic via vehicle_bridge |
| Measured speed | `/vehicle/speed` (wheel encoders) | **none yet** |
| Yaw rate | `/vehicle/imu` | D435i IMU (gyro) |
| Position fix | `/vehicle/gps` (ground truth, front bumper) | **none yet** |

- **Laptop:** ROS 2 Humble in the `ubuntu22` distrobox; Webots runs at about
  0.55× real time with perception and the driving stack. All nodes use
  `use_sim_time`; sensor stamps are wall-clock and are mapped with
  `timesync.ClockMap`.
- **Orin (target):** Jazzy. cart_driver hasn't been run there yet.
- Build with `PYTHONNOUSERSITE=1 colcon build --symlink-install` (user-site
  setuptools breaks `ament_python`).
