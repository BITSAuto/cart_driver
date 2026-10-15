# Interfaces

## driver (`cart_driver`)

| Dir | Topic | Type | Notes |
| --- | --- | --- | --- |
| in | `/vehicle/speed` | Float32 | m/s × `speed_scale` |
| in | `/steering/angle` | Float32 | degrees, + = right |
| in | `/vehicle/imu` | Imu | yaw rate (`use_imu`) |
| in | `/cart/safety` | String | `drive` / `coast` / `emergency_brake` |
| in | `/traversability/grid` | OccupancyGrid | integrated into memory at the capture-time camera pose |
| out | `/cmd_ackermann` | AckermannDrive | speed km/h (≥ 0), steering rad (+ = right) |
| out | `/cart/odom` + TF `odom → base_link` | Odometry | rear-axle pose |
| out | `/cart/state` | String | e.g. `drive`, `coast (blocked: no clear path)`, `coast (emergency_brake)`; with a road-centre line also the distance from it |
| out | `/cart/local_map` | OccupancyGrid | memory map, `odom`, 2 Hz |
| out | `/cart/plan` | Path | chosen candidate, `odom` |
| out | `/cart/steer_target` | Float32 | degrees, + = right |
| out | `/cart/centre_line` | Path | fitted road centre line in `odom` (road-centre cost) |

| Parameter | Default | Meaning |
| --- | --- | --- |
| `vehicle` | `tesla_sim` | profile name |
| `cruise_kmh` | 5.0 | cruise speed |
| `rate` | 20 Hz | control tick |
| `safety_timeout` | 0.5 s | coast if `/cart/safety` is older |
| `speed_topic`, `speed_scale` | `/vehicle/speed`, 1.0 | measured speed source |
| `imu_topic`, `use_imu` | `/vehicle/imu`, true | yaw rate for odometry |
| `memory_size_m`, `memory_resolution`, `memory_s` | 40, 0.1, 15 | memory map |
| `map_publish_rate` | 2 Hz | `/cart/local_map` |
| `horizon_m`, `plan_rate` | 10 m, 10 Hz | planner |
| `heading_tau` | 8 s | route-less heading keeping |
| `road_centre`, `centre_max_age` | true, 1.0 s | road-centre cost ([road-centre-cost.md](road-centre-cost.md)) |
| `road_x_near`, `road_x_far`, `road_min_width`, `road_max_width`, `road_nominal_width` | 3, 9.5, 2, 14, 0 m | estimator settings |

## safety (`cart_safety`)

| Dir | Name | Type | Notes |
| --- | --- | --- | --- |
| in | `/traversability/grid`, `/cart/local_map`, `/cart/odom`, speed, `/steering/angle` | | |
| out | `/vehicle/emergency_brake` | Bool | latched |
| out | `/cart/safety` | String | `drive` / `coast` / `emergency_brake` |
| out | `/cart/safety/collision_distance` | Float32 | m along the predicted path |
| service | `~/reset` | Trigger | clears the latch, only when stopped |

| Parameter | Default |
| --- | --- |
| `vehicle` | `tesla_sim` |
| `coast_decel` | 0.4 m/s² (must match the vehicle) |
| `stale_after` | 0.5 s |
| `reaction_time` | 0.3 s |
| `margin` | 0.5 m |
| `footprint_margin` | 0.1 m |
| `horizon` | 10 m |
| `brake_confirm_ticks` | 2 |
| `memory_max_age` | 1.0 s |
| `speed_topic`, `speed_scale` | `/vehicle/speed`, 1.0 |
