# Components

```mermaid
flowchart LR
    GRID[/traversability/grid/] --> DRV
    GRID --> SAF
    SPD[/vehicle/speed/] --> DRV
    SPD --> SAF
    STR[/steering/angle/] --> DRV
    STR --> SAF
    IMU[/vehicle/imu/] --> DRV
    GPS[/vehicle/gps/<br/>step 5] --> DRV
    subgraph DRV[driver node]
        OD[Odometry] --> LM[LocalMap memory]
        LM --> PL[ArcPlanner]
        RT[RouteFollower] --> PL
        SC[SpeedController PI]
    end
    DRV -->|/cart/local_map, /cart/odom| SAF[safety node]
    SAF -->|/cart/safety| DRV
    DRV -->|/cmd_ackermann| VEH[tesla_sim or vehicle_bridge]
    SAF -->|/vehicle/emergency_brake| VEH
```

## Nodes

| Executable | Node name | Role |
| --- | --- | --- |
| `driver` | `cart_driver` | odometry, memory map, planner, route following, PI speed hold; obeys `/cart/safety`; never brakes |
| `safety` | `cart_safety` | the **only** node that can brake; independent of the driver |

## Modules (`cart_driver/`)

| Module | Contents |
| --- | --- |
| `vehicle.py` | `VehicleParams`, `PROFILES` (`tesla_sim`, `cart`), footprint circles, steering ↔ curvature (with the + = right sign flip), `step_pose`, `offset_point`, `wrap` |
| `control.py` | `SpeedController` (PI, output ≥ 0), `Action` enum, `coast_distance`, `safety_action` |
| `odometry.py` | `Odometry`: dead reckoning from speed + steering (+ IMU yaw rate), 5 s pose history, `pose_at(t)` |
| `timesync.py` | `ClockMap`: wall-clock stamp → sim time |
| `gridview.py` | `GridView` over an `OccupancyGrid`, clearance (distance transform), footprint collision helpers |
| `local_map.py` | `LocalMap`: rolling memory map |
| `planner.py` | `ArcPlanner`, `PlannerWeights`, `Plan` |
| `route.py` (PR #4) | `Route`, `WorldAligner`, `RouteFollower` |
| `driver_node.py`, `safety_node.py` | the two ROS nodes |

## Launch and config
- `launch/sim.launch.py`: traversability (semantics on, sim student by
  default), `safety` and `driver`, all with `use_sim_time`. Arguments:
  `model`, `perception` (default true), `cart_params` (default
  `config/tesla_sim.yaml`). Deliberately not called `params`
  ([D-018](../03-decisions/D-018-launch-argument-cart-params.md)).
- `config/tesla_sim.yaml`: both nodes' parameters for the sim (step 5 adds
  the route).

## Tools (`tools/`)
- `simtools.py`: `spawn` (box relative to the GPS, `--flat` for drivable
  clutter), `remove`, `monitor SECONDS OUT.csv [--until-stopped]` (logs speed,
  safety, brake, steering, GPS, collision distance, cross-track; prints a
  summary).
- `record_route.py` (PR #4): drives the yellow centre line and records GPS.
- `make_route.py` (PR #4): builds route YAML from road geometry or a recorded
  CSV.
- `routes/tesla_city_loop.yaml` (PR #4).
