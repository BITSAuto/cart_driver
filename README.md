# cart_driver

Driving stack for the BITSAuto cart: hold a fixed low speed, steer a
continuous loop on the drivable area from `traversability`, and stay safe
**without using the brake in normal driving**.

## Constraints that shape the design

- **Never brake in normal driving.** The cart's brake can't modulate: it stops
  the vehicle instantly and can damage it. The stack holds **5 km/h** with
  throttle alone, slows only by **coasting** (throttle off, friction), and
  uses the emergency brake (`/vehicle/emergency_brake`) only as a last resort.
  That resort is reached when coasting can no longer avoid a collision.
- **Relay steering.** The actuator turns at about 12.9 °/s up to ±15°, so
  plans must respect how fast the wheels can turn, not just where they point.
- **Steering sign.** In this stack a **positive** steering angle turns
  **right**, on both `/cmd_ackermann` and `/steering/angle`. That's the
  opposite of REP 103. `vehicle.py` converts at the boundary; the planner
  works internally in curvature with positive = left.
- **Perception.** It sees about 2.7–10 m ahead with a narrow field of view
  (the camera is mounted 1.52 m high and pitched 9° down; the bonnet hides
  the near field). A short-term memory map covers what has left the view.

## Plan

1. Speed hold + odometry + an independent safety monitor.
2. Short-term memory map: rolling local occupancy in the `odom` frame.
3. Local planner: steering paths evaluated on the memory map, plus a
   scenario suite in the sim.
4. Loop route: follow a recorded loop (keep-left) continuously.

Each step lands as its own pull request.

## Nodes

| Node | Does | Topics |
| --- | --- | --- |
| `driver` | odometry; holds 5 km/h with a PI loop on measured speed (output ≥ 0, never a braking command); obeys `/cart/safety` | in: `/vehicle/speed`, `/steering/angle`, `/vehicle/imu`, `/cart/safety`; out: `/cmd_ackermann`, `/cart/odom` + TF `odom → base_link`, `/cart/state` |
| `safety` | the **only** node that can brake; independent of the driver | in: `/traversability/grid`, speed, `/steering/angle`; out: `/vehicle/emergency_brake`, `/cart/safety`, `/cart/safety/collision_distance`; service `~/reset` |

**Safety rules.** Each tick (20 Hz), `safety` predicts the swept footprint at
the current measured steering, out to 10 m on the latest perception grid. It
ignores lethal cells already under the vehicle, which can't be real. Then:

| Situation | Action |
| --- | --- |
| clear | `drive` |
| perception older than 0.5 s | `coast`: throttle off, roll to a stop, **no brake** |
| obstacle on the path closer than coast distance + 0.3 s reaction + 0.5 m, on 2 consecutive ticks | `emergency_brake`, **latched** until stopped and `~/reset` |

The driver coasts on anything but `drive`, and also if `safety` stops
publishing for 0.5 s.

## Running in the sim

```bash
~/ros2_ws/scripts/run_tesla_sim.sh                    # inside the distrobox
ros2 launch cart_driver sim.launch.py                 # perception + safety + driver (sim time)
python3 tools/simtools.py spawn box1 20 0 0.6 0.6 0.8 # obstacle 20 m ahead in the lane
python3 tools/simtools.py monitor 90 run.csv --until-stopped
```

## Results

Step 2, in tesla_sim (sim time), driving straight from the start pose:

| Scenario | Result |
| --- | --- |
| A. Cruise, box 20 m ahead in the lane (no avoidance yet) | speed held at 5.13 km/h on average (5.06–5.25); emergency brake fired when the box was 3.2 m away at 5.1 km/h (coasting needed 2.47 m); stopped 3.4 m short of it |
| B. Perception stopped mid-drive | `coast` once the grid was 0.52 s old; rolled to a stop in about 3.5 s; **0** emergency-brake engagements |
