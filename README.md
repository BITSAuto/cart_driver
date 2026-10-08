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
4. Loop route: follow a loop route (keep left) continuously.

Each step lands as its own pull request.

## Nodes

| Node | Does | Topics |
| --- | --- | --- |
| `driver` | odometry; holds 5 km/h with a PI loop on measured speed (output ≥ 0, never a braking command); obeys `/cart/safety`; memory map, planner, route following | in: `/vehicle/speed`, `/steering/angle`, `/vehicle/imu`, `/cart/safety`, `/traversability/grid`, `/vehicle/gps`; out: `/cmd_ackermann`, `/cart/odom` + TF `odom → base_link`, `/cart/state`, `/cart/local_map`, `/cart/plan`, `/cart/route`, `/cart/route/cross_track` |
| `safety` | the **only** node that can brake; independent of the driver | in: `/traversability/grid`, `/cart/local_map` + `/cart/odom`, speed, `/steering/angle`; out: `/vehicle/emergency_brake`, `/cart/safety`, `/cart/safety/collision_distance`; service `~/reset` |

**Local planner** (`planner.py`, inside `driver`, 10 Hz).
- **Candidates:** about 21 paths of the form "steer towards target T at the
  relay's real 12.9 °/s, then hold". Each starts from the *measured* wheel
  angle and is rolled out 10 m at cruise speed on the memory map.
- **Scoring:**
  - any footprint collision rules a path out;
  - clearance from lethal cells (soft 1 m band);
  - unknown cells;
  - the route (step 5), or without a route a slowly updated reference
    heading, so the car returns to its course after swerving;
  - steering change (relay wear) and straightness.
- **No clear path:** the driver **coasts**; braking stays with the safety
  monitor.
- **Detection distance:** avoidance needs early detection. With ±15° and the
  relay ramp, the Tesla (2.94 m wheelbase, 11 m radius) needs about 7 m of
  travel to move 1.7 m sideways. The cart's shorter wheelbase should turn
  tighter.
- **Outputs:** the chosen path is `/cart/plan` and the target is
  `/cart/steer_target`.

**Loop route** (`route.py`, inside `driver`; step 5).
- **The route:** `routes/tesla_city_loop.yaml` is a closed 480 m loop in the
  world frame, in the **kerb-side left lane** (8.06 m left of the centre
  line), driven anticlockwise. It describes the GPS point (the front bumper in
  tesla_sim), one point every 0.5 m.
- **Making routes:** `tools/make_route.py` builds a route from either:
  - the road geometry (`--rounded-rect`; tesla_city's loop is straight
    segments plus 40.5 m-radius curves), or
  - a recorded centre line (`--csv`, from `tools/record_route.py`, which
    drives the yellow line and logs GPS).

  The geometry is preferred. A recording matches it to 0.24 m on average but
  drifts up to 1.7 m at junctions, where there's no centre line to follow.
- **Odom → world:** the planner works in `odom`, the route in the world frame.
  Each GPS fix is paired with odometry of the same point at the same time. A
  2D rigid fit over the last 30 m gives the transform, and the window absorbs
  odometry drift. Until the vehicle has moved 3 m, the known start heading
  (`initial_world_yaw`) is used instead.
- **Progress:** the nearest route point, searched only from 5 m behind to 30 m
  ahead of the last one. Progress can't jump to a nearby stretch of the loop,
  and it wraps around to count laps.
- **Planner cost:** each candidate's end point (10 m ahead) is scored by its
  distance from the route and its heading error against the route. Obstacles
  still win: collisions rule a candidate out, and clearance is still costed.
  So the car leaves the lane to pass an obstacle and then returns to it.
- **No stop:** the route is closed, so the cart just keeps lapping.

**Memory map** (`local_map.py`, inside `driver`).
- **Why:** perception sees only 2.7–10 m ahead in a narrow cone, so the
  inside of a turn, where the body will sweep, is out of view.
- **How:** each perception frame is placed in a 40 m rolling grid in the
  `odom` frame, at the camera's pose at *capture* time. Webots stamps sensors
  with wall-clock time while the drivers run on sim time;
  `timesync.ClockMap` converts between them.
- **Keeping it current:** the newest observation of a cell wins, so an area
  seen free again clears an old obstacle. Cells unseen for 15 s fall back to
  unknown.
- **Use:** published at 2 Hz as `/cart/local_map`. The safety monitor checks
  it too, so obstacles that have slid under the bonnet still count.

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

The sim config follows `routes/tesla_city_loop.yaml`. Set `route_file: ''`
to drive without a route (heading keeping only).

## Results

Step 2, in tesla_sim (sim time), driving straight from the start pose:

| Scenario | Result |
| --- | --- |
| A. Cruise, box 20 m ahead in the lane (no avoidance yet) | speed held at 5.13 km/h on average (5.06–5.25); emergency brake fired when the box was 3.2 m away at 5.1 km/h (coasting needed 2.47 m); stopped 3.4 m short of it |
| B. Perception stopped mid-drive | `coast` once the grid was 0.52 s old; rolled to a stop in about 3.5 s; **0** emergency-brake engagements |

Step 3, memory map:

| Scenario | Result |
| --- | --- |
| C. Drive 20.9 m past a box 3.5 m to the side | odometry drift vs GPS 0.02 m; with the box 8.7 m **behind** the bumper (out of view) it is still in the map, **0.01 m** from its true position; no other lethal cells; 0 brakes |

Step 4, planner:

| Scenario | Result |
| --- | --- |
| D. Box in the lane 20 m ahead | steered around it with **0.49 m** between the body and the box; peak heading change about 12–14°, then back towards the original course; 5.05 km/h throughout; **no coasting, 0 brakes** |
| E. Paper, a paint line and a dark stain in the lane | drove straight over all three, **0.0°** of steering for the whole run; 0 brakes |
| F. Box appears 4.5 m ahead while cruising (too close to avoid) | emergency brake at 2.50 m (coasting needed 2.48 m); stopped **2.54 m short** |

Step 5, loop route:

| Scenario | Result |
| --- | --- |
| L. Follow `tesla_city_loop` from the start pose for 420 s (sim time) | **584 m, more than one full lap, without stopping**; 5.00 km/h on average (5.00–5.24); **0 brakes, 0 coasting**. The car starts 10 m right of the lane and merged into it within 21 m. After that it stayed **0.31 m** from the route on average (p95 0.60 m, max 0.93 m) by GPS truth. The planner's own cross-track estimate agreed to within 0.05 m on average. The pattern repeats every lap: about 0.2 m inside on corners and up to 0.6 m outside where a straight follows a corner. |
