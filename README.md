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
