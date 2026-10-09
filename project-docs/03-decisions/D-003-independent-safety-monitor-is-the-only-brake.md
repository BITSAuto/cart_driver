# D-003 · An independent safety monitor is the only thing that brakes

- **Date:** 2026-10-08
- **Status:** Accepted

## Decision
A separate node, `safety`, is the only publisher of `/vehicle/emergency_brake`.
It brakes only when an obstacle on the predicted path is closer than the cart
can stop by coasting: `coast distance + 0.3 s reaction + 0.5 m margin`.

## Details
- Separate process so a planner bug can't disable it.
- Predicts along the **measured** steering, not the planned one.
- Checks the live grid and the driver's memory map.
- Requires the collision on **2 consecutive ticks** (one bad frame can't
  brake) and ignores lethal cells already under the footprint — both added
  after a false brake at the start line (see bugs).
- **Latched** until the vehicle has stopped and `~/reset` is called, so a
  flickering obstacle can't release and re-apply the brake.

## Consequences
- Emergency brakes in scenarios are failures unless the scenario forces one
  (an obstacle appearing too close to avoid).
