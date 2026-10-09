# Speed and safety (step 2)

## Speed hold (driver)
- PI controller on measured speed (`kp` 0.5, `ki` 0.3, anti-windup), output
  clamped to **≥ 0** and ≤ 7 km/h. 0 means coast; it can never command
  braking.
- Target is `cruise_kmh` (5) only if `/cart/safety` is `drive`, fresh
  (< 0.5 s), and the planner found a clear path; otherwise 0 (coast).

## Safety monitor (separate node, 20 Hz)
Each tick it predicts the swept footprint along an arc at the **measured**
steering angle, out to 10 m, on:
1. the latest perception grid (start pose at `−camera_x`), and
2. the driver's memory map (`/cart/local_map` + `/cart/odom`, if under 1 s
   old), so obstacles that slid under the bonnet still count.

Lethal cells already under the vehicle's footprint are ignored
(`clear_footprint`; they can't be real if the cart is sitting on them).

| Situation | Action |
| --- | --- |
| clear | `drive` |
| perception older than 0.5 s | `coast` — throttle off, roll to a stop, **no brake** |
| obstacle on the path within coast distance + 0.3 s reaction + 0.5 m, on **2 consecutive ticks** | `emergency_brake`, **latched** until stopped and `~/reset` |

At 5 km/h the threshold is about 2.4 + 0.4 + 0.5 ≈ 3.3 m. Every engagement
is logged.

Why it's separate and why stale perception only coasts:
[D-003](../03-decisions/D-003-independent-safety-monitor-is-the-only-brake.md),
[D-004](../03-decisions/D-004-stale-perception-coasts.md).
