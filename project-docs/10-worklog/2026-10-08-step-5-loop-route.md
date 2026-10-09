# 2026-10-08 · Step 5: loop route

**Who:** project owner, with Claude Code
**PR:** [#4](https://github.com/BITSAuto/cart_driver/pull/4) (open)

## What was done
- Recorded the centre line (526 m) by following the yellow line; found it
  drifts at junctions; generated the route from the road geometry instead
  ([D-013](../03-decisions/D-013-route-from-road-geometry-kerb-side-left-lane.md)).
- `route.py`: GPS/odometry alignment ([D-014](../03-decisions/D-014-gps-odometry-windowed-alignment.md)),
  progress, route cost; driver wiring; 8 tests.

## Results
[step-5-loop.md](../09-testing-and-results/step-5-loop.md).

## Afterwards
The owner tested step 3 in RViz ("No map received", M-11), asked how to add
routes ([making-routes.md](../08-guides/making-routes.md)) and how to test on
the real cart ([real-cart-test-plan.md](../08-guides/real-cart-test-plan.md)).
