# D-021 · No recorded routes: drive from what perception sees

- **Date:** 2026-10-15
- **Status:** Accepted (owner: "I do not want step 5. I do not want to overfit to one loop")
- **Supersedes:** [D-013](D-013-route-from-road-geometry-kerb-side-left-lane.md), [D-014](D-014-gps-odometry-windowed-alignment.md); amends [D-005](D-005-continuous-loop-mission.md)

## Context
Step 5 ([PR #4](https://github.com/BITSAuto/cart_driver/pull/4)) followed one
recorded loop in world coordinates, aligned with GPS. It worked in the sim
(584 m, 0 brakes), but it ties the cart to a single pre-recorded path, needs a
position fix the real cart doesn't have, and does nothing for roads that
weren't recorded.

## Decision
Don't merge step 5. The cart drives along whatever road it is on, using
perception: the road-centre cost ([D-020](D-020-road-centre-cost-from-road-follower.md))
keeps it on the road, the planner and safety monitor handle obstacles. No GPS,
no route files.

## Consequences
- PR #4 was closed unmerged; its branch stays for reference. The planner
  still accepts an optional `route` cost object (from step 4), unused.
- A GPS / world position fix is no longer a blocker for the real cart (Q-004
  closed).
- New gaps that a route used to cover:
  - **junctions** (where the road opens out, nothing says which way to go;
    [Q-008](../06-open-questions/Q-008-junctions-and-a-to-b.md), now high
    priority);
  - **keeping left** (the centre cost centres on the whole road;
    [Q-015](../06-open-questions/Q-015-keep-left-without-a-route.md));
  - **roads too wide to see both edges** (tesla_city): only heading keeping.
