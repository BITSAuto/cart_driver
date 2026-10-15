# D-020 · Add a road-centre cost from road_follower's estimator to the planner

- **Date:** 2026-10-12
- **Status:** Accepted ([PR #6](https://github.com/BITSAuto/cart_driver/pull/6), merged 2026-10-15)

## Context
A separate session on the Orin (2026-10-09/10, `~/BITSAuto_All_repo`) built
`road_follower`: a steering-only policy that fits the road's centre line to
each perception grid and pure-pursues it with its own relay control. It
centres well in its `road_loop` world (mean 0.09 m) but does **not** avoid
obstacles, never slows or stops, and has no safety monitor. `cart_driver`
avoids obstacles and has the safety monitor but, without a route, only keeps
its heading, so it has no notion of "the middle of the road".

## Decision
Keep `cart_driver` in charge and use road_follower's estimator as a planner
cost:
- every grid goes through `road_model.RoadEstimator`; a fitted line is placed
  in odom at the capture-time camera pose (`centre_line.CentreLine`), seeded
  from the previous line;
- each candidate is scored by the mean distance of its body centre from the
  line (1.5 per m) and its heading error at the end of the overlap (2 per
  rad);
- while a line is fresh (≤ 1 s) it replaces route-less heading keeping.
  (A route cost could be added on top; routes were dropped by
  [D-021](D-021-no-recorded-routes.md).)

## Alternatives considered
- **Run road_follower and let cart_driver's safety monitor govern speed and
  braking:** two steering authorities (road_follower's raw relay commands
  override `/cmd_ackermann` steering in the sim), and still no avoidance.
- **Subscribe to road_follower's `/road_follower/centre_line`:** would need
  the road_follower node running, which also publishes steering commands.
- **Import `road_follower.road_model`:** road_follower is not in a git
  repository yet, so the package couldn't be reproduced from GitHub.

## Consequences
- `cart_driver/road_model.py` is a **vendored copy**; replace it with a
  dependency once road_follower is in git ([Q-012](../06-open-questions/Q-012-road-follower-not-in-git.md)).
- Needs lethal road edges on at least one side; in tesla_city (21.5 m roads)
  it rarely fits and behaviour is unchanged.
- Obstacles still win: collisions rule candidates out regardless of the
  centre cost.
