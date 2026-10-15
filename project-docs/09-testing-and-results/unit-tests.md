# Unit tests

```bash
python3 -m pytest test
```
| File | Covers |
| --- | --- |
| `test/test_core.py` (8) | steering sign convention; pose stepping on arcs and straights; odometry uses yaw rate and history; speed controller never brakes; `safety_action`; grid clearance and collision distance; footprint covers the body; `clear_footprint` removes only self artefacts |
| `test/test_local_map.py` (4) | an obstacle is remembered after leaving view, then expires; a rotated frame lands in the right place and newer free clears; recentring keeps world positions; `ClockMap` |
| `test/test_planner.py` (6) | rollouts respect the relay rate and sign; straight on a clear road; swerves around an obstacle in the lane; follows a bending road; a blocked road is invalid; heading keeping steers back |
| `test/test_centre_line.py` (6, branch `feat/road-centre-cost`) | estimator on an offset road; line placed in odom; projection beyond the ends; planner steers towards the line; obstacles override it; closed loop on a bend |

18 tests on `main`; 24 with the road-centre cost (2026-10-12). (Step 5's 8 route tests were not adopted.)
