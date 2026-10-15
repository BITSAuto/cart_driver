# Roadmap

## Sim (can be done remotely)
| Item | Why |
| --- | --- |
| Junction behaviour without a route | nothing decides which way to go ([Q-008](../06-open-questions/Q-008-junctions-and-a-to-b.md)) |
| Keep left with the road-centre cost | it centres on the whole road ([Q-015](../06-open-questions/Q-015-keep-left-without-a-route.md)) |
| S-shaped candidates for narrow roads | blocked by a box on a 7 m road ([Q-014](../06-open-questions/Q-014-narrow-road-obstacle-avoidance.md)) |
| Commit the scenario scripts (A–F) into `tools/` | they lived in a session scratch directory; results can't be reproduced by others ([Q-011](../06-open-questions/Q-011-scenario-scripts-not-in-repo.md)) |
| Tune lateral tracking (−0.6 m after corners) | [Q-010](../06-open-questions/Q-010-lateral-offset-after-corners.md) |
| Moving obstacles / pedestrians | untested ([Q-007](../06-open-questions/Q-007-moving-obstacles.md)) |
| Open routes, stopping at a destination | not supported ([Q-006](../06-open-questions/Q-006-open-routes-and-stopping.md)) |
| Use the cart profile in the sim once dimensions are known | |

## Real cart (in order)
1. A speed source publishing `/vehicle/speed` ([Q-003](../06-open-questions/Q-003-real-cart-speed-source.md)).
2. vehicle_bridge: command timeout, encoder calibration, throttle table.
3. Measure dimensions and coast rate ([Q-001](../06-open-questions/Q-001-cart-dimensions.md), [Q-002](../06-open-questions/Q-002-real-coast-deceleration.md)).
4. `launch/cart.launch.py` + `config/cart.yaml` with a **shadow mode**
   (driver and safety run but `/cmd_ackermann` and `/vehicle/emergency_brake`
   are remapped away from the bridge).
5. Shadow-mode drive with a human driving; record a bag; replay to check for
   false brakes and planner choices.
6. Live tests following [08-guides/real-cart-test-plan.md](../08-guides/real-cart-test-plan.md).
7. Road following on campus roads with clear edges (no recorded routes, [D-021](../03-decisions/D-021-no-recorded-routes.md)).
