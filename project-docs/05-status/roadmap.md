# Roadmap

## Sim (can be done remotely)
| Item | Why |
| --- | --- |
| Commit the scenario scripts (A–F, L) into `tools/` | they lived in a session scratch directory; results can't be reproduced by others ([Q-011](../06-open-questions/Q-011-scenario-scripts-not-in-repo.md)) |
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
7. Campus loop route once there is a position source ([Q-004](../06-open-questions/Q-004-world-fix-for-routes.md), [Q-005](../06-open-questions/Q-005-campus-loop-route.md)).
