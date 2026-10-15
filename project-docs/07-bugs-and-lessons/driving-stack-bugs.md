# Driving-stack bugs (2026-10-08)

## M-01: False emergency brake at the start line
- **Symptom:** the safety node braked right after starting.
- **Root cause:** tesla_sim's pedestrian crossings were slabs ~0.095 m above
  the road; depth saw a step.
- **Fix:** crossings lowered (tesla_sim D-019); `clear_footprint` ignores
  lethal cells under the vehicle; braking needs 2 consecutive ticks.

## M-02: The bonnet was lethal in every grid
- **Root cause:** `sim.launch.py` declared `params`; launch arguments are
  global, so traversability loaded this package's config and lost its bonnet
  mask.
- **Fix:** renamed to `cart_params` ([D-018](../03-decisions/D-018-launch-argument-cart-params.md), `5cec4ff`).

## M-03: The planner over-swerved and drifted without a route
- **Fix:** route-less heading keeping (τ 8 s) ([D-011](../03-decisions/D-011-route-less-heading-keeping.md)).

## M-04: The swerve test was infeasible
- **Root cause:** with a short horizon and an obstacle only ~3 m ahead, no
  relay-limited path could clear it.
- **Fix:** 10 m horizon; the test box placed where it's seen at ~8 m
  ([D-012](../03-decisions/D-012-ten-metre-horizon.md)).

## M-05: A suspected odometry sign bug that wasn't
- **What happened:** odometry and GPS seemed to disagree in sign. The logs
  were from two different runs. Compared within one run, odometry matched
  GPS (0.02 m over 20.9 m).
- **Rule:** confirm the data comes from the same run before debugging.

## M-13: Blocked by a box on a narrow road
- **Found:** 2026-10-12, in road_follower's `road_loop` world (7 m road).
- **Symptom:** `coast (blocked: no clear path)` ~3.5 m short of a box in the
  lane; 0 brakes.
- **Root cause:** single "steer then hold" arcs, each required to be clear for
  10 m. Seen ~8 m ahead, every arc that clears the box hits the opposite curb
  within 10 m. Reproduced offline; independent of the road-centre cost.
- **Status:** open ([Q-014](../06-open-questions/Q-014-narrow-road-obstacle-avoidance.md)).
- **Rule:** test avoidance where the road is barely wider than needed, not
  only on wide roads (scenario D used a 21.5 m road).

## M-06: Tracebacks on shutdown
- "rcl_shutdown already called" when Ctrl-C had already shut the context
  down: wrapped shutdown in try/except (step 2).
- `RuntimeError: Unable to convert call argument` from a callback cut off by
  Ctrl-C: swallowed only when `rclpy.ok()` is false. That fix was part of
  step 5 (PR #4), which was not adopted, so it is **not on `main`**; port it
  separately.
