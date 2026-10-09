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

## M-06: Tracebacks on shutdown
- "rcl_shutdown already called" when Ctrl-C had already shut the context
  down: wrapped shutdown in try/except (step 2).
- `RuntimeError: Unable to convert call argument` from a callback cut off by
  Ctrl-C: swallowed only when `rclpy.ok()` is false (step 5, PR #4).
