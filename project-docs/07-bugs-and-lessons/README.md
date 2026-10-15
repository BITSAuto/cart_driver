# 07 · Bugs and lessons

| Page | Bugs |
| --- | --- |
| [driving-stack-bugs.md](driving-stack-bugs.md) | M-01 false e-brake at the start line · M-02 bonnet lethal everywhere · M-03 over-swerve and drift · M-04 infeasible swerve test · M-05 suspected odometry sign bug · M-06 shutdown tracebacks · M-13 blocked by a box on a narrow road |
| [sim-and-process-pitfalls.md](sim-and-process-pitfalls.md) | M-07 wall-clock timing · M-08 SIGINT ignored by background launches · M-09 `pkill -f` killing the shell · M-10 build mode error · M-11 RViz "No map received" · M-12 recorded centre line drifts |

## Lessons
- Check the world before tuning thresholds: depth sees sim artefacts as real.
- One bad frame must never brake: confirm over consecutive ticks.
- Launch arguments are global.
- Compare like with like: confirm two logs are from the same run before
  chasing a sign bug.
- Time everything on sim time.
