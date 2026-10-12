# 2026-10-12 · Road-centre cost from road_follower

**Who:** project owner, with Claude Code
**Branch / PR:** `feat/road-centre-cost`

## What was done
- Looked at the Orin's new `~/BITSAuto_All_repo` and its `road_follower`
  package; streamed the Webots view to the laptop over an SSH tunnel
  ([orin-workspace-and-road-follower.md](../04-knowledge/orin-workspace-and-road-follower.md)).
- Owner asked for road_follower's centre line as a cost term in the planner:
  vendored its estimator, added `centre_line.py`, planner and driver wiring,
  6 tests ([D-020](../03-decisions/D-020-road-centre-cost-from-road-follower.md),
  [road-centre-cost.md](../02-architecture/road-centre-cost.md)).
- Tested on the Orin in the `road_loop` world, with a copy of road_follower's
  `stack.sh` that runs cart_driver instead (in a scratch directory; its
  viewer PID bug fixed there).

## Results
[road-centre-cost.md](../09-testing-and-results/road-centre-cost.md): 1.26
laps, mean 0.14 m from the true centre, 0 brakes; a box in the 7 m lane
blocks the planner (M-13, Q-014; not caused by the new cost).

## Left for next time
- Two-segment candidates for narrow roads (Q-014).
- Put road_follower in git and drop the vendored copy (Q-012).
- Combine with the route (PR #4) and test (Q-013).
