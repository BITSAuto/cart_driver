# Q-014 · Passing an obstacle on a narrow road

- **Opened:** 2026-10-12 · **Status:** Open · **Priority:** high

On the 7 m `road_loop` road a box in the lane, first seen ~8 m ahead,
leaves the planner without a valid candidate and the car coasts to a stop
in front of it (sim and offline,
[road-centre-cost.md](../09-testing-and-results/road-centre-cost.md)). Each
candidate is one arc "steer to T, then hold" and must be collision-free for
the full 10 m: arcs that clear the box run into the opposite curb before
10 m. Stopping safely is correct behaviour, but a gap of 3.5 m beside the box
is wide enough for the Tesla.

Options:
- **Two-segment candidates** (steer to T1 for d m, then to T2): S-curves that
  swerve and straighten. Needs a vectorised rollout (21 × 21 candidates).
- Require collision-freedom only over a commit distance (stop distance plus
  one re-plan) and turn later collisions into a cost. Simpler, but gives up
  the guarantee that the chosen path is clear to the horizon.
- The cart's shorter wheelbase will turn tighter than the Tesla; re-check with
  the real profile once its dimensions are known.
