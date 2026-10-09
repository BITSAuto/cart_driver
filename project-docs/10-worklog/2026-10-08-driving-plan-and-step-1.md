# 2026-10-08 · Driving plan and step 1 (sim fidelity)

**Who:** project owner, with Claude Code
**Goal:** a plan for the driving algorithm under the cart's constraints.

## What was done
- Owner's constraints: emergency-only brake, fixed 5 km/h, coast to slow,
  continuous loop, dimensions later, camera height from the bag
  ([D-002](../03-decisions/D-002-fixed-5-kmh-throttle-only.md), [D-005](../03-decisions/D-005-continuous-loop-mission.md)).
- Replaced the earlier Nav2 plan with this package's five steps ([D-001](../03-decisions/D-001-custom-stack-not-nav2.md)).
- Step 1 in other repos: tesla_sim PR #1, vehicle_bridge PR #1, traversability PR #2.
