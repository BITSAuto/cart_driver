# D-011 · Without a route, return to a slowly updated reference heading

- **Date:** 2026-10-08
- **Status:** Accepted

Before routes existed, the planner over-swerved and drifted after avoiding
an obstacle. It now scores candidates by their end heading against a
reference heading that follows the actual heading with a slow time constant
(8 s), so the car comes back to its course after swerving. With a route
(step 5) the route cost replaces this.
