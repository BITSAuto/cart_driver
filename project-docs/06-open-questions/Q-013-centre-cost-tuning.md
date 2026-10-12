# Q-013 · Tuning and coverage of the road-centre cost

- **Opened:** 2026-10-12 · **Status:** Open · **Priority:** medium

Untested so far: combined with a route (PR #4), with obstacles in the lane in
`road_loop`, with semantic perception (grass lethal) instead of geometry
only, and on real campus roads. The weights (1.5 per m, 2 per rad) were
copied from the route cost; convergence from 1.2 m off-centre takes about
25 m in the unit test, which may be too gentle.
