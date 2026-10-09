# D-001 · A small custom driving stack, not Nav2

- **Date:** 2026-10-08
- **Status:** Accepted
- **Supersedes:** the 2026-10-07 "Cart Navigation Plan" (a private doc: Nav2 with Hybrid-A* and Regulated Pure Pursuit, LiDAR SLAM + GPS localisation, a campus A→B mission)

## Context
The owner set the real constraints on 2026-10-08: emergency-only brake, fixed
5 km/h, coasting to slow, relay steering, and a continuous loop rather than
A→B. Nav2's controllers assume smoothly variable speed and braking that is
freely available; neither is true here.

## Decision
A new package, `cart_driver`, with two nodes: a driver (odometry, memory map,
relay-aware planner, throttle-only speed hold, route following) and an
independent safety monitor.

## Consequences
- Simple enough to tune by hand and reason about; written around the cart's
  limits.
- Nav2 (e.g. MPPI) could be revisited later for A→B missions.
