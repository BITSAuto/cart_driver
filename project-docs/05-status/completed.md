# Completed

Newest first.

## 2026-10-15 · Single config — [PR #7](https://github.com/BITSAuto/cart_driver/pull/7)
`config/road_loop.yaml` removed: without a route it was identical to
`tesla_sim.yaml`, so one config serves every world.

## 2026-10-12 → 2026-10-15 · Road-centre cost — [PR #6](https://github.com/BITSAuto/cart_driver/pull/6), merged `f7ed532`
road_follower's centre-line estimator as a planner cost
([D-020](../03-decisions/D-020-road-centre-cost-from-road-follower.md)); step 5
dropped ([D-021](../03-decisions/D-021-no-recorded-routes.md)); the sim model
default picks what each machine can run. Results:
[road-centre-cost.md](../09-testing-and-results/road-centre-cost.md).

## 2026-10-09 · Project docs — [PR #5](https://github.com/BITSAuto/cart_driver/pull/5), merged `35f1f5f`
This book ([D-019](../03-decisions/D-019-project-docs.md)).

## 2026-10-08 · Step 4, planner — [PR #3](https://github.com/BITSAuto/cart_driver/pull/3) (`7b8bf8b`, merged `a0dabc2` 2026-10-09)
Arc planner on the memory map, route-less heading keeping, scenarios D–F
([D-009](../03-decisions/D-009-relay-aware-arc-planner.md)–[D-012](../03-decisions/D-012-ten-metre-horizon.md)).

## 2026-10-08 · Step 3, memory map — [PR #2](https://github.com/BITSAuto/cart_driver/pull/2) (`1706eb8`, merged `02d0258`)
Memory map in `odom` at capture time, `ClockMap`, safety checks the map too,
scenario C ([D-008](../03-decisions/D-008-memory-map-in-odom-at-capture-time.md)).

## 2026-10-08 · Step 2, speed and safety — [PR #1](https://github.com/BITSAuto/cart_driver/pull/1) (`e48e2be`, `5cec4ff`, merged `8983e60`)
PI speed hold, odometry, independent safety monitor, `simtools.py`,
scenarios A–B, `cart_params` rename
([D-002](../03-decisions/D-002-fixed-5-kmh-throttle-only.md)–[D-007](../03-decisions/D-007-odometry-from-speed-steering-and-yaw-rate.md), [D-018](../03-decisions/D-018-launch-argument-cart-params.md)).

## 2026-10-08 · Package skeleton — `dba1187`
Repo BITSAuto/cart_driver (private) with the plan
([D-001](../03-decisions/D-001-custom-stack-not-nav2.md)).

## 2026-10-08 · Step 1, sim fidelity (other repos)
tesla_sim PR #1, vehicle_bridge PRs #1/#2, traversability PR #2 — all merged.
