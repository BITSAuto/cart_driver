# D-013 · The loop route comes from the road geometry, in the kerb-side left lane

- **Date:** 2026-10-08
- **Status:** In review ([PR #4](https://github.com/BITSAuto/cart_driver/pull/4))

## Context
A centre line recorded by following the yellow line (`record_route.py`, 526 m)
matched the world's analytic road geometry to 0.24 m on average, but drifted
up to 1.7 m at junctions where the line is missing.

## Decision
Generate the route from the geometry (straight segments plus 40.5 m-radius
corners) and offset it **8.06 m left** of the centre line: the kerb-side lane
under keep-left (India) rules, the natural lane for a slow vehicle. 480 m,
anticlockwise.

## Consequences
- `make_route.py` supports both sources; recorded GPS tracks (`--csv`) are
  the way to make routes on campus, where no analytic geometry exists.
