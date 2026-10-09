# Sim and process pitfalls

## M-07: Timing in wall-clock seconds
Webots runs at ~0.55× real time; measuring the speed model in wall time made
it look 0.58× too slow. Use `use_sim_time` and map sensor stamps with
`ClockMap`.

## M-08: Launches started as background jobs ignore SIGINT
`cmd &` in a non-interactive shell starts `cmd` with SIGINT ignored; Ctrl-C
and `kill -INT` don't stop it and orphans pile up. Start launches through a
wrapper that resets SIGINT to default before `exec` (e.g. a tiny Python
`os.execvp` wrapper), and check for leftover processes after test runs.

## M-09: `pkill -f` killed the calling shell
A stop command containing the pattern matched its own shell. Run the stop
script as its own command.

## M-10: `error: option --uninstall not recognized` on build
User-site setuptools vs colcon's develop mode. Build with
`PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select cart_driver`.

## M-11: RViz showed "No map received" for `/cart/local_map`
- **Found:** 2026-10-08 by the owner, testing step 3.
- **Cause:** the map only starts after the first perception grid (a minute or
  two of model loading); RViz had subscribed before. Also, the 40 m map
  follows the car, so with RViz at the origin it can be out of view. The car
  had also stopped on a (correct) emergency brake: step 3 doesn't steer, so
  it drove straight at the curving road edge.
- **What to do:** wait for perception, re-tick the display, zoom out to the
  car (`/cart/odom`), use the step-4+ code to see driving.

## M-12: A recorded centre line drifts at junctions
`record_route.py` follows the yellow line; where there is none (junctions) it
goes straight, drifting up to 1.7 m. Use the road geometry when it's known
([D-013](../03-decisions/D-013-route-from-road-geometry-kerb-side-left-lane.md)).
