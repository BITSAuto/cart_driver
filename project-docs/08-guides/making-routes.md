# Making routes (step 5, PR #4)

> **Not adopted (2026-10-15):** step 5 was not merged; see [D-021](../03-decisions/D-021-no-recorded-routes.md). Kept as a record.

A route is a YAML list of world-frame points (one every 0.5 m) for the GPS
point to follow, in driving order. `route_file` selects it.

**From road geometry** (best in tesla_city):
```bash
python3 tools/make_route.py routes/city_cw_inner.yaml --rounded-rect -45 105 -105 45 40.5 --cw --offset 2.69 --name city_cw_inner
```
Offsets: 2.69 inner left lane, 8.06 kerb-side lane (default), 0 centre line.

**From a recording** (CSV with `x,y` columns):
```bash
python3 tools/make_route.py routes/my_loop.yaml --csv my_path.csv --offset 0 --name my_loop
python3 tools/record_route.py centreline.csv          # sim: follows the yellow centre line
```
Use `--offset 0` if the recording was driven in the lane.

**Selecting it:** copy `config/tesla_sim.yaml`, set `route_file` (relative
names are looked up in `share/cart_driver/routes/`, which needs a rebuild for
new files; absolute paths don't) and `initial_world_yaw` (the start heading,
3.1415 at tesla_sim's spawn), then
`ros2 launch cart_driver sim.launch.py cart_params:=/path/to/config.yaml`.

**Rules:** closed loops only; point order = direction of travel; start near
the route facing along it; no turns tighter than ~11 m radius (Tesla);
obstacles still take priority.
