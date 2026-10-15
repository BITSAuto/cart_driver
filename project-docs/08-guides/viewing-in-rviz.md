# Viewing in RViz

Run RViz with `use_sim_time:=true` on the same domain; fixed frame `odom`.

| Display | Topic |
| --- | --- |
| Map | `/cart/local_map` |
| Path | `/cart/plan` |
| Path | `/cart/centre_line` (road-centre cost) |
| Odometry or TF | `/cart/odom`, `odom → base_link` |
| Map (in `camera_ground`) | `/traversability/grid` |

"No map received" right after launch is normal: the memory map starts with
the first perception grid. The map is 40 m and follows the car, so zoom out
or focus on the car ([sim-and-process-pitfalls.md](../07-bugs-and-lessons/sim-and-process-pitfalls.md)).
