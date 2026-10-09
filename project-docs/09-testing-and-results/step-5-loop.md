# Step 5: loop route (2026-10-08, PR #4)

| Scenario | Result |
| --- | --- |
| L. Follow `tesla_city_loop` from the start pose for 420 s | **584 m, more than one lap, without stopping**; 5.00 km/h mean (5.00–5.24); **0 brakes, 0 coasting**. Started 10 m right of the lane, merged within 21 m. Afterwards **0.31 m** from the route on average (p95 0.60, max 0.93) by GPS truth; the planner's own cross-track estimate agreed within 0.05 m on average. Repeating pattern: ~0.2 m inside on corners, up to 0.6 m outside where a straight follows a corner |

The track plot and cross-track graph were shared in the session; regenerate
from `simtools monitor` CSV output (`gps_x`, `gps_y`, `cross_track`).
