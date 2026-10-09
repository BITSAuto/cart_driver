# Vehicle profiles (`vehicle.py`)

`VehicleParams(wheelbase, width, front_overhang, rear_overhang, camera_x,
max_steer_deg, steer_rate_deg_s, cruise_kmh=5.0)`. Positions are measured
from the rear axle.

| Profile | Wheelbase | Width | Front / rear overhang | Camera x | Steer | Rate | Status |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `tesla_sim` | 2.94 | 1.85 | 0.85 / 0.95 | 1.67 | ±15° | 12.86 °/s | Verified from the PROTO and world |
| `cart` | 1.7 | 1.2 | 0.4 / 0.4 | 1.8 | ±15° | 12.86 °/s | **placeholders** except steering ([Q-001](../06-open-questions/Q-001-cart-dimensions.md)) |

`footprint_circles(margin)` approximates the body with circles along its
axis (radius = half width + margin), used by both the planner (0.15 m hard
margin) and the safety node (0.1 m). The GPS point is wheelbase + front
overhang ahead of the rear axle (3.79 m on the Tesla).
