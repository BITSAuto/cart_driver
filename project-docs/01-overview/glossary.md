# Glossary

| Term | Meaning |
| --- | --- |
| **Coast** | Throttle off (speed command 0); friction slows the cart. The normal way to slow or stop |
| **Coast distance** | `v² / (2 · coast_decel)`: how far the cart rolls when the throttle is released |
| **Stop distance** | coast distance + `v · reaction_time` + `margin`; the safety threshold |
| **Emergency brake** | `/vehicle/emergency_brake`; only the safety node publishes it; latched |
| **Safety action** | `drive`, `coast` or `emergency_brake`, published on `/cart/safety` |
| **Blocked** | No collision-free candidate path; the driver coasts |
| **Rear-axle pose** | `(x, y, yaw)` of the rear axle centre in `odom`; the vehicle reference point |
| **Footprint circles** | The body approximated by circles along its axis, for collision checks |
| **Memory map** | The rolling 40 m local map in `odom` that remembers perception for 15 s |
| **ClockMap** | Maps wall-clock sensor stamps to sim time |
| **Candidate** | One planner path: "steer towards target T at the relay rate, then hold" |
| **Reference heading** | Slow average of the heading the planner returns to when there is no route |
| **Route** | A closed polyline in the world frame for the GPS point to follow |
| **Cross-track** | Signed distance from the route; + = left of it |
| **Kerb-side left lane** | The lane next to the left road edge (keep left, India) |
| **Profile** | `VehicleParams` for a vehicle: `tesla_sim` or `cart` |
