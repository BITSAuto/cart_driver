# Driving numbers

| Quantity | Value | Confidence |
| --- | --- | --- |
| Cruise | 5 km/h = 1.39 m/s | owner's constraint |
| Coast deceleration | 0.4 m/s² | **Assumed** for the cart; matches the sim (Verified 0.40) |
| Coast distance from 5 km/h | 2.41 m in 3.35 s (sim) | Verified (sim) |
| Emergency-brake threshold at 5 km/h | ≈ 2.4 + 0.42 + 0.5 ≈ 3.3 m | follows from the rule |
| E-brake stop from 5 km/h | ≈ 0.25 s (sim) | Verified (sim) |
| Steering rate | 12.86 °/s; straight → full lock ≈ 1.2 s ≈ 1.6 m at 5 km/h | Verified from hardware constants |
| Tightest turn (Tesla, ±15°) | ≈ 11 m radius | Verified (geometry) |
| Lateral shift available | Tesla needs ≈ 7 m of travel to move 1.7 m sideways | Verified (planner rollouts) |
| Obstacle seen at < ~5 m in the lane | can't be avoided → emergency brake | Verified (scenario F) |
