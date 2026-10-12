# Road-centre cost (branch `feat/road-centre-cost`)

Keeps the vehicle in the middle of the road, using the road edges perception
sees, without giving up obstacle avoidance or the safety monitor.
Why: [D-020](../03-decisions/D-020-road-centre-cost-from-road-follower.md).

## Per perception grid (driver, `fit_centre`)
1. `road_model.RoadEstimator` (vendored from road_follower) looks at the grid
   in `camera_ground` from 3 to 9.5 m ahead, every 0.5 m. At each distance it
   takes the free span the vehicle is in: the first is searched from the
   previous line's position (the **seed**), later ones from the previous
   span's centre, so the search follows bends and can't jump over a curb.
2. A span edge is **hard** if it ends at lethal cells (curb, wall, grass marked
   lethal) and **soft** if it ends at unknown cells (edge of the camera's
   view). Two hard edges give the centre; one hard edge plus half the road
   width seen recently gives it too; no hard edge gives nothing.
3. A robust curvature-limited quadratic through the centres gives the line
   `y = c0 + s·x + k·x²/2`. It needs ≥ 4 samples covering ≥ 2.5 m, spans of
   2–14 m width.
4. `CentreLine.from_estimate` samples the fit every 0.25 m and places it in
   `odom` at the camera pose **at the frame's capture time** (the same pose the
   memory map uses). Published as `/cart/centre_line`.

## In the planner
For each of the 21 candidates, the body centre (wheelbase/2 ahead of the rear
axle) is compared with the line at ~12 points along the stretch of path the
line covers (the line is extended straight past its ends for this):

```
cost += 1.5 × mean |distance from the line|  +  2.0 × |heading error at the end of the stretch|
```

- Used only while the line is ≤ `centre_max_age` (1 s) old.
- While used, route-less heading keeping is off (it would fight bends).
- With a route (step 5, PR #4) both costs add.
- Collision and clearance are unchanged: an obstacle on the centre line rules
  candidates out, the car passes it, and the cost pulls it back afterwards.

## Parameters (driver)

| Parameter | Default | Meaning |
| --- | --- | --- |
| `road_centre` | `true` | run the estimator and use the cost |
| `centre_max_age` | 1.0 s | how long a fitted line is used |
| `road_x_near`, `road_x_far` | 3.0, 9.5 m | sample range ahead of the camera |
| `road_min_width`, `road_max_width` | 2.0, 14.0 m | spans outside this are not a road |
| `road_nominal_width` | 0 (none) | road width to assume before both edges have been seen |

## When it does nothing
Roads wider than perception can see across (tesla_city's 21.5 m), open
ground, junctions (the road opens out). Then no line is fitted and the
planner behaves as before.
