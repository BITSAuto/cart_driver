# Route following (step 5, [PR #4](https://github.com/BITSAuto/cart_driver/pull/4), closed unmerged)

> **Not adopted (2026-10-15):** step 5 was not merged; see [D-021](../03-decisions/D-021-no-recorded-routes.md). Kept as a record.

## The route
`routes/tesla_city_loop.yaml`: a closed 480 m loop in the **kerb-side left
lane** (8.06 m left of the centre line), anticlockwise, in the world frame,
describing the **GPS point** (the front bumper in tesla_sim), one point every
0.5 m. Generated from the road geometry by
`tools/make_route.py --rounded-rect -45 105 -105 45 40.5`
([D-013](../03-decisions/D-013-route-from-road-geometry-kerb-side-left-lane.md)).

## Odom → world (`WorldAligner`)
Each GPS fix is paired with odometry of the same point at the same time
(via `ClockMap` and `pose_at`), one pair per 0.5 m travelled. A 2D rigid
(Kabsch) fit over the last 30 m gives the transform; the window absorbs
odometry drift. Until the vehicle has moved 3 m, the heading comes from
`initial_world_yaw` ([D-014](../03-decisions/D-014-gps-odometry-windowed-alignment.md)).

## Progress (`RouteFollower`)
Nearest route point, searched only from 5 m behind to 30 m ahead of the last
one, so progress can't jump to a nearby stretch of the loop; wrapping past
the end counts a lap. The first fix searches the whole loop.

## Planner cost
Each candidate's end pose → its GPS point → world; cost = |cross-track| ×
1.5 + |heading error| × 2. Obstacles still win: collisions rule candidates
out, so the car leaves the lane to pass something and then returns.

## Limits
Closed loops only; the car must start near the route, roughly facing along
it; turns no tighter than ~11 m radius for the Tesla; needs a world fix
(GPS) — none exists on the real cart yet.
