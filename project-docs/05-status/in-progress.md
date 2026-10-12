# In progress

## Road-centre cost — branch `feat/road-centre-cost`
- Planner cost from road_follower's centre-line estimator
  ([D-020](../03-decisions/D-020-road-centre-cost-from-road-follower.md),
  [road-centre-cost.md](../02-architecture/road-centre-cost.md)).
- Code, 6 tests and docs done; sim evaluation in the `road_loop` world on the
  Orin: [road-centre-cost.md](../09-testing-and-results/road-centre-cost.md).
- Built on `main` (without step 5); merging it with PR #4 needs a check that
  route and centre costs combine sensibly ([Q-013](../06-open-questions/Q-013-centre-cost-tuning.md)).

## Step 5: loop route — [PR #4](https://github.com/BITSAuto/cart_driver/pull/4), open
- Branch `feat/step5-loop-route` (`30df058`), now based on `main`.
- `route.py` (Route, WorldAligner, RouteFollower), driver wiring,
  `routes/tesla_city_loop.yaml`, `make_route.py`, `record_route.py`,
  cross-track in `simtools monitor`, quieter Ctrl-C, 8 new tests (26 total).
- Result: scenario L ([step-5-loop.md](../09-testing-and-results/step-5-loop.md)).
- Waiting for the owner's review. When it merges, update this page,
  completed.md, and the D-013/D-014 statuses.
