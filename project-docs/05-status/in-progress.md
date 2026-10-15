# In progress

## Road-centre cost — branch `feat/road-centre-cost`
- Planner cost from road_follower's centre-line estimator
  ([D-020](../03-decisions/D-020-road-centre-cost-from-road-follower.md),
  [road-centre-cost.md](../02-architecture/road-centre-cost.md)).
- Code, 6 tests and docs done; sim evaluation in the `road_loop` world on the
  Orin: [road-centre-cost.md](../09-testing-and-results/road-centre-cost.md).
- Built on `main`. Adds `config/road_loop.yaml` for road_follower's world.

## Step 5: loop route — not adopted
[PR #4](https://github.com/BITSAuto/cart_driver/pull/4) is to be closed
unmerged ([D-021](../03-decisions/D-021-no-recorded-routes.md)). Its branch
`feat/step5-loop-route` stays as a record.
