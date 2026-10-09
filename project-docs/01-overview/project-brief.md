# Project brief

## Mission (2026-10-08)
Drive the BITSAuto cart around **one continuous loop**, at a fixed
**5 km/h**, on the drivable area found by `traversability`, without stopping
("I don't really want to stop. Just one continuous loop path for now"), and
without using the brake except in a genuine emergency.

## Hard constraints (from the owner)
- **The brake is emergency-only.** It stops the cart instantly and can damage
  it. Speed is controlled with throttle alone; to slow down, release the
  throttle and let friction coast the cart.
- **Fixed 5 km/h**, low enough that every turn is possible.
- **Relay steering:** ~12.9 °/s, ±15°; plans must allow for the time the
  wheels take to turn.
- **Perception sees ~2.7–10 m ahead** in a narrow cone (the bonnet hides the
  near field), so obstacles leave the view before the cart passes them.
- Testing is in the Webots sim (tesla_sim) with the sim-trained
  SegFormer-B0 RGB-D + depth perception, because the owner works remotely.
- Cart dimensions: to be supplied later.

## The plan
1. **Sim fidelity** (in tesla_sim, vehicle_bridge, traversability): coasting,
   an emergency-only brake, ±15° steering, the real camera mount.
2. **Speed hold + odometry + an independent safety monitor.**
3. **Short-term memory map** in the `odom` frame.
4. **Local planner** on the memory map, plus avoidance scenarios.
5. **Loop route:** follow a closed keep-left loop continuously.

Each step landed as its own PR, stacked on the previous one.

## Status (2026-10-09)
Steps 1–4 merged. Step 5 is in review
([PR #4](https://github.com/BITSAuto/cart_driver/pull/4)): in the sim the car
drove 584 m (more than a lap) at 5.00 km/h with 0 brakes and 0 coasting,
0.31 m from the route on average. Nothing has run on the real cart: it needs
a speed source, a position fix for the route, its real dimensions and coast
rate ([05-status](../05-status/README.md)).

## Non-goals for now
Stopping at destinations, A→B routing on a campus map, junction decisions,
reversing, moving obstacles.
