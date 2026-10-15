# Road-centre cost (2026-10-12 and 2026-10-15, PR #6)

Orin, tesla_sim headless at ~0.30× real time, road_follower's `road_loop`
world (7 m road, 0.15 m curbs, 366 m loop), geometry-only perception (as
road_follower's own results), Tesla profile, 5 km/h. Scored with
road_follower's `tools/eval_run.py` (rear axle against the world's true
centre line, from GPS + wheel angle).

## Loop, no obstacles (330 s sim)

| Run | Distance | Rear-axle offset from the true centre | Brakes / coasting |
| --- | --- | --- | --- |
| cart_driver + road-centre cost | **460 m (1.26 laps)**, never left the road | mean **0.14 m**, p95 0.38 m, max 0.47 m | 0 / 0 (5 km/h throughout) |
| road_follower (its own result, 2026-10-10, for comparison) | 330 s, 1.26 laps | mean 0.09 m, p95 0.21 m, max 0.32 m | n/a (no safety monitor) |

Split by section: straights mean 0.12 m (signed −0.07 m); corners mean
0.17 m, signed **+0.16 m towards the inside**: the cost centres the body's
midpoint, so the rear axle cuts inside on the 20 m-radius bends. The fitted
road width read 6.9 m on straights and up to ~8 m in bends.

Not measured: the same loop without the cost. With only route-less heading
keeping nothing makes it follow the bends, so it would be expected to block
or brake at the first one.

## Box in the lane (120 s sim)
A 0.6 × 0.6 × 0.8 m box at (15.0, −50.3), i.e. 0.3 m right of centre and
11.2 m ahead of the bumper at the start.

**Result: blocked.** The car set off, turned slightly left (y +0.17 m), and
stopped by coasting at rear-axle x ≈ 7.45 m with `coast (blocked: no clear
path)`: no candidate was collision-free. 0 emergency brakes, no contact.

**Cause (reproduced offline):** perception first sees the box when it is
~8 m ahead of the bumper. On a 7 m road, every single "steer then hold" arc
that clears the box keeps turning into the opposite curb within the 10 m
horizon, so the planner has no valid candidate. Offline with the same
geometry: blocked at x = 7.0, y = +0.13 **with and without** the centre cost;
passes if the box is known from 15 m away, or on a 12 m road. A planner
limitation, not caused by the centre cost
([Q-014](../06-open-questions/Q-014-narrow-road-obstacle-avoidance.md)).

## Unit tests
24 pass (6 new): estimator on an offset road; placing the line in odom;
projection beyond the line's ends; the planner steering towards a line on
either side; obstacles still overriding the line; a closed loop on a 40 m
bend (1.2 m off-centre → 0.00 m over ~25 m).

## Laptop re-test (2026-10-15, branch at `0dfee75` + model-default change)
Humble laptop, ~0.43× real time, **full perception** (sim-trained SegFormer-B0
RGB-D + depth), and `road_loop` with the sensors moved to the **real cart
mount** (1.52 m, 9° down). The Orin's `road_loop.wbt` still has the old
level camera at ~1.3 m, which the runs above used.

| Test | Result |
| --- | --- |
| A. `road_loop`, no obstacles, 330 s | **461 m (1.26 laps)**, 5.00 km/h (4.89–5.06), never left the road; rear axle **0.17 m** from the true centre on average (p95 0.44, max 0.57); 0 brakes; one 0.1 s `coast` at 22.5 s (perception briefly late) |
| B. tesla_city, box 20 m ahead in the lane, 120 s | steered round it (passed ~1.5–2 m to its side by the bumper GPS), **0 brakes**, no blocking; 166 m at 5.0 km/h |

In B the centre line never fitted (21.5 m road), yet the car followed the
first bend: heading keeping plus the clearance cost kept it along the
**outer** barrier, about 1 m from it. That is not lane-keeping, but it does
not stop at the bend as expected.
