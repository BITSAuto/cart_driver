# Planner (step 4)

`ArcPlanner` in the driver, re-planned at 10 Hz on the memory map.

## Candidates
21 targets from −15° to +15°. Each is "steer towards T at the relay's real
12.86 °/s, then hold", starting from the **measured** wheel angle, rolled out
at cruise speed in 0.15 m steps for 10 m (the front reaches the far edge of
perception).

## Scoring

| Term | Weight | Meaning |
| --- | --- | --- |
| collision | — | any footprint circle (0.15 m hard margin) on a lethal cell rules the candidate out; distance to the collision is kept to rank invalid ones |
| clearance | 4 | squared intrusion into a 1 m soft band around lethal cells |
| unknown | 2 | share of the path over unknown cells |
| route (step 5) | 1.5 per m | end point's distance from the route |
| route heading (step 5) | 2 per rad | end heading vs the route's |
| centre (road edges seen) | 1.5 per m | mean distance of the body centre from the road centre line ([road-centre-cost.md](road-centre-cost.md)) |
| centre heading | 2 per rad | heading vs the centre line at the end of the overlap |
| heading (no route, no centre line) | 3 per rad | end heading vs a slow reference heading (τ 8 s) |
| smoothness | 0.02 per ° | change from the previous target (relay chatter) |
| steering | 0.005 per ° | preference for straight |

The cheapest valid candidate wins. If none is valid, the driver **coasts**
("blocked"); braking stays the safety monitor's call.

## Why this design
Nav2's controllers assume smoothly variable speed and free braking; neither
holds here. A receding horizon of simple relay-aware arcs gives smooth,
feedback-corrected paths that the steering can actually follow
([D-009](../03-decisions/D-009-relay-aware-arc-planner.md)).

## Limits
With ±15° and the relay ramp, the Tesla needs about 7 m of travel to move
1.7 m sideways, so avoidance needs early detection (obstacles appearing
closer than ~5 m end in an emergency brake). The cart's shorter wheelbase
should turn tighter.
