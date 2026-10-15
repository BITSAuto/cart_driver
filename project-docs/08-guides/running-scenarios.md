# Running scenarios

Use a fresh sim for each scenario (the car must be at the start pose).

```bash
python3 tools/simtools.py spawn box1 20 0 0.6 0.6 0.8             # box 20 m ahead of the bumper, in the lane
python3 tools/simtools.py spawn paper 15 0.5 0.6 0.4 0.002 --flat # flat clutter, drivable
python3 tools/simtools.py remove box1 paper
python3 tools/simtools.py monitor 90 run.csv --until-stopped        # log and summarise
```
`spawn NAME AHEAD LEFT SX SY SZ [--yaw 3.1415] [--flat] [--color "r g b"]`
places a box relative to the GPS (front bumper) along the start heading.

`monitor` records speed, safety action, brake, steering, GPS, collision
distance and route cross-track at 10 Hz on sim time, and prints: distance,
speed while driving, safety state changes, emergency-brake engagements (with
speed and distance), and route cross-track once within 1 m.

The scenarios behind the recorded results:

| ID | Setup | Pass criteria |
| --- | --- | --- |
| A | box in the lane 20 m ahead (step 2, no steering) | speed held ~5 km/h; brakes only once coasting can't stop |
| B | stop perception mid-drive | `coast` at ~0.5 s, 0 brakes |
| C | box 3.5 m to the side; drive past | box still in the map out of view, near its true position |
| D | box in the lane 20 m ahead | steers around it, 0 brakes, no coasting |
| E | paper, paint line, dark stain in the lane (`--flat`) | drives straight over, 0 brakes |
| F | box appears 4.5 m ahead while cruising | emergency brake, stops short |
| L | follow the loop for 420 s (step 5, not adopted) | > 1 lap, 0 brakes, cross-track < 1 m |

The shell wrappers that ran these lived outside the repo
([Q-011](../06-open-questions/Q-011-scenario-scripts-not-in-repo.md)).
