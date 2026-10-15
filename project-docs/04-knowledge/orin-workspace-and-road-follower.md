# The Orin workspace and road_follower

**Verified** 2026-10-12, by reading the Orin directly.

### `~/BITSAuto_All_repo` on the Orin
- Created by a Claude Code session on the Orin, 2026-10-09/10. It moved the
  ROS workspace, models, data and `seg_benchmark` into one folder and added
  `env.sh` and `guides/`. **It is not a git repository.**
- `ros2_ws/src/cart_driver`, `traversability`, `tesla_sim` and
  `vehicle_bridge` are copies of our packages. `cart_driver` is a git clone
  (with two local path edits for the new layout); `tesla_sim` carries
  **uncommitted** changes from that session (a `/steering/command` input,
  30 Hz camera and depth, delay parameters).
- `ros2_ws/src/road_follower` exists only there.

### Setting the ROS domain
`ROS_DOMAIN_ID=23 source env.sh`, as the guides write it, does **not** keep
the domain: a prefix assignment only lasts for that command, so the shell
falls back to domain 0 and sees none of the topics. Use
`export ROS_DOMAIN_ID=23; source ~/BITSAuto_All_repo/env.sh`.
`ackermann_msgs` is built in that workspace, so `/cmd_ackermann` can't be
echoed until `env.sh` has been sourced.

### road_follower in brief
A steering-only policy: `road_model.py` fits the road centre line to each
grid; pure pursuit aims 4–8 m ahead on it; a relay driver with a fitted
actuation delay sends raw 49/50/51 on `/steering/command`. Constant speed on
`/cmd_ackermann` (steering 0 there; the modified sim ignores it while raw
commands arrive). No obstacle avoidance, no stopping, no safety monitor.
Its own results in `road_loop`: mean 0.09 m from the centre.
`vehicle_bridge` has no `/steering/command` input, so on the real cart it
wouldn't steer.

### The `road_loop` world
`road_follower/worlds/road_loop.wbt`: a 7 m road (6.9 m measured by the
estimator) with 0.15 m curbs, a 366 m loop, car spawned at (0, −50) heading
+x. Made because tesla_city's 21.5 m roads never show both edges to a 10 m
camera.

### Sim speed and streaming
- Headless (`xvfb`), about 0.3–0.35× real time: the camera and depth sensors
  are rendered under the virtual display, most likely on the CPU.
- `--stream` serves Webots' 3D view to a browser on port 8000 (viewer page)
  and 1234 (WebSocket). From the laptop a direct `ws://bitsauto-orin:1234`
  failed with error 1006 while a raw handshake worked; an SSH tunnel of both
  ports to `localhost` works.
- `stack.sh stop` leaves the viewer's web server running: it records the PID
  of a subshell (`( cd ... && setsid nohup python3 ... & echo $! ... )`), not
  the server's, so port 8000 stays taken.
