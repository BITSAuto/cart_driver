# Running in the sim

```bash
distrobox enter ubuntu22 -- ~/ros2_ws/scripts/run_tesla_sim.sh       # terminal 1
ros2 launch cart_driver sim.launch.py                                # terminal 2, same ROS_DOMAIN_ID
python3 tools/simtools.py monitor 120 run.csv                        # terminal 3, optional
```
- Launch arguments: `model` (perception backend; default the sim student),
  `perception` (default true), `cart_params` (default
  `config/tesla_sim.yaml`).
- Perception takes a minute or two to load; until the first grid the car
  holds still, then the driver starts at 5 km/h.
- `ros2 topic echo /cart/state` shows what the driver is doing, and the
  distance from the road centre when a centre line is fitted.
- An emergency brake is **latched**: after the car stops, call
  `ros2 service call /cart_safety/reset std_srvs/srv/Trigger`, or restart the
  sim to return to the start pose.
- Stop the launch with Ctrl-C in its own terminal; if you start it from a
  script, restore SIGINT first.
