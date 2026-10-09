# Odometry and memory map (step 3)

## Odometry (`odometry.py`)
Bicycle-model dead reckoning at 20 Hz from measured speed and steering
angle; when moving, curvature comes from the IMU yaw rate (`κ = ω / v`)
instead of the steering angle. Keeps 5 s of pose history so `pose_at(t)` can
interpolate the pose at a sensor's capture time. Measured drift vs GPS:
0.02 m over 20.9 m (sim).

## Time sync (`timesync.py`)
`ClockMap` observes (wall time, sim time) pairs each tick, estimates the
rate over a 3 s window, and converts a wall-clock stamp to sim time.

## Memory map (`local_map.py`)
- **Why:** perception sees 2.7–10 m ahead in a narrow cone; the inside of a
  turn, where the body sweeps, is out of view.
- **What:** a 40 m × 40 m grid at 0.1 m in `odom`, recentred when the vehicle
  is more than 5 m from its centre (contents shifted).
- **Integrating a frame:** place the grid at the camera pose at capture time;
  2×2 subsampling; write free cells first, then lethal (lethal wins within a
  frame); the **newest observation of a cell wins** across frames, so an area
  seen free again clears an old obstacle.
- **Forgetting:** cells unseen for 15 s become unknown.
- Published at 2 Hz on `/cart/local_map`; the safety node checks it too.
