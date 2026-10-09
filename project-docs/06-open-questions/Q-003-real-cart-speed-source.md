# Q-003 · Where does the real cart's measured speed come from?

- **Opened:** 2026-10-09 · **Status:** Open (hardware choice) · **Priority:** **blocking** for real driving

Nothing publishes `/vehicle/speed` on the cart. An early message from the
owner (2026-10-08) said the cart has no wheel encoders. Options, best first:
1. a wheel or motor-shaft sensor (hall effect or encoder);
2. visual(-inertial) odometry from the D435i (e.g. cuVSLAM on the Orin);
3. GNSS velocity (noisy at 5 km/h).
Whatever it is should publish `std_msgs/Float32` m/s (or set `speed_topic`
and `speed_scale`).
