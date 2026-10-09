# D-007 · Odometry from speed, steering and IMU yaw rate

- **Date:** 2026-10-08
- **Status:** Accepted

Bicycle-model dead reckoning; when moving, curvature from the IMU yaw rate
(more reliable than the steering angle through slip and Ackermann
differences). No visual odometry for now. Drift in the sim: 0.02 m over
20.9 m. A pose history allows placing sensor data at its capture time.
