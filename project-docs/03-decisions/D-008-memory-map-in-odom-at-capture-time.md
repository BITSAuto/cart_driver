# D-008 · A short-term memory map in `odom`, placed at capture time

- **Date:** 2026-10-08
- **Status:** Accepted

Perception sees only ~2.7–10 m ahead; the inside of a turn is out of view.
Each grid is placed in a rolling 40 m map at the camera pose **when the frame
was captured** (perception lags by up to ~0.2 s), the newest observation of a
cell wins, and cells unseen for 15 s become unknown. Webots stamps sensors
with wall-clock time, so a `ClockMap` converts stamps to sim time.
