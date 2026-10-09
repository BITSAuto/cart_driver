# D-014 · Align odom to the world with GPS over a 30 m window

- **Date:** 2026-10-08
- **Status:** In review ([PR #4](https://github.com/BITSAuto/cart_driver/pull/4))

The planner works in `odom`; the route is in the world frame. Pair each GPS
fix with odometry of the same point at the same time, and fit a 2D rigid
transform (Kabsch) over the last 30 m; before 3 m of travel use the known
start heading. A recent window absorbs odometry drift without a full
localisation stack. The planner's estimate matched GPS truth to 0.05 m mean.
On the real cart this needs a GPS (or other world fix), which doesn't exist
yet.
