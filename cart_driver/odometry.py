"""Dead-reckoning odometry of the rear-axle pose in a local ``odom`` frame.

Speed comes from the wheel encoders (/vehicle/speed or /bitsauto/speed);
heading change from the IMU's yaw rate when available, otherwise from the
bicycle model with the measured steering angle. Keeps a short pose history so
observations stamped in the recent past (perception lags by 50-200 ms) can be
placed where the vehicle actually was when they were taken.
"""

import bisect
import math

from cart_driver.vehicle import step_pose, steer_to_curvature, wrap


class Odometry:
    def __init__(self, wheelbase, history_s=5.0):
        self.wheelbase = wheelbase
        self.history_s = history_s
        self.pose = (0.0, 0.0, 0.0)
        self.time = None
        self.distance = 0.0
        self._times, self._poses = [], []

    def update(self, t, speed, steer_right_deg, yaw_rate=None):
        """Integrate from the last update to time ``t`` (s)."""
        if self.time is None:
            self.time = t
            self._record()
            return self.pose
        dt = t - self.time
        if dt <= 0:
            return self.pose
        if yaw_rate is not None and abs(speed) > 0.05:
            kappa = yaw_rate / speed
        else:
            kappa = steer_to_curvature(steer_right_deg, self.wheelbase)
        x, y, yaw = step_pose(self.pose, speed, kappa, dt)
        self.pose = (x, y, wrap(yaw))
        self.distance += abs(speed) * dt
        self.time = t
        self._record()
        return self.pose

    def _record(self):
        self._times.append(self.time)
        self._poses.append(self.pose)
        cutoff = self.time - self.history_s
        while self._times and self._times[0] < cutoff:
            self._times.pop(0)
            self._poses.pop(0)

    def pose_at(self, t):
        """Interpolated pose at time ``t``; clamps to the history's ends."""
        if not self._times:
            return self.pose
        i = bisect.bisect_left(self._times, t)
        if i <= 0:
            return self._poses[0]
        if i >= len(self._times):
            return self._poses[-1]
        t0, t1 = self._times[i - 1], self._times[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        (x0, y0, h0), (x1, y1, h1) = self._poses[i - 1], self._poses[i]
        return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), wrap(h0 + a * wrap(h1 - h0)))


def heading_between(p0, p1):
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])
