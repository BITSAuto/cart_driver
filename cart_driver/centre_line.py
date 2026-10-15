"""The road's centre line as a planner cost, from road_follower's estimator.

road_model.RoadEstimator (vendored from road_follower) fits a centre line to
each perception grid, in the grid's ``camera_ground`` frame: it finds the free
span the vehicle is in, bounded by lethal road edges (curbs, walls, grass
marked lethal), and fits ``y = c0 + s x + k x^2 / 2`` through the span
centres, about 3-9.5 m ahead of the camera.

``CentreLine`` places that fit in the odom frame at the camera pose of the
frame's *capture* time (as the memory map does), so it stays put while the
vehicle moves on and the next frame arrives. The planner then scores each
candidate path by how far its body centre runs from the line and how well its
heading matches the line's at the end of the overlap. Collision and clearance
are untouched: an obstacle in the lane still rules candidates out, so the
vehicle leaves the centre to pass it and is pulled back afterwards.
"""

import math

import numpy as np


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class CentreLine:
    """A polyline in odom: ``points`` (M, 2), ``headings`` (M,), ``stamp`` (s)."""

    def __init__(self, points, headings, stamp, width=0.0):
        self.points = np.asarray(points, dtype=float)
        self.headings = np.asarray(headings, dtype=float)
        self.stamp = stamp
        self.width = width
        seg = np.diff(self.points, axis=0)
        self.seg_len = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)
        self.seg_dir = seg / self.seg_len[:, None]
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])

    @classmethod
    def from_estimate(cls, est, camera_pose, stamp, spacing=0.25):
        """``est``: an ok road_model.RoadEstimate (camera_ground frame);
        ``camera_pose``: (x, y, yaw) of the camera in odom at capture time."""
        n = max(2, int(math.ceil((est.x_max - est.x_min) / spacing)) + 1)
        xs = np.linspace(est.x_min, est.x_max, n)
        ys = est.c0 + est.slope * xs + 0.5 * est.curvature * xs * xs
        dydx = est.slope + est.curvature * xs
        cx, cy, cyaw = camera_pose
        c, s = math.cos(cyaw), math.sin(cyaw)
        pts = np.column_stack([cx + c * xs - s * ys, cy + s * xs + c * ys])
        return cls(pts, _wrap(cyaw + np.arctan(dydx)), stamp, est.width)

    def project(self, xy):
        """For points (K, 2): (along-track m from the line's start, signed
        lateral m, + = left of the line, line heading there). Beyond either end
        the end segment is extended, so the result is defined everywhere."""
        xy = np.atleast_2d(xy)
        a = self.points[:-1]
        d = xy[:, None, :] - a[None, :, :]                                  # (K, M-1, 2)
        t = (d * self.seg_dir[None]).sum(axis=2)                             # along each segment
        tc = np.clip(t, 0.0, self.seg_len[None])
        off = d - tc[..., None] * self.seg_dir[None]
        j = np.argmin((off ** 2).sum(axis=2), axis=1)                        # nearest segment
        k = np.arange(len(xy))
        tj = t[k, j]
        last = len(self.seg_len) - 1
        lo = np.where(j == 0, -np.inf, 0.0)                                  # extend before the start
        hi = np.where(j == last, np.inf, self.seg_len[j])                    # ...and past the end
        tj = np.clip(tj, lo, hi)
        u = self.seg_dir[j]
        dj = d[k, j]
        lateral = u[:, 0] * dj[:, 1] - u[:, 1] * dj[:, 0]
        along = self.s[j] + tj
        return along, lateral, np.arctan2(u[:, 1], u[:, 0])

    def lateral_at(self, xy):
        return self.project(xy)[1]

    def seed(self, camera_pose, x_near, limit=3.0):
        """Where this line lies at ``x_near`` ahead of a (new) camera pose, as
        the lateral offset the estimator should start its search from."""
        cx, cy, cyaw = camera_pose
        c, s = math.cos(cyaw), math.sin(cyaw)
        rel = self.points - np.array([cx, cy])
        xc = c * rel[:, 0] + s * rel[:, 1]
        yc = -s * rel[:, 0] + c * rel[:, 1]
        y = float(np.interp(x_near, xc, yc)) if np.all(np.diff(xc) > 0) else float(yc[np.argmin(abs(xc - x_near))])
        return float(np.clip(y, -limit, limit))

    def cost(self, poses, point_x, step_m):
        """Score candidate rollouts ``poses`` (N, S, 3) of rear-axle poses taken
        every ``step_m`` m. The body point ``point_x`` m ahead of the rear axle
        is compared with the line over the stretch of path the line covers.

        Returns (mean |lateral| (N,), heading error at the end of the stretch
        (N,)) or None if no part of the paths lies alongside the line.
        """
        n, s_count, _ = poses.shape
        body = poses[..., :2] + point_x * np.stack([np.cos(poses[..., 2]), np.sin(poses[..., 2])], axis=-1)
        start_along = self.project(body[0, :1])[0][0]
        arc = np.arange(s_count) * step_m
        ks = np.flatnonzero((start_along + arc >= 0.0) & (start_along + arc <= self.length))
        if len(ks) == 0:
            return None
        ks = ks[::max(1, len(ks) // 12)]                                      # ~12 samples is plenty
        pts = body[:, ks, :].reshape(-1, 2)
        _, lateral, heading = self.project(pts)
        lateral = lateral.reshape(n, len(ks))
        heading = heading.reshape(n, len(ks))
        cross = np.abs(lateral).mean(axis=1)
        heading_err = _wrap(poses[:, ks[-1], 2] - heading[:, -1])
        return cross, heading_err
