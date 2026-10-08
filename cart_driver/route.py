"""Loop route following: where the route is, and how far each plan ends from it.

The route (routes/*.yaml, made by tools/make_route.py) is a closed polyline
in the *world* frame describing where the GPS antenna point (``gps_x`` m
ahead of the rear axle; the front bumper in tesla_sim) should go. The
planner works in the *odom* frame, so three pieces join them:

  Route         the polyline, its headings and left normals; nearest-point
                queries restricted to a window of indices
  WorldAligner  odom -> world rigid transform, fitted (2D Kabsch) to the last
                ``window_m`` of (odometry, GPS) point pairs. Fitting a recent
                window rather than the whole history absorbs odometry drift.
                Until the vehicle has moved ``min_spread`` m the heading
                can't be fitted, so the known start heading is used.
  RouteFollower progress along the loop (monotonic, wraps around, never jumps
                across the loop to a nearby edge) and ``cost(end_poses)`` for
                ArcPlanner: cross-track distance and heading error of each
                candidate's end pose.
"""

import math

import numpy as np
import yaml

from cart_driver import vehicle


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Route:
    def __init__(self, points, closed=True, name='route'):
        self.points = np.asarray(points, dtype=float)
        self.closed = closed
        self.name = name
        nxt = np.roll(self.points, -1, axis=0) if closed else np.vstack([self.points[1:], self.points[-1:]])
        prv = np.roll(self.points, 1, axis=0) if closed else np.vstack([self.points[:1], self.points[:-1]])
        d = nxt - prv
        self.heading = np.arctan2(d[:, 1], d[:, 0])
        self.normal = np.column_stack([-np.sin(self.heading), np.cos(self.heading)])     # left
        seg = np.hypot(*np.diff(np.vstack([self.points, self.points[:1]]) if closed else self.points, axis=0).T)
        self.length = float(seg.sum())
        self.spacing = self.length / len(seg)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            doc = yaml.safe_load(f)
        return cls(doc['points'], closed=doc.get('closed', True), name=doc.get('name', 'route'))

    def __len__(self):
        return len(self.points)

    def window(self, start, n):
        idx = np.arange(start, start + n)
        return idx % len(self) if self.closed else np.clip(idx, 0, len(self) - 1)

    def nearest(self, xy, idx=None):
        """For each point in ``xy`` (K, 2): (route index, signed cross-track
        m (+ = left of the route), route heading). Searches only ``idx``."""
        xy = np.atleast_2d(xy)
        idx = np.arange(len(self)) if idx is None else np.asarray(idx)
        diff = xy[:, None, :] - self.points[idx][None, :, :]
        j = idx[np.argmin((diff ** 2).sum(axis=2), axis=1)]
        cross = ((xy - self.points[j]) * self.normal[j]).sum(axis=1)
        return j, cross, self.heading[j]


class WorldAligner:
    """odom -> world: world = R(theta) @ odom + t."""

    def __init__(self, initial_yaw, window_m=30.0, spacing=0.5, min_spread=3.0):
        self.initial_yaw = initial_yaw
        self.max_pairs = max(2, int(window_m / spacing))
        self.spacing = spacing
        self.min_spread = min_spread
        self.odom, self.world = [], []
        self.theta, self.t = None, None

    @property
    def ready(self):
        return self.theta is not None

    def add(self, odom_xy, world_xy, odom_yaw):
        """One (odometry, GPS) pair for the same point at the same time."""
        if self.odom and math.dist(odom_xy, self.odom[-1]) < self.spacing:
            return
        self.odom.append(tuple(odom_xy))
        self.world.append(tuple(world_xy))
        del self.odom[:-self.max_pairs], self.world[:-self.max_pairs]
        self._fit(odom_yaw)

    def _fit(self, odom_yaw):
        a, b = np.array(self.odom), np.array(self.world)
        if np.ptp(a, axis=0).max() < self.min_spread and len(self.odom) < self.max_pairs:
            # Too little travel to fit a heading: trust the known start heading,
            # place the newest pair exactly.
            self.theta = vehicle.wrap(self.initial_yaw - odom_yaw) if self.theta is None else self.theta
            self.t = b[-1] - self._rotate(a[-1])
            return
        ca, cb = a - a.mean(axis=0), b - b.mean(axis=0)
        dot = (ca * cb).sum()
        crs = (ca[:, 0] * cb[:, 1] - ca[:, 1] * cb[:, 0]).sum()
        self.theta = math.atan2(crs, dot)
        self.t = b.mean(axis=0) - self._rotate(a.mean(axis=0))

    def _rotate(self, xy):
        c, s = math.cos(self.theta), math.sin(self.theta)
        xy = np.asarray(xy, dtype=float)
        return np.stack([c * xy[..., 0] - s * xy[..., 1], s * xy[..., 0] + c * xy[..., 1]], axis=-1)

    def to_world(self, xy):
        return self._rotate(xy) + self.t

    def to_odom(self, xy):
        c, s = math.cos(self.theta), math.sin(self.theta)
        d = np.asarray(xy, dtype=float) - self.t
        return np.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]], axis=-1)


class RouteFollower:
    """The ``route`` object ArcPlanner scores candidates against."""

    def __init__(self, route, aligner, point_x, behind_m=5.0, ahead_m=30.0):
        self.route = route
        self.aligner = aligner
        self.point_x = point_x                       # route point, m ahead of the rear axle
        self.behind = int(behind_m / route.spacing)
        self.ahead = int(ahead_m / route.spacing)
        self.progress = None                         # route index
        self.laps = 0
        self.cross_track = math.nan
        self.heading_error = math.nan

    def _world(self, poses):
        poses = np.atleast_2d(poses)
        pts = np.column_stack([poses[:, 0] + self.point_x * np.cos(poses[:, 2]),
                               poses[:, 1] + self.point_x * np.sin(poses[:, 2])])
        return self.aligner.to_world(pts), _wrap(poses[:, 2] + self.aligner.theta)

    def update(self, pose):
        """Advance progress from the current rear-axle pose (odom)."""
        xy, yaw = self._world(pose)
        if self.progress is None:
            idx = None                                # first fix: whole loop
        else:
            idx = self.route.window(self.progress - self.behind, self.behind + self.ahead)
        j, cross, heading = self.route.nearest(xy, idx)
        j = int(j[0])
        if self.progress is not None and self.route.closed and j < self.progress - len(self.route) // 2:
            self.laps += 1
        self.progress = j
        self.cross_track = float(cross[0])
        self.heading_error = float(_wrap(yaw[0] - heading[0]))
        return self.progress

    @property
    def distance(self):
        """Distance along the route since lap 0, index 0 (m)."""
        return (self.laps * len(self.route) + (self.progress or 0)) * self.route.spacing

    def cost(self, end_poses):
        xy, yaw = self._world(end_poses)
        idx = self.route.window(self.progress - self.behind, self.behind + self.ahead)
        _, cross, heading = self.route.nearest(xy, idx)
        return np.abs(cross), _wrap(yaw - heading)
