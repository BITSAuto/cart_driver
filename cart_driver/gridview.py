"""Occupancy-grid lookups shared by the planner and the safety monitor.

A GridView wraps an int8 occupancy array (rows = y, cols = x; 0 free,
100 lethal, -1 unknown) with its origin and resolution, and answers: what is
at these points, and how far is each point from the nearest lethal cell.
"""

import math

import cv2
import numpy as np

FREE, LETHAL, UNKNOWN = 0, 100, -1


class GridView:
    def __init__(self, data, origin_x, origin_y, resolution):
        self.data = np.asarray(data, np.int8)
        self.origin_x, self.origin_y, self.res = float(origin_x), float(origin_y), float(resolution)
        self._clearance = None

    @classmethod
    def from_msg(cls, msg):
        data = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        return cls(data, msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution)

    def cells(self, xs, ys):
        cols = np.floor((np.asarray(xs) - self.origin_x) / self.res).astype(np.int64)
        rows = np.floor((np.asarray(ys) - self.origin_y) / self.res).astype(np.int64)
        inside = (rows >= 0) & (rows < self.data.shape[0]) & (cols >= 0) & (cols < self.data.shape[1])
        return rows, cols, inside

    def value_at(self, xs, ys, outside=UNKNOWN):
        rows, cols, inside = self.cells(xs, ys)
        out = np.full(np.shape(xs), outside, np.int8)
        out[inside] = self.data[rows[inside], cols[inside]]
        return out

    @property
    def clearance(self):
        """Distance (m) from each cell to the nearest lethal cell."""
        if self._clearance is None:
            free = (self.data != LETHAL).astype(np.uint8)
            if free.all():
                self._clearance = np.full(self.data.shape, np.inf, np.float32)
            else:
                self._clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        return self._clearance

    def clearance_at(self, xs, ys, outside=np.inf):
        rows, cols, inside = self.cells(xs, ys)
        out = np.full(np.shape(xs), outside, np.float32)
        out[inside] = self.clearance[rows[inside], cols[inside]]
        return out


def clear_footprint(view, pose, circles):
    """Copy of ``view`` with lethal cells under the vehicle's current
    footprint cleared. Nothing real can be there -- the vehicle would already
    be touching it -- so these are artefacts (body parts seen by the camera,
    noise) that would otherwise read as a collision at distance 0."""
    data = view.data.copy()
    rows, cols = np.mgrid[0:data.shape[0], 0:data.shape[1]]
    xs = view.origin_x + (cols + 0.5) * view.res
    ys = view.origin_y + (rows + 0.5) * view.res
    inside = np.zeros(data.shape, bool)
    for (cx, cy), (_, r) in zip(footprint_points([pose], circles)[0], circles):
        inside |= (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
    data[inside & (data == LETHAL)] = FREE
    return GridView(data, view.origin_x, view.origin_y, view.res)


def footprint_points(poses, circles):
    """Centres of footprint circles for (N, 3) rear-axle poses -> (N, C, 2)."""
    poses = np.asarray(poses, float)
    offs = np.array([c[0] for c in circles])
    cy, sy = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    return np.stack((poses[:, 0:1] + offs * cy, poses[:, 1:2] + offs * sy), axis=-1)


def collision_distance(view, poses, circles, step):
    """Arc length (m) along ``poses`` (spaced ``step`` m) to the first pose
    whose footprint touches a lethal cell; inf if none does."""
    pts = footprint_points(poses, circles)
    radii = np.array([c[1] for c in circles])
    clear = view.clearance_at(pts[..., 0], pts[..., 1])
    hit = (clear < radii[None, :]).any(axis=1)
    if not hit.any():
        return math.inf
    return float(np.argmax(hit) * step)
