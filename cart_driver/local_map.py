"""Short-term memory map: rolling local occupancy in the odom frame.

Perception only sees 2.7-10 m ahead in a narrow cone. While turning, the
inside of the curve -- exactly where the cart's body will sweep -- is out of
view. This map remembers what was seen for ``memory_s`` seconds, placed in
the odom frame using the vehicle pose at each frame's capture time.

Each observed cell takes the latest observation (so an area seen as free
again clears an old obstacle); within one frame lethal wins over free.
Cells not observed for ``memory_s`` fall back to unknown. The map is a
square of ``size_m`` that re-centres on the vehicle by whole-cell shifts.
"""

import math

import numpy as np

from cart_driver.gridview import FREE, LETHAL, UNKNOWN, GridView


class LocalMap:
    def __init__(self, size_m=40.0, resolution=0.1, memory_s=15.0, recenter_m=5.0):
        self.res = resolution
        self.n = int(round(size_m / resolution))
        self.memory_s = memory_s
        self.recenter_m = recenter_m
        self.value = np.full((self.n, self.n), UNKNOWN, np.int8)
        self.stamp = np.full((self.n, self.n), -np.inf, np.float64)
        self.origin = None                       # (x, y) of cell (0, 0)'s corner, odom frame

    def _center_on(self, x, y):
        half = self.n * self.res / 2
        target = (math.floor((x - half) / self.res) * self.res, math.floor((y - half) / self.res) * self.res)
        if self.origin is None:
            self.origin = target
            return
        dc = int(round((target[0] - self.origin[0]) / self.res))
        dr = int(round((target[1] - self.origin[1]) / self.res))
        if dc == 0 and dr == 0:
            return
        self.value = _shift(self.value, dr, dc, UNKNOWN)
        self.stamp = _shift(self.stamp, dr, dc, -np.inf)
        self.origin = (self.origin[0] + dc * self.res, self.origin[1] + dr * self.res)

    def maybe_recenter(self, x, y):
        if self.origin is None:
            self._center_on(x, y)
            return
        half = self.n * self.res / 2
        cx, cy = self.origin[0] + half, self.origin[1] + half
        if math.hypot(x - cx, y - cy) > self.recenter_m:
            self._center_on(x, y)

    def integrate(self, view, sensor_pose, t):
        """Add a perception grid. ``view``: GridView in the sensor's ground
        frame (x forward); ``sensor_pose``: that frame's (x, y, yaw) in odom at
        capture time ``t``."""
        self.maybe_recenter(sensor_pose[0], sensor_pose[1])
        rows, cols = np.nonzero(view.data != UNKNOWN)
        if len(rows) == 0:
            return
        vals = view.data[rows, cols]
        # 2x2 sub-samples per cell so rotated cells don't leave holes.
        offs = np.array([0.25, 0.75]) * view.res
        gx = (view.origin_x + cols * view.res)[:, None, None] + offs[None, :, None]
        gy = (view.origin_y + rows * view.res)[:, None, None] + offs[None, None, :]
        gx, gy = np.broadcast_arrays(gx, gy)
        v = np.broadcast_to(vals[:, None, None], gx.shape).ravel()
        gx, gy = gx.ravel(), gy.ravel()
        c, s = math.cos(sensor_pose[2]), math.sin(sensor_pose[2])
        ox = sensor_pose[0] + c * gx - s * gy
        oy = sensor_pose[1] + s * gx + c * gy
        mc = np.floor((ox - self.origin[0]) / self.res).astype(np.int64)
        mr = np.floor((oy - self.origin[1]) / self.res).astype(np.int64)
        ok = (mr >= 0) & (mr < self.n) & (mc >= 0) & (mc < self.n)
        mr, mc, v = mr[ok], mc[ok], v[ok]
        free = v == FREE
        self.value[mr[free], mc[free]] = FREE           # free first ...
        self.value[mr[~free], mc[~free]] = LETHAL       # ... lethal wins within a frame
        self.stamp[mr, mc] = t

    def view(self, now):
        """GridView (odom frame) with stale cells reported as unknown."""
        data = self.value.copy()
        data[now - self.stamp > self.memory_s] = UNKNOWN
        return GridView(data, self.origin[0], self.origin[1], self.res)


def _shift(a, dr, dc, fill):
    """Move the window by (dr, dc) cells: out[r, c] = a[r + dr, c + dc]."""
    out = np.full_like(a, fill)
    n_r, n_c = a.shape
    if abs(dr) >= n_r or abs(dc) >= n_c:
        return out
    src_r = slice(max(dr, 0), n_r + min(dr, 0))
    dst_r = slice(max(-dr, 0), n_r + min(-dr, 0))
    src_c = slice(max(dc, 0), n_c + min(dc, 0))
    dst_c = slice(max(-dc, 0), n_c + min(-dc, 0))
    out[dst_r, dst_c] = a[src_r, src_c]
    return out
