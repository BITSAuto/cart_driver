# Vendored from road_follower/road_follower/road_model.py (BITSAuto_All_repo on
# the Orin, 2026-10-10), unchanged apart from this header. road_follower is not
# in a git repository yet; when it is, depend on it instead of this copy.
# See project-docs/03-decisions/D-020-road-centre-cost-from-road-follower.md.

"""Road centre-line estimation from the traversability occupancy grid.

Pure numpy, no ROS, so it can be unit-tested and run closed-loop offline.

Input is the grid ``traversability`` publishes: row = y index, column = x
index, in the ``camera_ground`` frame (X forward, Y left, Z up), cell values
0 free, 100 lethal, -1 unknown.

What the estimator does, one frame at a time:

1. At a series of forward distances, take the free span the car is standing
   in. The first span is seeded at the car's own lateral position (the car is
   on the road); each later span is seeded at the centre of the previous one,
   so the estimate follows the road round a bend and cannot jump to a free
   area on the far side of a curb or fence.
2. Each end of a span is a *hard* edge if it stops at a lethal cell (a curb, a
   barrier, a wall) and a *soft* edge if it stops at unknown cells (the end of
   the camera's view, or missing depth). A soft edge says nothing about the
   road, so it never contributes to the centre.
3. The centre of a span with two hard edges is their midpoint. With only one
   hard edge, the centre is that edge offset by half the road width seen
   recently. With none, the sample is dropped.
4. A robust quadratic ``y = c0 + s x + k x^2 / 2`` is fitted through the
   centres: ``c0`` is the lateral offset of the road centre at the camera,
   ``atan(s)`` the road direction relative to the car, ``k`` its curvature.
"""

import math
from dataclasses import dataclass, field

import numpy as np

FREE, LETHAL, UNKNOWN = 0, 100, -1


@dataclass
class Params:
    x_near: float = 3.0           # m, first sample distance (perception sees from ~2.7 m)
    x_far: float = 9.5            # m, last sample distance (perception range is 10 m)
    x_step: float = 0.5           # m
    band: float = 0.3             # m, forward thickness of the band averaged per sample
    min_lethal_run: float = 0.3   # m, lethal runs shorter than this are depth noise
    max_gap: float = 0.5          # m, unknown runs shorter than this inside a span are bridged
    seed_search: float = 2.5      # m, how far from the seed a free cell is accepted
    min_width: float = 2.0        # m, narrower spans are not a road the cart fits in
    max_width: float = 14.0       # m, wider "spans" are open ground, not a bounded road
    nominal_width: float = 0.0    # m, road width to assume before any is observed (0 = none)
    width_alpha: float = 0.2      # smoothing of the observed road width
    min_points: int = 4           # samples needed for a fit
    min_span: float = 2.5         # m, forward extent the samples must cover
    max_curvature: float = 0.15   # 1/m, fit constraint (radius > 6.7 m)
    huber: float = 0.3            # m, robust-fit scale
    curvature_ridge: float = 5.0  # penalty on curvature: noise must not bend the line when extrapolated to x=0
    one_sided_weight: float = 0.5


@dataclass
class RoadEstimate:
    ok: bool = False
    c0: float = 0.0               # m, road-centre lateral offset at x = 0 (+left)
    slope: float = 0.0            # dy/dx at x = 0
    curvature: float = 0.0        # 1/m, + turns left
    x_min: float = 0.0            # m, forward extent the fit is supported on
    x_max: float = 0.0
    width: float = 0.0            # m, observed road width (0 if never both-edged)
    n_points: int = 0
    n_both: int = 0
    n_one: int = 0
    residual: float = 0.0         # m, rms fit residual
    reason: str = ''
    samples: list = field(default_factory=list)   # (x, centre, kind) for debugging/visualising

    def centre_at(self, x):
        return self.c0 + self.slope * x + 0.5 * self.curvature * x * x

    @property
    def heading_error(self):
        """Road direction relative to the car, rad (+ the road bears left)."""
        return math.atan(self.slope)


def _despeckle(profile, min_run):
    """Remove lethal runs shorter than ``min_run`` cells: depth noise, not a curb.

    Between free cells the run becomes free; next to unknown it becomes unknown
    (we cannot claim the ground there is free)."""
    out = profile.copy()
    n = len(out)
    i = 0
    while i < n:
        if out[i] == LETHAL:
            j = i
            while j < n and out[j] == LETHAL:
                j += 1
            if (j - i) < min_run:
                before = out[i - 1] if i > 0 else UNKNOWN
                after = out[j] if j < n else UNKNOWN
                out[i:j] = FREE if (before == FREE and after == FREE) else UNKNOWN
            i = j
        else:
            i += 1
    return out


def _span(profile, seed, res, max_gap_cells, search_cells):
    """Free span around ``seed`` (a row index): (lo, hi, lo_hard, hi_hard) or None.

    ``lo``/``hi`` are the first and last free row of the span.
    """
    n = len(profile)
    seed = int(np.clip(seed, 0, n - 1))
    start = None
    for d in range(search_cells + 1):
        for cand in (seed + d, seed - d):
            if 0 <= cand < n and profile[cand] == FREE:
                start = cand
                break
        if start is not None:
            break
    if start is None:
        return None

    def walk(step):
        i = start
        last = start
        while True:
            j = i + step
            if j < 0 or j >= n:
                return last, False                     # grid boundary: soft
            v = profile[j]
            if v == FREE:
                last = i = j
            elif v == LETHAL:
                return last, True                      # hard edge
            else:                                      # unknown: bridge a short gap
                k = j
                while 0 <= k < n and profile[k] == UNKNOWN:
                    k += step
                if 0 <= k < n and profile[k] == FREE and abs(k - j) <= max_gap_cells:
                    last = i = k
                else:
                    return last, False                 # soft edge
    hi, hi_hard = walk(+1)
    lo, lo_hard = walk(-1)
    return lo, hi, lo_hard, hi_hard


class RoadEstimator:
    """Stateful estimator; keeps the observed road width between frames."""

    def _side_of_obstacle(self, grid, c, row, res, x0, y0, band_cells, min_run):
        """The car's axis runs into a lethal strip (a curb it is heading for).
        Free ground lies on both sides of it, but only one side is where the
        car is. The car sits at x = 0, y = 0, so extend the strip back to
        x = 0 and see which side of it the origin lies on. Returns the row of
        a free cell on that side, or None if the origin is inside the strip.
        """
        h, w = grid.shape

        def run_at(col):
            colv = (grid[:, max(col - band_cells, 0):col + band_cells + 1] == LETHAL).any(axis=1)
            if not colv[row] and not colv[max(row - 5, 0):row + 6].any():
                return None
            r = int(np.argmax(colv[max(row - 5, 0):row + 6])) + max(row - 5, 0) if not colv[row] else row
            a = b = r
            while a - 1 >= 0 and colv[a - 1]:
                a -= 1
            while b + 1 < h and colv[b + 1]:
                b += 1
            return a, b

        step = int(round(1.0 / res))
        r1 = run_at(c)
        r2 = run_at(min(c + step, w - 1 - band_cells))
        if r1 is None:
            return None
        x1 = x0 + (c + 0.5) * res
        if r2 is None or (r2[1] - r2[0]) > 3 * (r1[1] - r1[0]) + 10:
            slope = 0.0
        else:
            slope = ((r2[0] + r2[1]) / 2 - (r1[0] + r1[1]) / 2) * res / (step * res)
        lo0 = y0 + r1[0] * res - slope * x1
        hi0 = y0 + (r1[1] + 1) * res - slope * x1
        if 0.0 < lo0 - 0.1:
            return r1[0] - 1            # the car is below the strip
        if 0.0 > hi0 + 0.1:
            return r1[1] + 1            # the car is above it
        return None

    def __init__(self, params=None):
        self.p = params or Params()
        self.width = self.p.nominal_width if self.p.nominal_width > 0 else None

    def estimate(self, grid, res, x0, y0, seed_y=None):
        """``grid`` is (H, W): row = y index from ``y0``, column = x index from ``x0``.

        ``seed_y`` is where the road centre is expected at the nearest sample
        (the previous estimate, dead-reckoned). Without one the car's own axis
        (y = 0) is used -- and if that lands on an obstacle there is no way to
        tell which side of it the road is on, so no estimate is made.
        """
        p = self.p
        h, w = grid.shape
        xs = np.arange(p.x_near, p.x_far + 1e-6, p.x_step)
        band_cells = max(1, int(round(p.band / res / 2)))
        min_run = max(1, int(round(p.min_lethal_run / res)))
        gap_cells = int(round(p.max_gap / res))
        search = int(round(p.seed_search / res))

        raw = []   # (x, lo_y, hi_y, lo_hard, hi_hard)
        have_prior = seed_y is not None
        seed = seed_y if have_prior else 0.0
        first = True
        for x in xs:
            c = int(round((x - x0) / res))
            if c - band_cells < 0 or c + band_cells >= w:
                continue
            block = grid[:, c - band_cells:c + band_cells + 1]
            prof = np.full(h, UNKNOWN, dtype=np.int8)
            prof[(block == FREE).any(axis=1)] = FREE
            prof[(block == LETHAL).any(axis=1)] = LETHAL
            prof = _despeckle(prof, min_run)
            if first and not have_prior:
                first = False
                row = int(round((seed - y0) / res - 0.5))
                if 0 <= row < h and prof[row] == LETHAL:
                    row = self._side_of_obstacle(grid, c, row, res, x0, y0, band_cells, min_run)
                    if row is None:
                        est = RoadEstimate(width=self.width or 0.0)
                        est.reason = 'ambiguous: the car is on a lethal strip ahead'
                        return est
                    seed = y0 + (row + 0.5) * res
            sp = _span(prof, int(round((seed - y0) / res - 0.5)), res, gap_cells, search)
            if sp is None:
                continue
            lo, hi, lo_hard, hi_hard = sp
            y_lo = y0 + lo * res            # lower edge of the first free cell
            y_hi = y0 + (hi + 1) * res      # upper edge of the last free cell
            raw.append((float(x), y_lo, y_hi, lo_hard, hi_hard))
            if lo_hard and hi_hard:
                seed = 0.5 * (y_lo + y_hi)
            elif hi_hard and self.width:
                seed = y_hi - self.width / 2
            elif lo_hard and self.width:
                seed = y_lo + self.width / 2
            else:
                seed = 0.5 * (y_lo + y_hi)

        # Update the observed width from both-edged samples (median, so one bad row is harmless).
        widths = [hi - lo for _, lo, hi, lh, hh in raw
                  if lh and hh and p.min_width <= hi - lo <= p.max_width]
        if widths:
            m = float(np.median(widths))
            self.width = m if self.width is None else (1 - p.width_alpha) * self.width + p.width_alpha * m

        pts = []   # (x, centre, weight, kind)
        for x, lo, hi, lh, hh in raw:
            wid = hi - lo
            if lh and hh:
                if p.min_width <= wid <= p.max_width:
                    pts.append((x, 0.5 * (lo + hi), 1.0, 'both'))
            elif hh and self.width:
                pts.append((x, hi - self.width / 2, p.one_sided_weight, 'left'))
            elif lh and self.width:
                pts.append((x, lo + self.width / 2, p.one_sided_weight, 'right'))

        est = RoadEstimate(width=self.width or 0.0)
        est.samples = [(x, c, k) for x, c, _, k in pts]
        est.n_both = sum(1 for q in pts if q[3] == 'both')
        est.n_one = len(pts) - est.n_both
        est.n_points = len(pts)
        if len(pts) < p.min_points:
            est.reason = f'only {len(pts)} usable samples'
            return est
        px = np.array([q[0] for q in pts])
        if px.max() - px.min() < p.min_span:
            est.reason = 'samples cover too little road'
            return est
        py = np.array([q[1] for q in pts])
        pw = np.array([q[2] for q in pts])

        # Robust (IRLS / Huber) quadratic, curvature-bounded.
        a = np.stack([np.ones_like(px), px, 0.5 * px * px], axis=1)
        wt = pw.copy()
        coef = np.zeros(3)
        for _ in range(4):
            sw = np.sqrt(wt)
            aa = np.vstack([a * sw[:, None], [0.0, 0.0, p.curvature_ridge]])
            bb = np.concatenate([py * sw, [0.0]])
            coef, *_ = np.linalg.lstsq(aa, bb, rcond=None)
            coef[2] = float(np.clip(coef[2], -p.max_curvature, p.max_curvature))
            r = py - a @ coef
            wt = pw * np.where(np.abs(r) <= p.huber, 1.0, p.huber / np.maximum(np.abs(r), 1e-9))
        r = py - a @ coef
        est.ok = True
        est.c0, est.slope, est.curvature = float(coef[0]), float(coef[1]), float(coef[2])
        est.x_min, est.x_max = float(px.min()), float(px.max())
        est.residual = float(np.sqrt(np.average(r * r, weights=pw)))
        return est
