"""Local planner: pick a steering target by rolling out candidate paths.

The cart steers with a relay that turns the wheels at a fixed ~12.9 deg/s
(up to +/-15 deg), so the planner cannot assume it gets any angle instantly.
Each candidate is "steer towards target T at the relay's rate, then hold",
simulated from the *current* wheel angle at cruise speed over ``horizon_m``
(10 m, so the front of the vehicle reaches the far edge of perception),
and scored on the memory map (odom frame):

  collision   any footprint circle touching a lethal cell -> invalid
              (distance to that collision is kept, to rank invalid ones)
  clearance   penalty for passing within ``soft_clearance`` of lethal cells
  unknown     share of the path over cells never (or not recently) seen
  route       distance from the route ahead (when a route is given)
  heading     without a route: end heading vs. a slowly updated reference
              heading, so the vehicle returns to its course after swerving
  smoothness  change from the previous target (relay chatter wears it)
  steering    small preference for straight

Re-planned every tick, so "steer then hold" is only ever executed for a
fraction of a second: the receding horizon turns these simple arcs into
smooth, feedback-corrected paths.

Signs: targets are in the stack's convention (deg, + = RIGHT); internally
the rollout uses curvature with + = LEFT (vehicle.steer_to_curvature).
"""

import math
from dataclasses import dataclass, field

import numpy as np

from cart_driver import vehicle
from cart_driver.gridview import UNKNOWN, footprint_points


@dataclass
class PlannerWeights:
    clearance: float = 4.0
    unknown: float = 2.0
    route: float = 1.5
    route_heading: float = 2.0
    heading: float = 3.0          # per rad, route-less heading keeping
    smoothness: float = 0.02      # per degree of change
    steering: float = 0.005       # per degree away from straight


@dataclass
class Plan:
    target_deg: float             # stack convention, + = right
    poses: np.ndarray             # (S, 3) rear-axle poses of the chosen candidate
    valid: bool                   # a collision-free candidate existed
    collision_dist: float         # m to collision along the chosen path (inf = none)
    costs: np.ndarray = field(default=None)   # per-candidate cost (inf = invalid)
    targets: np.ndarray = field(default=None)


class ArcPlanner:
    def __init__(self, params, horizon_m=10.0, n_candidates=21, step_m=0.15, hard_margin=0.15,
                 soft_clearance=1.0, weights=None):
        self.p = params
        self.horizon_m = horizon_m
        self.step_m = step_m
        self.targets = np.linspace(-params.max_steer_deg, params.max_steer_deg, n_candidates)
        self.circles = params.footprint_circles(hard_margin)
        self.radii = np.array([c[1] for c in self.circles])
        self.soft_clearance = soft_clearance
        self.w = weights or PlannerWeights()

    def rollout(self, start, steer_now_deg, speed):
        """(N, S, 3) rear-axle poses for every candidate target."""
        speed = max(speed, self.p.cruise_ms * 0.5)
        n_steps = int(math.ceil(self.horizon_m / self.step_m))
        dt = self.step_m / speed
        max_step = self.p.steer_rate_deg_s * dt
        out = np.empty((len(self.targets), n_steps + 1, 3))
        for i, target in enumerate(self.targets):
            pose, steer = start, steer_now_deg
            out[i, 0] = pose
            for k in range(1, n_steps + 1):
                steer += float(np.clip(target - steer, -max_step, max_step))
                kappa = vehicle.steer_to_curvature(steer, self.p.wheelbase)
                pose = vehicle.step_pose(pose, speed, kappa, dt)
                out[i, k] = pose
        return out

    def plan(self, view, start, steer_now_deg, speed, prev_target_deg=0.0, route=None, ref_heading=None):
        """Choose a steering target. ``view``: GridView in the same frame as
        ``start`` (rear-axle pose). ``route``: optional object with
        ``cost(poses) -> (cross_track m, heading error rad)`` per candidate end.
        ``ref_heading``: without a route, the course to return to (odom rad)."""
        poses = self.rollout(start, steer_now_deg, speed)
        n, s, _ = poses.shape
        pts = footprint_points(poses.reshape(-1, 3), self.circles).reshape(n, s, len(self.circles), 2)
        clear = view.clearance_at(pts[..., 0], pts[..., 1])                  # (N, S, C)
        margin = clear - self.radii[None, None, :]

        hit = (margin < 0).any(axis=2)                                        # (N, S)
        first_hit = np.where(hit.any(axis=1), hit.argmax(axis=1), s)
        collision_dist = np.where(first_hit < s, first_hit * self.step_m, np.inf)
        valid = first_hit == s

        # Clearance: how far inside the soft band the tightest circle gets, per step.
        cost = self.w.clearance * np.mean(np.clip(self.soft_clearance - margin.min(axis=2), 0, None) ** 2, axis=1)
        mid = pts[:, :, len(self.circles) // 2, :]                            # body centre
        unknown = view.value_at(mid[..., 0], mid[..., 1]) == UNKNOWN
        cost += self.w.unknown * unknown.mean(axis=1)
        if route is not None:
            cross, heading = route.cost(poses[:, -1, :])
            cost += self.w.route * np.asarray(cross) + self.w.route_heading * np.abs(np.asarray(heading))
        elif ref_heading is not None:
            err = np.arctan2(np.sin(poses[:, -1, 2] - ref_heading), np.cos(poses[:, -1, 2] - ref_heading))
            cost += self.w.heading * np.abs(err)
        cost += self.w.smoothness * np.abs(self.targets - prev_target_deg)
        cost += self.w.steering * np.abs(self.targets)

        costs = np.where(valid, cost, np.inf)
        if valid.any():
            i = int(np.argmin(costs))
        else:
            i = int(np.argmax(collision_dist))       # nothing is clear: the path that hits last
        return Plan(float(self.targets[i]), poses[i], bool(valid.any()), float(collision_dist[i]),
                    costs=costs, targets=self.targets)
