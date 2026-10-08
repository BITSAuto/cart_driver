import math

import numpy as np

from cart_driver import vehicle
from cart_driver.gridview import FREE, LETHAL, GridView
from cart_driver.planner import ArcPlanner

P = vehicle.PROFILES['tesla_sim']


def road(width=8.0, length=40.0, obstacles=()):
    """Free straight road along +x centred on y=0, lethal outside; obstacles as (x0, x1, y0, y1)."""
    res = 0.1
    data = np.full((int(20 / res), int(length / res)), LETHAL, np.int8)       # y -10..10
    ys = -10 + (np.arange(data.shape[0]) + 0.5) * res
    data[np.abs(ys) < width / 2, :] = FREE
    xs = (np.arange(data.shape[1]) + 0.5) * res
    for x0, x1, y0, y1 in obstacles:
        data[np.ix_((ys > y0) & (ys < y1), (xs > x0) & (xs < x1))] = LETHAL
    return GridView(data, 0.0, -10.0, res)


def test_rollout_respects_relay_rate_and_sign():
    pl = ArcPlanner(P)
    poses = pl.rollout((0, 0, 0), 0.0, P.cruise_ms)
    right = poses[-1]                       # target +15 deg = right turn
    left = poses[0]
    assert right[-1, 1] < -0.5 and left[-1, 1] > 0.5        # right ends at negative y
    # the wheels need ~1.2 s to reach full lock, so after 0.15 m the path is still nearly straight
    assert abs(right[1, 2]) < 0.01


def test_goes_straight_on_a_clear_road():
    plan = ArcPlanner(P).plan(road(), (2.0, 0.0, 0.0), 0.0, P.cruise_ms)
    assert plan.valid and abs(plan.target_deg) < 2.0


def test_swerves_around_an_obstacle_in_the_lane():
    # Box 8.2 m ahead of the front bumper (bumper at x 5.79), slightly right of centre. Closer than
    # ~7 m the Tesla can't clear it: +/-15 deg gives an 11 m radius and the relay needs 1.2 s to lock.
    view = road(width=12.0, obstacles=[(14.0, 14.8, -1.2, 0.6)])
    plan = ArcPlanner(P).plan(view, (2.0, 0.0, 0.0), 0.0, P.cruise_ms)
    assert plan.valid
    assert plan.target_deg < -2.0                                          # steers left (negative = left)


def test_follows_the_road_when_it_bends():
    # road turning right: free band bends from y=0 to y=-6 over x 10..20
    res = 0.1
    data = np.full((200, 400), LETHAL, np.int8)
    xs = (np.arange(400) + 0.5) * res
    ys = -10 + (np.arange(200) + 0.5) * res
    centre = np.where(xs < 8, 0.0, np.clip(-(xs - 8) * 0.6, -6, 0))
    data[np.abs(ys[:, None] - centre[None, :]) < 3.5] = FREE
    plan = ArcPlanner(P).plan(GridView(data, 0.0, -10.0, res), (2.0, 0.0, 0.0), 0.0, P.cruise_ms)
    assert plan.valid and plan.target_deg > 2.0                            # turns right


def test_blocked_road_is_reported_invalid():
    view = road(width=4.0, obstacles=[(8.0, 9.0, -3.0, 3.0)])
    plan = ArcPlanner(P).plan(view, (2.0, 0.0, 0.0), 0.0, P.cruise_ms)
    assert not plan.valid and math.isfinite(plan.collision_dist)


def test_heading_keeping_steers_back_to_course():
    # On an open road, heading 0.3 rad off a reference course of 0: steer back (towards negative yaw = right).
    plan = ArcPlanner(P).plan(road(width=30.0), (2.0, 0.0, 0.3), 0.0, P.cruise_ms, ref_heading=0.0)
    assert plan.target_deg > 2.0
