import math

import numpy as np

from cart_driver import vehicle
from cart_driver.centre_line import CentreLine
from cart_driver.gridview import FREE, LETHAL, UNKNOWN, GridView
from cart_driver.planner import ArcPlanner
from cart_driver.road_model import RoadEstimator

P = vehicle.PROFILES['tesla_sim']


def camera_grid(centre_y=0.0, width=7.0, curvature=0.0, res=0.1):
    """A perception grid in camera_ground (x 0..12, y -8..8) of a road with
    0.3 m lethal curbs, seen from 2.7 m to 10 m ahead; unknown elsewhere."""
    xs = (np.arange(int(12 / res)) + 0.5) * res
    ys = -8 + (np.arange(int(16 / res)) + 0.5) * res
    data = np.full((len(ys), len(xs)), UNKNOWN, np.int8)
    for c, x in enumerate(xs):
        if not 2.7 <= x <= 10.0:
            continue
        mid = centre_y + 0.5 * curvature * x * x
        d = ys - mid
        data[np.abs(d) < width / 2, c] = FREE
        data[(np.abs(d) >= width / 2) & (np.abs(d) < width / 2 + 0.3), c] = LETHAL
    return data, 0.0, -8.0, res


def estimate(**kw):
    data, ox, oy, res = camera_grid(**kw)
    return RoadEstimator().estimate(data, res, ox, oy)


def test_estimator_finds_an_offset_centre():
    est = estimate(centre_y=1.0)
    assert est.ok
    assert abs(est.c0 - 1.0) < 0.1 and abs(est.slope) < 0.02
    assert abs(est.width - 7.0) < 0.3


def test_centre_line_is_placed_in_odom_at_the_camera_pose():
    est = estimate(centre_y=1.0)
    # Camera at (10, 5) facing +y: the camera's left (+y in camera_ground) is odom -x.
    line = CentreLine.from_estimate(est, (10.0, 5.0, math.pi / 2), stamp=3.0)
    assert np.allclose(line.points[:, 0], 9.0, atol=0.1)
    assert np.all(np.diff(line.points[:, 1]) > 0)
    assert np.allclose(line.headings, math.pi / 2, atol=0.02)
    # A point on the camera's axis is 1 m right of the line.
    assert abs(line.lateral_at([(10.0, 10.0)])[0] + 1.0) < 0.1
    # Seen from a camera 2 m further on, the line is still 1 m to its left.
    assert abs(line.seed((10.0, 7.0, math.pi / 2), 3.0) - 1.0) < 0.1


def test_projection_extends_past_both_ends():
    line = CentreLine(np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]), np.zeros(3), 0.0)
    along, lateral, heading = line.project(np.array([[-3.0, 0.5], [5.0, -2.0], [1.5, 1.0]]))
    assert np.allclose(along, [-3.0, 5.0, 1.5]) and np.allclose(lateral, [0.5, -2.0, 1.0])
    assert np.allclose(heading, 0.0)


def open_ground():
    data = np.full((200, 400), FREE, np.int8)                 # x 0..40, y -10..10, no edges
    return GridView(data, 0.0, -10.0, 0.1)


def straight_centre(y, x0=4.0, x1=12.0):
    xs = np.linspace(x0, x1, 33)
    return CentreLine(np.column_stack([xs, np.full_like(xs, y)]), np.zeros_like(xs), 0.0)


def test_planner_steers_towards_the_centre_line():
    pl = ArcPlanner(P)
    start = (0.0, 0.0, 0.0)
    straight = pl.plan(open_ground(), start, 0.0, P.cruise_ms, ref_heading=0.0)
    assert abs(straight.target_deg) < 1.0
    left = pl.plan(open_ground(), start, 0.0, P.cruise_ms, ref_heading=0.0, centre=straight_centre(1.5))
    right = pl.plan(open_ground(), start, 0.0, P.cruise_ms, ref_heading=0.0, centre=straight_centre(-1.5))
    assert left.target_deg < -2.0 and right.target_deg > 2.0             # + = right
    on = pl.plan(open_ground(), start, 0.0, P.cruise_ms, ref_heading=0.0, centre=straight_centre(0.0))
    assert abs(on.target_deg) < 1.0


def test_centre_line_does_not_override_obstacles():
    # The centre line runs straight through a box: the planner must still go round it.
    data = np.full((200, 400), FREE, np.int8)
    ys = -10 + (np.arange(200) + 0.5) * 0.1
    xs = (np.arange(400) + 0.5) * 0.1
    data[np.ix_((ys > -0.9) & (ys < 0.9), (xs > 14.0) & (xs < 14.8))] = LETHAL
    view = GridView(data, 0.0, -10.0, 0.1)
    plan = ArcPlanner(P).plan(view, (2.0, 0.0, 0.0), 0.0, P.cruise_ms, ref_heading=0.0,
                              centre=straight_centre(0.0, 6.0, 14.0))
    assert plan.valid and abs(plan.target_deg) > 2.0


def test_closed_loop_settles_on_the_centre_of_a_curving_road():
    """Drive the planner's choice for a few seconds on a road bending left,
    re-fitting the centre from a rendered grid each step."""
    pl = ArcPlanner(P)
    est_r = RoadEstimator()
    k = 1 / 40.0                                   # road curving left, 40 m radius, centre starts 1.2 m left
    def centre_y(x):
        return 1.2 + 0.5 * k * x * x
    pose, steer = np.array([0.0, 0.0, 0.0]), 0.0
    offsets = []
    for _ in range(180):                           # 18 s at 10 Hz, ~25 m
        cam = vehicle.offset_point(pose, P.camera_x)
        cyaw = pose[2]
        # Render the camera's grid from the world road (approximate: road offset at the camera).
        c, s = math.cos(cyaw), math.sin(cyaw)
        res = 0.1
        gx = (np.arange(120) + 0.5) * res
        gy = -8 + (np.arange(160) + 0.5) * res
        X, Y = np.meshgrid(gx, gy)
        wx, wy = cam[0] + c * X - s * Y, cam[1] + s * X + c * Y
        d = wy - centre_y(wx)
        grid = np.full(X.shape, UNKNOWN, np.int8)
        seen = (X >= 2.7) & (X <= 10.0)
        grid[seen & (np.abs(d) < 3.5)] = FREE
        grid[seen & (np.abs(d) >= 3.5) & (np.abs(d) < 3.8)] = LETHAL
        est = est_r.estimate(grid, res, 0.0, -8.0)
        assert est.ok
        line = CentreLine.from_estimate(est, (cam[0], cam[1], cyaw), 0.0)
        plan = pl.plan(open_ground_at(pose), pose, steer, P.cruise_ms, prev_target_deg=steer, centre=line)
        # Step the vehicle 0.1 s along the chosen path (14 steps of 0.15 m ~ 0.14 m at 5 km/h).
        pose = plan.poses[1]
        steer += float(np.clip(plan.target_deg - steer, -P.steer_rate_deg_s * 0.1, P.steer_rate_deg_s * 0.1))
        body = vehicle.offset_point(pose, P.wheelbase / 2)
        offsets.append(body[1] - centre_y(body[0]))
    assert abs(offsets[0]) > 0.9
    assert abs(np.mean(offsets[-30:])) < 0.25


def open_ground_at(pose):
    data = np.full((400, 400), FREE, np.int8)
    return GridView(data, pose[0] - 20.0, pose[1] - 20.0, 0.1)
