import math
import os
import sys

import numpy as np

from cart_driver import vehicle
from cart_driver.route import Route, RouteFollower, WorldAligner

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import make_route  # noqa: E402


def square_loop(offset=0.0):
    centre = make_route.resample(make_route.rounded_rect(0, 100, 0, 100, 20, 0.1), 0.5)
    return Route(make_route.resample(make_route.offset_left(centre, offset), 0.5))


def test_rounded_rect_is_anticlockwise_from_top_edge():
    route = square_loop()
    assert np.allclose(route.points[0], (50, 100), atol=0.3)
    assert abs(vehicle.wrap(route.heading[0] - math.pi)) < 0.05        # heading -x
    expected = 4 * 60 + 2 * math.pi * 20
    assert abs(route.length - expected) < 1.0


def test_keep_left_offset_goes_inside_an_anticlockwise_loop():
    route = square_loop(offset=8.0)
    # Top edge driven towards -x: left is -y.
    j, cross, _ = route.nearest(np.array([[50.0, 92.0]]))
    assert abs(cross[0]) < 0.3
    assert abs(route.length - (4 * 60 + 2 * math.pi * 12)) < 1.0


def test_signed_cross_track():
    route = square_loop()
    _, cross, _ = route.nearest(np.array([[50.0, 98.0], [50.0, 101.0]]))
    assert cross[0] > 1.9 and cross[1] < -0.9                       # +left (-y here), -right


def drive_pairs(aligner, theta, t, path):
    c, s = math.cos(theta), math.sin(theta)
    for x, y, yaw in path:
        aligner.add((x, y), (c * x - s * y + t[0], s * x + c * y + t[1]), yaw)


def test_aligner_recovers_rigid_transform():
    al = WorldAligner(initial_yaw=0.0)
    path = [(0.2 * k, 0.01 * k * k * 0.04, 0.0) for k in range(100)]
    drive_pairs(al, 2.0, (27.0, 47.0), path)
    assert abs(vehicle.wrap(al.theta - 2.0)) < 1e-6
    assert np.allclose(al.to_world((0.0, 0.0)), (27.0, 47.0), atol=1e-6)
    assert np.allclose(al.to_odom(al.to_world((3.0, -1.0))), (3.0, -1.0))


def test_aligner_uses_start_heading_before_moving():
    al = WorldAligner(initial_yaw=math.pi)
    al.add((3.79, 0.0), (27.66, 47.01), 0.0)
    assert al.ready and abs(vehicle.wrap(al.theta - math.pi)) < 1e-9
    # A point 10 m ahead in odom (+x) is 10 m towards -x in the world.
    assert np.allclose(al.to_world((13.79, 0.0)), (17.66, 47.01))


def test_follower_progress_wraps_and_counts_laps():
    route = square_loop()
    al = WorldAligner(initial_yaw=0.0)
    al.theta, al.t = 0.0, np.zeros(2)                                # odom == world
    f = RouteFollower(route, al, point_x=0.0)
    for k in range(0, len(route) * 2, 3):                            # two laps along the route
        p = route.points[k % len(route)]
        f.update((p[0], p[1], route.heading[k % len(route)]))
    assert f.laps == 1
    assert abs(f.distance - 2 * route.length) < 3 * route.spacing + 1


def test_follower_does_not_jump_across_the_loop():
    # Two parallel edges 6 m apart (a narrow loop): progress must stay on its edge.
    centre = make_route.resample(make_route.rounded_rect(0, 100, 0, 6, 3, 0.1), 0.5)
    route = Route(centre)
    al = WorldAligner(initial_yaw=0.0)
    al.theta, al.t = 0.0, np.zeros(2)
    f = RouteFollower(route, al, point_x=0.0)
    f.update((50.0, 6.0, math.pi))
    for x in np.arange(50.0, 20.0, -0.5):
        f.update((x, 2.6, math.pi))                                  # nearer the other edge now
        assert abs(route.points[f.progress][1] - 6.0) < 0.5


def test_cost_prefers_candidates_ending_on_the_route():
    route = square_loop()
    al = WorldAligner(initial_yaw=0.0)
    al.theta, al.t = 0.0, np.zeros(2)
    f = RouteFollower(route, al, point_x=0.0)
    f.update((60.0, 100.0, math.pi))
    ends = np.array([[50.0, 100.0, math.pi], [50.0, 97.0, math.pi], [50.0, 100.0, math.pi - 0.5]])
    cross, heading = f.cost(ends)
    assert cross[0] < 0.1 and abs(cross[1] - 3.0) < 0.1
    assert abs(heading[0]) < 0.05 and abs(heading[2] + 0.5) < 0.05
