import math

import numpy as np
import pytest

from cart_driver import control, vehicle
from cart_driver.control import Action
from cart_driver.gridview import LETHAL, UNKNOWN, GridView, collision_distance
from cart_driver.odometry import Odometry

P = vehicle.PROFILES['tesla_sim']


def test_steering_sign_convention():
    # + steering (right) -> negative curvature (clockwise)
    assert vehicle.steer_to_curvature(10, P.wheelbase) < 0
    assert vehicle.curvature_to_steer(vehicle.steer_to_curvature(10, P.wheelbase), P.wheelbase) == pytest.approx(10)


def test_step_pose_arc_and_straight():
    x, y, yaw = vehicle.step_pose((0, 0, 0), 1.0, 0.0, 2.0)
    assert (x, y, yaw) == pytest.approx((2, 0, 0))
    # quarter circle of radius 5 to the left
    x, y, yaw = vehicle.step_pose((0, 0, 0), 1.0, 0.2, math.pi / 2 * 5)
    assert (x, y, yaw) == pytest.approx((5, 5, math.pi / 2), abs=1e-9)


def test_odometry_uses_yaw_rate_and_history():
    odo = Odometry(P.wheelbase)
    for i in range(101):
        odo.update(i * 0.1, 1.0, 0.0, yaw_rate=0.1)          # 10 s at 1 m/s, 0.1 rad/s left
    assert odo.pose[2] == pytest.approx(1.0, abs=1e-6)
    assert odo.distance == pytest.approx(10.0)
    assert odo.pose_at(5.0)[2] == pytest.approx(0.5, abs=1e-6)
    # without IMU, steering drives the curvature (right = clockwise)
    odo2 = Odometry(P.wheelbase)
    for i in range(11):
        odo2.update(i * 0.1, 1.0, 10.0)
    assert odo2.pose[2] < 0


def test_speed_controller_never_brakes_and_trims_gain():
    pi = control.SpeedController(max_cmd=3.0)
    assert pi.update(0.0, 1.0, 0.1) == 0.0                    # target 0 = coast
    # plant that only reaches 80 % of the command: the loop must push the command up
    v = 0.0
    for _ in range(600):
        cmd = pi.update(1.39, v, 0.1)
        assert 0.0 <= cmd <= 3.0
        v += (0.8 * cmd - v) * 0.1
    assert v == pytest.approx(1.39, abs=0.03)
    assert pi.update(1.0, 3.0, 0.1) >= 0.0                    # overspeed: still no negative command


def test_safety_action():
    kw = dict(coast_decel=0.4)
    assert control.safety_action(0.1, 1.39, math.inf, **kw) is Action.DRIVE
    assert control.safety_action(0.9, 1.39, math.inf, **kw) is Action.COAST
    # coast stop from 5 km/h ~2.4 m + 0.42 + 0.5 -> brake inside ~3.3 m only
    assert control.safety_action(0.1, 1.39, 3.0, **kw) is Action.EMERGENCY_BRAKE
    assert control.safety_action(0.1, 1.39, 4.0, **kw) is Action.DRIVE
    assert control.safety_action(0.1, 0.0, 0.5, **kw) is Action.DRIVE      # standing still: no brake


def test_gridview_clearance_and_collision_distance():
    data = np.zeros((100, 100), np.int8)                      # 10 x 10 m at 0.1, origin (0, -5)
    data[45:55, 60:62] = LETHAL                               # obstacle at x 6.0-6.2, y -0.5..0.5
    view = GridView(data, 0.0, -5.0, 0.1)
    assert view.value_at(np.array([6.1, 20.0]), np.array([0.0, 0.0]))[1] == UNKNOWN
    assert view.clearance_at(np.array([5.0]), np.array([0.0]))[0] == pytest.approx(1.0, abs=0.1)
    poses = np.array([[s, 0.0, 0.0] for s in np.arange(0, 8, 0.1)])
    circles = [(0.0, 0.5)]
    assert collision_distance(view, poses, circles, 0.1) == pytest.approx(5.5, abs=0.15)
    poses_side = poses.copy()
    poses_side[:, 1] = 2.0
    assert collision_distance(view, poses_side, circles, 0.1) == math.inf


def test_footprint_covers_body():
    circles = P.footprint_circles()
    xs = [c[0] for c in circles]
    r = circles[0][1]
    assert min(xs) - r <= -P.rear_overhang + 1e-6 and max(xs) + r >= P.wheelbase + P.front_overhang - 1e-6


def test_clear_footprint_removes_self_artefacts_only():
    from cart_driver.gridview import clear_footprint
    data = np.zeros((100, 100), np.int8)
    data[48:52, 10:14] = LETHAL                               # under the car at x ~1.0-1.4
    data[48:52, 70:72] = LETHAL                               # real obstacle at x ~7
    view = clear_footprint(GridView(data, 0.0, -5.0, 0.1), (0.0, 0.0, 0.0), [(1.0, 0.9)])
    assert (view.data[48:52, 10:14] == 0).all()
    assert (view.data[48:52, 70:72] == LETHAL).all()
