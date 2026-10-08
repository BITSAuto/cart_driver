import math

import numpy as np
import pytest

from cart_driver.gridview import FREE, LETHAL, UNKNOWN, GridView
from cart_driver.local_map import LocalMap
from cart_driver.timesync import ClockMap


def perception(obstacle_x=None):
    """10 x 10 m sensor grid (x 0..10 forward, y -5..5), all free, optional obstacle."""
    data = np.zeros((100, 100), np.int8)
    if obstacle_x is not None:
        c = int(obstacle_x / 0.1)
        data[48:52, c:c + 3] = LETHAL
    return GridView(data, 0.0, -5.0, 0.1)


def test_obstacle_is_remembered_after_it_leaves_view_then_expires():
    m = LocalMap(size_m=40, memory_s=5.0)
    m.integrate(perception(obstacle_x=5.0), (0.0, 0.0, 0.0), t=0.0)
    v = m.view(1.0)
    assert v.value_at(np.array([5.1]), np.array([0.0]))[0] == LETHAL
    # Turn 90 deg left: the obstacle is no longer in the sensor's view, but remembered.
    m.integrate(perception(), (0.0, 0.0, math.pi / 2), t=2.0)
    assert m.view(3.0).value_at(np.array([5.1]), np.array([0.0]))[0] == LETHAL
    assert m.view(6.0).value_at(np.array([5.1]), np.array([0.0]))[0] == UNKNOWN


def test_rotated_frame_lands_in_right_place_and_newer_free_clears():
    m = LocalMap(size_m=40)
    m.integrate(perception(obstacle_x=5.0), (2.0, 1.0, math.pi / 2), t=0.0)   # facing +y
    assert m.view(0.5).value_at(np.array([2.0]), np.array([6.1]))[0] == LETHAL
    m.integrate(perception(), (2.0, 1.0, math.pi / 2), t=1.0)
    assert m.view(1.5).value_at(np.array([2.0]), np.array([6.1]))[0] == FREE


def test_recentring_keeps_world_positions():
    m = LocalMap(size_m=20, recenter_m=3.0)
    m.integrate(perception(obstacle_x=5.0), (0.0, 0.0, 0.0), t=0.0)
    m.integrate(GridView(np.full((10, 10), UNKNOWN, np.int8), 0, 0, 0.1), (4.0, 0.0, 0.0), t=0.1)  # moves > 3 m
    assert m.view(0.2).value_at(np.array([5.1]), np.array([0.0]))[0] == LETHAL


def test_clock_map():
    c = ClockMap()
    for w in np.arange(0, 3, 0.1):
        c.observe(1000.0 + w, 50.0 + 0.5 * w)                 # sim runs at half speed
    assert c.rate == pytest.approx(0.5)
    assert c.to_sim(1002.0) == pytest.approx(51.0, abs=1e-6)
    assert ClockMap(identity=True).to_sim(123.0) == 123.0
