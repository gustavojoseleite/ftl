import math

import pytest

from ftl_follower.control import (clamp_curvature, gap_speed, lookahead,
                                  pure_pursuit_curvature, ramp)
from ftl_follower.geo import LocalFrame, wrap, yaw_from_quaternion


def test_gap_speed_matches_the_vehicle_ahead_at_the_set_gap():
    assert gap_speed(3.0, 10.0, 10.0, 0.4, 4.0, 5.0) == pytest.approx(3.0)


def test_gap_speed_closes_and_eases():
    assert gap_speed(2.0, 12.0, 10.0, 0.4, 4.0, 5.0) == pytest.approx(2.8)
    assert gap_speed(2.0, 8.0, 10.0, 0.4, 4.0, 5.0) == pytest.approx(1.2)


def test_gap_speed_limits():
    assert gap_speed(3.0, 40.0, 10.0, 0.4, 4.0, 5.0) == 4.0     # never above v_max
    assert gap_speed(0.0, 7.0, 10.0, 0.4, 4.0, 5.0) == 0.0      # never backwards
    assert gap_speed(3.0, 4.9, 10.0, 0.4, 4.0, 5.0) == 0.0      # too close: stop


def test_pure_pursuit_straight_ahead_is_zero():
    assert pure_pursuit_curvature(0, 0, 0, 5, 0) == pytest.approx(0.0)


def test_pure_pursuit_sign_and_value():
    # Target 5 m ahead, 1 m to the left: k = 2y/L^2 = 2/26.
    assert pure_pursuit_curvature(0, 0, 0, 5, 1) == pytest.approx(2 / 26)
    assert pure_pursuit_curvature(0, 0, 0, 5, -1) == pytest.approx(-2 / 26)
    # Same geometry with the vehicle heading north.
    assert pure_pursuit_curvature(0, 0, math.pi / 2, -1, 5) == pytest.approx(2 / 26)


def test_clamp_curvature_to_the_turning_circle():
    assert clamp_curvature(1.0, 3.2) == pytest.approx(1 / 3.2)
    assert clamp_curvature(-1.0, 3.2) == pytest.approx(-1 / 3.2)
    assert clamp_curvature(0.1, 3.2) == 0.1


def test_lookahead_and_ramp():
    assert lookahead(0.0, 4.0, 1.0, 10.0) == 4.0
    assert lookahead(3.0, 4.0, 1.0, 10.0) == 7.0
    assert lookahead(30.0, 4.0, 1.0, 10.0) == 10.0
    assert ramp(0.0, 3.0, 1.0, 0.05) == pytest.approx(0.05)
    assert ramp(2.0, 2.01, 1.0, 0.05) == pytest.approx(2.01)


def test_local_frame_metres():
    f = LocalFrame(-20.9494, -48.4794)
    e, n = f.to_enu(-20.9494 + 0.001, -48.4794 + 0.001)
    assert n == pytest.approx(110.7, abs=0.5)     # a millidegree of latitude
    assert e == pytest.approx(104.0, abs=0.5)     # shorter: cos(21 degrees)


def test_yaw_and_wrap():
    assert yaw_from_quaternion(0, 0, math.sin(0.3), math.cos(0.3)) == pytest.approx(0.6)
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
