import math

import pytest

from ftl_follower.trail import Trail


def straight(n=50, step=1.0):
    t = Trail(spacing=0.3)
    for i in range(n):
        t.append(i * step, 0.0)
    return t


def test_points_closer_than_spacing_are_skipped():
    t = Trail(spacing=0.5)
    for x in (0.0, 0.1, 0.2, 0.6, 0.7, 1.2):
        t.append(x, 0.0)
    assert t.xs == [0.0, 0.6, 1.2]
    assert t.end_s == pytest.approx(1.2)


def test_project_gives_arc_length_and_signed_offset():
    t = straight()
    s, lateral, _ = t.project(10.5, 2.0)
    assert s == pytest.approx(10.5)
    assert lateral == pytest.approx(2.0)          # left of a trail heading +x
    _, lateral, _ = t.project(10.5, -1.0)
    assert lateral == pytest.approx(-1.0)


def test_gap_is_measured_along_a_bend_not_straight_across():
    # A 90 degree corner: 10 m east, then 10 m north.
    t = Trail(spacing=0.3)
    t.seed_line((0.0, 0.0), (10.0, 0.0))
    t.seed_line((10.0, 0.0), (10.0, 10.0))
    s, _, _ = t.project(0.0, 0.0)
    assert t.end_s - s == pytest.approx(20.0, abs=0.35)   # 14.1 in a straight line


def test_point_at_interpolates_and_clamps():
    t = straight(11)
    assert t.point_at(4.25) == pytest.approx((4.25, 0.0))
    assert t.point_at(-3.0) == (0.0, 0.0)
    assert t.point_at(99.0) == (10.0, 0.0)


def test_project_follows_the_hint_on_a_trail_that_crosses_itself():
    # Out along y=0 and back along y=0.2: two passes over the same ground.
    t = Trail(spacing=0.3)
    t.seed_line((0.0, 0.0), (20.0, 0.0))
    t.seed_line((20.0, 0.0), (20.0, 0.2))
    t.seed_line((20.0, 0.2), (0.0, 0.2))
    _, _, i_out = t.project(5.0, 0.1, hint=0, window=30)
    s_back, _, _ = t.project(5.0, 0.1, hint=len(t) - 60, window=60)
    assert t.ss[i_out] < 20.0
    assert s_back > 20.0


def test_prune_keeps_arc_length_absolute():
    t = straight(50)
    removed = t.prune(20.0)
    assert removed > 0
    assert t.ss[0] <= 20.0 < t.ss[1] + 1.0
    s, _, _ = t.project(30.0, 0.0)
    assert s == pytest.approx(30.0)
    assert len(t) >= 2


def test_seed_line_spacing():
    t = Trail(spacing=0.5)
    t.seed_line((0.0, 0.0), (0.0, 10.0))
    assert t.end_s == pytest.approx(10.0)
    gaps = [b - a for a, b in zip(t.ss, t.ss[1:])]
    assert all(g == pytest.approx(0.5) for g in gaps)
    assert math.isclose(t.ys[-1], 10.0)
