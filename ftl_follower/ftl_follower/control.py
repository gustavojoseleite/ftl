"""The two control laws of a follower, as plain functions."""

import math


def gap_speed(v_ahead, gap, desired, k_gap, v_max, min_gap):
    """Speed that holds `desired` metres behind the vehicle ahead.

    The vehicle ahead's own speed, plus a pull proportional to the gap error:
    too far back and it closes, too close and it eases off. Below `min_gap` it
    stops outright, whatever the vehicle ahead is doing.
    """
    if gap < min_gap:
        return 0.0
    return max(0.0, min(v_max, v_ahead + k_gap * (gap - desired)))


def lookahead(v, ld_min, ld_gain, ld_max):
    """Pure-pursuit lookahead distance: longer at speed, for a calmer wheel."""
    return max(ld_min, min(ld_max, ld_min + ld_gain * v))


def pure_pursuit_curvature(x, y, yaw, tx, ty):
    """Curvature of the arc from pose (x, y, yaw) through target (tx, ty)."""
    dx, dy = tx - x, ty - y
    xl = math.cos(yaw) * dx + math.sin(yaw) * dy
    yl = -math.sin(yaw) * dx + math.cos(yaw) * dy
    l2 = xl * xl + yl * yl
    if l2 < 1e-6:
        return 0.0
    return 2.0 * yl / l2


def clamp_curvature(kappa, min_radius):
    k = 1.0 / min_radius
    return max(-k, min(k, kappa))


def ramp(current, target, rate, dt):
    step = rate * dt
    return current + max(-step, min(step, target - current))
