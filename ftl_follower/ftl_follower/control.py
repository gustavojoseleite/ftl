"""As duas leis de controle de um seguidor, como funções simples."""

import math


def gap_speed(v_ahead, gap, desired, k_gap, v_max, min_gap):
    """
    Velocidade que mantém `desired` metros atrás do veículo da frente.

    É a velocidade do próprio veículo da frente, mais uma correção proporcional ao erro
    do espaçamento: longe demais, ele se aproxima; perto demais, alivia. Abaixo de
    `min_gap` ele para de vez, não importa o que o veículo da frente faça.
    """
    if gap < min_gap:
        return 0.0
    return max(0.0, min(v_max, v_ahead + k_gap * (gap - desired)))


def lookahead(v, ld_min, ld_gain, ld_max):
    """Distância de visada do `pure pursuit`: maior em velocidade, para um volante mais calmo."""
    return max(ld_min, min(ld_max, ld_min + ld_gain * v))


def pure_pursuit_curvature(x, y, yaw, tx, ty):
    """Curvatura do arco da pose (x, y, yaw) até o alvo (tx, ty)."""
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
