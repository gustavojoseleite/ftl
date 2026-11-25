"""Testes do trilho compartilhado e do comando do seguidor do trilho único."""
import math

from ftl_trajectory.path import SharedPath
from ftl_trajectory.trail_follower import command, TrailFollower
import pytest

PARAMS = dict(TrailFollower.DEFAULTS)


def _line(n=50):
    return SharedPath([(float(k), 0.0) for k in range(n)])


def test_projection_inside_and_beyond_the_ends():
    path = _line()
    assert path.project(10.5, 0.3) == pytest.approx((10.5, 0.3))
    assert path.project(10.5, -0.3)[1] == pytest.approx(-0.3)
    assert path.project(-8.0, 0.0)[0] == pytest.approx(-8.0)  # antes do começo
    assert path.project(60.0, 0.0)[0] == pytest.approx(60.0)  # depois do fim
    assert path.point_at(12.25) == pytest.approx((12.25, 0.0))
    assert path.point_at(-3.0) == (0.0, 0.0)
    assert path.point_at(99.0) == (49.0, 0.0)


def test_projection_stays_near_the_hint():
    # Um trilho em U: a ida e a volta passam a 2 m uma da outra.
    out = [(float(k), 0.0) for k in range(30)]
    back = [(29.0 - k, 2.0) for k in range(30)]
    path = SharedPath(out + back)
    s_near_start, _ = path.project(5.0, 0.8, hint=5.0, window=10.0)
    assert s_near_start == pytest.approx(5.0)
    s_back, _ = path.project(5.0, 1.2, hint=path.length - 5.0, window=10.0)
    assert s_back > 40.0


def test_repeated_points_are_dropped():
    path = SharedPath([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)])
    assert len(path) == 2
    assert path.length == pytest.approx(1.0)


def test_command_keeps_the_gap_and_steers_to_the_trail():
    path = _line(100)
    hints = {}
    # Atrás do veículo da frente a 10 m, no trilho: só copia a velocidade dele, sem esterçar.
    v, omega, gap, lateral = command(path, (20.0, 0.0, 0.0, 2.0), (30.0, 0.0, 0.0, 2.0), hints,
                                     PARAMS, 2.0, 0.05)
    assert gap == pytest.approx(10.0)
    assert v == pytest.approx(2.0)
    assert omega == pytest.approx(0.0)
    assert hints == {'me': pytest.approx(20.0), 'ahead': pytest.approx(30.0)}
    # Um metro à esquerda do trilho: esterça para a direita.
    _, omega, _, lateral = command(path, (20.0, 1.0, 0.0, 2.0), (30.0, 0.0, 0.0, 2.0), {},
                                   PARAMS, 2.0, 0.05)
    assert lateral == pytest.approx(1.0)
    assert omega < 0.0
    # Perto demais do veículo da frente: para.
    v, *_ = command(path, (20.0, 0.0, 0.0, 2.0), (24.0, 0.0, 0.0, 2.0), {}, PARAMS, 2.0, 0.05)
    assert v == 0.0


def test_command_before_the_trail_starts():
    # O trator ainda atrás do começo do trilho mede a distância pela extensão do primeiro trecho.
    path = _line(3)
    v, _, gap, _ = command(path, (-10.0, 0.0, 0.0, 0.0), (2.5, 0.0, 0.0, 1.0), {}, PARAMS, 0.0,
                           0.05)
    assert gap == pytest.approx(12.5)
    assert v > 0.0
    assert math.isfinite(v)


def test_command_does_not_wind_up_while_the_vehicle_is_stuck():
    # Parado (controlador de tração ainda fora) e muito atrás: o comando não passa de
    # max_lead acima da velocidade medida, por mais ciclos que passem.
    path = _line(200)
    v = 0.0
    for _ in range(400):
        v, *_ = command(path, (0.0, 0.0, 0.0, 0.0), (90.0, 0.0, 0.0, 3.0), {}, PARAMS, v, 0.05)
    assert v == pytest.approx(PARAMS['max_lead'])
    # Andando, o comando sobe na rampa normal a partir do anterior.
    v2, *_ = command(path, (0.0, 0.0, 0.0, 3.0), (90.0, 0.0, 0.0, 3.0), {}, PARAMS, 3.0, 0.05)
    assert v2 == pytest.approx(3.0 + PARAMS['accel'] * 0.05)
