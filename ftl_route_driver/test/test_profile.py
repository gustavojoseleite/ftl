"""
Testes do perfil de velocidade e do protocolo (F2.1, R2 e R3), sem ROS.

- `profile.speed_profile(route, cruise, a_lat=1.0, decel=1.0, step=0.5)` devolve
  `(s, v)`: `v = min(cruzeiro, √(a_lat/|κ|))`, antecipado para trás com a desaceleração `decel`,
  para a velocidade nunca exigir frear mais que isso antes de uma curva.
- `profile.speed_at(perfil, s)` interpola o perfil.
- `protocol.spacing(gap=4.0)` é a distância entre `base_link` que deixa `gap` metros entre
  para-choques (balanços do URDF: trator −1,15/+2,81 m; pickup −1,12/+4,10 m).
- `protocol.finished(route, last_s)` diz se o último trator já passou o fim da última curva.
"""
import math

from ftl_route_driver.profile import speed_at, speed_profile
from ftl_route_driver.protocol import finished, spacing
from ftl_route_driver.routes import route
import numpy as np
import pytest


def test_cruise_on_straights_and_limit_in_curves():
    r = route('citrus_farm_90')
    profile = speed_profile(r, cruise=11.1)
    first_leg = (2 * 205 - 2 * 24) / 2
    assert speed_at(profile, first_leg / 2) == pytest.approx(11.1)
    assert speed_at(profile, first_leg + 15.0) == pytest.approx(math.sqrt(24.0), rel=0.02)


def test_slow_cruise_is_never_raised():
    profile = speed_profile(route('citrus_farm_30'), cruise=5 / 3.6)
    assert np.max(profile[1]) == pytest.approx(5 / 3.6)


def test_anticipation_never_needs_more_than_decel():
    s, v = speed_profile(route('citrus_farm_30'), cruise=11.1, decel=1.0)
    dv2 = (v[1:] ** 2 - v[:-1] ** 2) / (2 * np.diff(s))
    assert dv2.min() >= -1.0 - 1e-6


def test_spacing_for_four_metres_between_bumpers():
    assert spacing(4.0) == pytest.approx(7.95, abs=0.03)


def test_finished_after_the_last_curve():
    r = route('citrus_farm_90')
    end_of_last_curve = r.length - ((2 * 205 - 2 * 24) / 2 + 2)
    assert not finished(r, end_of_last_curve - 1.0)
    assert finished(r, end_of_last_curve + 1.0)


def test_progress_on_a_closed_loop():
    """
    `protocol.Progress(route).update(x, y)` dá o progresso contínuo, em metros.

    A volta é fechada e a última reta passa do início, então a projeção crua de quem está na
    largada, ou atrás dela, cai no fim da volta. O progresso começa negativo atrás do início e
    cresce sem saltos.
    """
    from ftl_route_driver.protocol import Progress
    r = route('citrus_farm_90')
    tractor = Progress(r)
    assert tractor.update(-205.0, -7.95) == pytest.approx(-7.95, abs=0.1)
    leader = Progress(r)
    assert leader.update(-205.0, 0.0) == pytest.approx(0.0, abs=0.1)
    assert leader.update(-205.0, 50.0) == pytest.approx(50.0, abs=0.1)
    assert not finished(r, tractor.update(-205.0, -7.95))
