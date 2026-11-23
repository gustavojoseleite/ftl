"""
Testes das contas da análise (F2.4, R1 a R4), sem ROS, com resultados conhecidos.

- `geometry.reference_point(x, y, yaw, robot)`: a antena projetada no chão (`ANTENNA`, m à
  frente da base: trator 1,075, pickup 1,545);
- `geometry.Polyline(xy)`: `length` e `project(x, y)` → (s, lateral com sinal, positivo à
  esquerda);
- `segments.segments(route)`: os trechos (`kind` 'straight' ou 'curve', `index`, `s0`, `s1`,
  `turn` = +1 à esquerda, −1 à direita, 0 na reta);
- `metrics.summary(erros)`: `rmse`, `max` (do valor absoluto) e `p95`;
- `metrics.corner_cut(laterais, turn)`: o maior desvio para dentro da curva (0 se nenhum);
- `metrics.string_stability(normas)`: `normas[i] / normas[i-1]`;
- `metrics.rms_rate(t, valores)`: o RMS da derivada no tempo;
- `stats.mean_ci(valores)`: (média, meia-largura do IC de 95%, distribuição t).
"""
import math

from ftl_analysis.geometry import ANTENNA, Polyline, reference_point
from ftl_analysis.metrics import corner_cut, rms_rate, string_stability, summary
from ftl_analysis.segments import segments
from ftl_analysis.stats import mean_ci
from ftl_route_driver.routes import route
import numpy as np
import pytest


def test_reference_point():
    assert ANTENNA == {'autonomous_tractor': 1.075, 'pickup': 1.545}
    assert reference_point(0.0, 0.0, math.pi / 2, 'autonomous_tractor') == pytest.approx(
        (0.0, 1.075))
    assert reference_point(1.0, 2.0, 0.0, 'pickup') == pytest.approx((2.545, 2.0))


def test_polyline_projection():
    line = Polyline(np.c_[np.linspace(0, 100, 201), np.zeros(201)])
    assert line.length == pytest.approx(100.0)
    s, lateral = line.project(40.0, 0.3)
    assert (s, lateral) == pytest.approx((40.0, 0.3))
    assert line.project(40.0, -0.2)[1] == pytest.approx(-0.2)


def test_constant_offset_gives_its_rmse():
    errors = np.full(500, 0.10)
    result = summary(errors)
    assert (result['rmse'], result['max'], result['p95']) == pytest.approx((0.10, 0.10, 0.10))
    assert summary(-errors)['max'] == pytest.approx(0.10)


def test_corner_cut():
    angles = np.linspace(0, math.pi / 2, 200)
    leader = np.c_[30 * np.cos(angles), 30 * np.sin(angles)]
    inside = np.c_[29.5 * np.cos(angles), 29.5 * np.sin(angles)]
    line = Polyline(leader)
    laterals = np.array([line.project(x, y)[1] for x, y in inside[5:-5]])
    # A curva vira à esquerda (anti-horária): para dentro é à esquerda, lateral positiva.
    assert corner_cut(laterals, +1) == pytest.approx(0.5, abs=0.01)
    assert corner_cut(laterals, -1) == pytest.approx(0.0)


def test_string_stability():
    assert string_stability([1.0, 2.0, 2.0, 1.0]) == pytest.approx([2.0, 1.0, 0.5])


def test_rms_rate():
    t = np.linspace(0.0, 10.0, 1001)
    assert rms_rate(t, 0.3 * t) == pytest.approx(0.3, rel=1e-3)


def test_mean_ci():
    mean, half = mean_ci([1.0, 2.0, 3.0])
    assert mean == pytest.approx(2.0)
    assert half == pytest.approx(4.303 * 1.0 / math.sqrt(3), rel=1e-3)
    assert math.isnan(mean_ci([5.0])[1])


@pytest.mark.parametrize('name, curves', [('citrus_farm_30', 12), ('citrus_farm_60', 6),
                                          ('citrus_farm_90', 4)])
def test_segments_of_the_perimeters(name, curves):
    parts = segments(route(name))
    assert sum(p.kind == 'curve' for p in parts) == curves
    assert sum(p.kind == 'straight' for p in parts) == curves + 1
    assert parts[0].kind == 'straight' and parts[0].s0 == pytest.approx(0.0)
    assert all(a.s1 == pytest.approx(b.s0) for a, b in zip(parts, parts[1:]))
    assert all(p.turn == -1 for p in parts if p.kind == 'curve')
