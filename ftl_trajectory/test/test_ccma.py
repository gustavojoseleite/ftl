"""Testes do CCMA (F6.1, R2): recupera o raio de um arco ruidoso, ao contrário da média móvel."""
import math
import random

from ftl_trajectory.ccma import Ccma, ccma, correction, curvature, gaussian_weights
import numpy as np
import pytest

RADIUS = 24.0


def _noisy_arc(radius=RADIUS, step=1.0, sweep=math.pi, noise=0.02, seed=3):
    rng = random.Random(seed)
    n = int(radius * sweep / step) + 1
    return [(radius * math.cos(k * step / radius) + rng.gauss(0.0, noise),
             radius * math.sin(k * step / radius) + rng.gauss(0.0, noise)) for k in range(n)]


def _fit_radius(points):
    """Raio do círculo que melhor ajusta os pontos (ajuste algébrico de Kasa)."""
    pts = np.asarray(points)
    a = np.column_stack([2 * pts[:, 0], 2 * pts[:, 1], np.ones(len(pts))])
    b = (pts ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    return math.sqrt(c + cx * cx + cy * cy)


def _moving_average(points, half):
    w = gaussian_weights(half)
    return [tuple(sum(w[k] * points[i + k - half][d] for k in range(2 * half + 1))
                  for d in (0, 1)) for i in range(half, len(points) - half)]


def test_weights_are_symmetric_and_normalized():
    w = gaussian_weights(5)
    assert len(w) == 11
    assert sum(w) == pytest.approx(1.0)
    assert w == pytest.approx(w[::-1])
    assert gaussian_weights(0) == [1.0]


def test_curvature_sign_and_value():
    a, b, c = (RADIUS, 0.0), (0.0, RADIUS), (-RADIUS, 0.0)
    assert curvature(a, b, c) == pytest.approx(1.0 / RADIUS)
    assert curvature(c, b, a) == pytest.approx(-1.0 / RADIUS)
    assert curvature((0, 0), (1, 0), (2, 0)) == 0.0
    assert correction(0.0, 1.0, 5) == 0.0


def test_ccma_recovers_the_radius_and_ma_does_not():
    points = _noisy_arc()
    margin = 12  # longe das pontas, onde as janelas encolhem
    ma = _moving_average(points, 8)[margin:-margin]
    corrected = ccma(points, w_ma=8, w_cc=3)[margin:-margin]
    r_ma, r_ccma = _fit_radius(ma), _fit_radius(corrected)
    assert r_ma < RADIUS * 0.995, f'a média móvel deveria encolher o raio: {r_ma:.3f} m'
    assert abs(r_ccma - RADIUS) / RADIUS <= 0.02, f'raio do CCMA: {r_ccma:.3f} m'
    assert abs(r_ccma - RADIUS) < abs(r_ma - RADIUS)


def test_straight_line_stays_straight():
    points = [(float(k), 0.0) for k in range(40)]
    assert all(abs(y) < 1e-9 for _, y in ccma(points))


def test_incremental_equals_batch():
    points = _noisy_arc(sweep=math.pi / 2, seed=7)
    batch = ccma(points, 5, 3)
    filt = Ccma(5, 3)
    chunks = [3, 1, 7, 2, 11, 5, 9, 100]
    k = 0
    for size in chunks:
        for x, y in points[k:k + size]:
            filt.add(x, y)
        filt.update()
        k += size
    assert len(filt.smoothed) == len(batch) == len(points) - filt.reach
    assert np.allclose(filt.smoothed, batch, atol=1e-12)
    assert filt.pending() == points[len(batch):]


def test_nothing_final_with_few_points():
    filt = Ccma(5, 3)
    for k in range(filt.reach):
        filt.add(k, 0.0)
    assert filt.update() == []
    assert len(filt.pending()) == filt.reach
