"""Testes da amostragem espacial adaptativa (F6.1, R1 e R3)."""
import math

from ftl_trajectory.sampling import AdaptiveSampler, spacing
import pytest


def _arc(radius, length, step=0.01):
    """Posições e rumos de um veículo numa curva à esquerda de raio ``radius``."""
    out = []
    for k in range(int(length / step) + 1):
        theta = k * step / radius
        out.append((radius * math.sin(theta), radius * (1 - math.cos(theta)), theta))
    return out


def _density(radius, length=60.0):
    sampler = AdaptiveSampler(d_base=1.0, d_min=0.25, dpsi_max=math.radians(3.0))
    kept = sum(sampler.offer(*pose) for pose in _arc(radius, length))
    return kept / length


def test_density_grows_with_curvature():
    densities = [_density(r) for r in (1e6, 30.0, 12.0, 6.0)]
    assert densities == sorted(densities)
    assert densities[0] == pytest.approx(1.0, abs=0.05)
    assert densities[-1] > 3.0


def test_spacing_follows_the_formula():
    dpsi = math.radians(3.0)
    assert spacing(math.inf, 1.0, 0.25, dpsi) == 1.0
    assert spacing(12.0, 1.0, 0.25, dpsi) == pytest.approx(12.0 * dpsi)
    assert spacing(2.0, 1.0, 0.25, dpsi) == 0.25
    # Na simulação, o espaçamento medido na curva bate com a fórmula.
    assert 1.0 / _density(12.0) == pytest.approx(12.0 * dpsi, rel=0.05)


def test_no_point_while_stopped():
    sampler = AdaptiveSampler()
    assert sampler.offer(0.0, 0.0, 0.0)
    # Parado, com o ruído do rumo e da posição bem abaixo dos limiares.
    for k in range(1000):
        assert not sampler.offer(0.01 * math.sin(k), 0.01 * math.cos(k), 0.01 * math.sin(k))


def test_turning_in_place_needs_d_min():
    sampler = AdaptiveSampler(d_base=1.0, d_min=0.25, dpsi_max=math.radians(3.0))
    sampler.offer(0.0, 0.0, 0.0)
    assert not sampler.offer(0.1, 0.0, math.radians(10.0))
    assert sampler.offer(0.3, 0.0, math.radians(10.0))


def test_invalid_parameters():
    with pytest.raises(ValueError):
        AdaptiveSampler(d_base=0.2, d_min=0.5)
    with pytest.raises(ValueError):
        AdaptiveSampler(dpsi_max=0.0)
