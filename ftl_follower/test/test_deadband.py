"""
Testes da zona morta do seguidor (R2 e R3), sem ROS.

`gap_speed(..., deadband=u)`: com |gap − desejado| ≤ u, o seguidor só copia a velocidade da
frente; fora da zona, a correção é `k_gap · (erro − u)` com o sinal do erro. Sem `deadband`, é a
lei da B0. A trilha com espaçamento igual à incerteza não cresce com o vaivém de um veículo
parado.
"""
import random

from ftl_follower.control import gap_speed
from ftl_follower.trail import Trail
import pytest


@pytest.mark.parametrize('gap', [9.0, 9.5, 10.0, 10.5, 11.0])
def test_inside_the_dead_band_only_the_speed_ahead(gap):
    assert gap_speed(0.0, gap, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == 0.0
    assert gap_speed(2.0, gap, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == pytest.approx(2.0)


def test_outside_the_dead_band_the_correction_is_continuous():
    assert gap_speed(0.0, 12.0, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == pytest.approx(0.4)
    assert gap_speed(2.0, 8.0, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == pytest.approx(1.6)
    assert gap_speed(0.0, 11.01, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == pytest.approx(0.004)


def test_without_dead_band_it_is_the_baseline():
    assert gap_speed(2.0, 12.0, 10.0, 0.4, 4.0, 5.0) == pytest.approx(2.8)
    assert gap_speed(2.0, 12.0, 10.0, 0.4, 4.0, 5.0, deadband=0.0) == pytest.approx(2.8)


def test_limits_still_hold_with_dead_band():
    assert gap_speed(3.0, 4.9, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == 0.0
    assert gap_speed(3.0, 40.0, 10.0, 0.4, 4.0, 5.0, deadband=1.0) == 4.0


def jitter(trail, samples=20000, sigma=0.1, seed=3):
    """Um veículo parado em (10, 0), com a estimativa oscilando σ por eixo."""
    rng = random.Random(seed)
    for _ in range(samples):
        trail.append(10.0 + rng.gauss(0.0, sigma), rng.gauss(0.0, sigma))
    return trail.end_s


def test_parked_vehicle_inflates_a_fine_trail():
    # O comportamento observado: com 0,3 m, o vaivém vira dezenas de metros de caminho.
    assert jitter(Trail(spacing=0.3)) > 50.0


def test_parked_vehicle_does_not_grow_a_trail_spaced_by_the_uncertainty():
    assert jitter(Trail(spacing=1.0)) < 1.0
