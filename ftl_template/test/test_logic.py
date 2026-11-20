"""
Testes de unidade da lógica do modelo: o limitador de taxa.

A lógica é Python puro, sem ``rclpy``, e se testa sem o ROS. Num pacote novo,
estes testes dão lugar aos da lógica dele.
"""

from ftl_template.logic import limit_rate
import pytest


def test_target_within_reach_passes_through():
    assert limit_rate(0.0, 0.5, max_rate=1.0, dt=1.0) == pytest.approx(0.5)


def test_step_up_is_limited_by_the_rate():
    assert limit_rate(0.0, 10.0, max_rate=1.0, dt=0.5) == pytest.approx(0.5)


def test_step_down_is_limited_by_the_rate():
    assert limit_rate(1.0, -10.0, max_rate=2.0, dt=0.25) == pytest.approx(0.5)


def test_zero_rate_holds_the_output():
    assert limit_rate(3.0, 10.0, max_rate=0.0, dt=1.0) == pytest.approx(3.0)


@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_non_positive_dt_is_an_error(dt):
    with pytest.raises(ValueError):
        limit_rate(0.0, 1.0, max_rate=1.0, dt=dt)


def test_negative_rate_is_an_error():
    with pytest.raises(ValueError):
        limit_rate(0.0, 1.0, max_rate=-1.0, dt=0.1)
