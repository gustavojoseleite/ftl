"""Testes das chaves da rota no cenário (F2.1, R3 e R5), sem simulação.

O cenário ganha `route.enabled` (falso), `route.cruise_kmh` (10,8), `route.a_lat` (1,0),
`convoy.x` e `convoy.y`. O espaçamento padrão passa a 7,95 m (4 m entre para-choques). Com a
rota ligada, `convoy_arguments` põe a fila no início da rota do mundo, na altura do chão ali
(`z`); um mundo sem rota falha.
"""
import math

from ftl_bringup.scenario import convoy_arguments, keys, resolve, ScenarioError
import pytest


def test_route_keys_and_defaults():
    for key in ('route.enabled', 'route.cruise_kmh', 'route.a_lat', 'convoy.x', 'convoy.y'):
        assert key in keys(), key
    config = resolve('single_120')
    assert config['route'] == {'enabled': False, 'cruise_kmh': pytest.approx(10.8),
                               'a_lat': pytest.approx(1.0)}
    assert config['convoy']['spacing'] == pytest.approx(7.95)


def test_route_places_the_convoy_at_its_start():
    config = resolve('single_120', {'route.enabled': True, 'world': 'citrus_farm_30.sdf'})
    args = convoy_arguments(config)
    assert float(args['x']) == pytest.approx(-205.0)
    assert float(args['y']) == pytest.approx(0.0)
    assert float(args['yaw']) == pytest.approx(math.pi / 2)
    assert float(args['z']) == pytest.approx(8.077, abs=0.01)


def test_without_route_the_convoy_goes_where_asked():
    args = convoy_arguments(resolve('single_120', {'convoy.x': 12.0, 'convoy.y': -3.0}))
    assert (float(args['x']), float(args['y'])) == (12.0, -3.0)
    assert float(args['z']) == 0.0


def test_world_without_a_route_fails():
    with pytest.raises(ScenarioError, match='route'):
        resolve('single_120', {'route.enabled': True, 'world': 'empty.sdf'})


def test_cruise_range():
    with pytest.raises(ScenarioError, match=r'route\.cruise_kmh'):
        resolve('single_120', {'route.cruise_kmh': 80.0})
