"""
Os parâmetros do `route_driver` saem da configuração resolvida (F2.5, R6), sem ROS.

`ftl_bringup.scenario.driver_parameters(config)` devolve o dicionário de parâmetros do nó: a rota
(o mundo sem `.sdf`), o cruzeiro, a aceleração lateral, o líder, o último trator, o
`use_sim_time` e o `wait_for` com o `ftl/state` de todos os veículos, do líder ao último trator.
"""
from ftl_bringup.scenario import driver_parameters, parse_overrides, resolve


def test_driver_parameters():
    config = resolve('single_120', parse_overrides(
        'world=citrus_farm_60.sdf convoy.followers=3 route.enabled=true route.cruise_kmh=20'))
    params = driver_parameters(config)
    assert params == {
        'route': 'citrus_farm_60', 'cruise_kmh': 20.0, 'a_lat': config['route']['a_lat'],
        'leader': 'pickup', 'last': 'tractor_3', 'use_sim_time': True,
        'wait_for': ['/pickup/ftl/state', '/tractor_1/ftl/state', '/tractor_2/ftl/state',
                     '/tractor_3/ftl/state']}
