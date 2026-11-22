"""O cenário do FTL repassa o perfil dos sensores ao comboio, com o `nominal` como padrão.

Sem isso, a simulação do FTL rodava sempre com o perfil `default` do comboio, cujo GNSS tem 1 m de
ruído, e não com o GNSS RTK da D13.
"""
import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SIM = REPO / 'ftl_gz' / 'launch' / 'sim.launch.py'


def scenario():
    spec = importlib.util.spec_from_file_location('ftl_sim_launch', SIM)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {name: default for name, default, _ in module.SCENARIO}


def test_sensor_profile_is_forwarded_with_the_nominal_default():
    assert scenario()['sensor_profile'] == 'nominal'


def test_the_other_scenario_defaults_are_kept():
    args = scenario()
    assert (args['leader'], args['follower'], args['followers']) == (
        'pickup', 'autonomous_tractor', '2')
    assert args['spacing'] == '10.0'
