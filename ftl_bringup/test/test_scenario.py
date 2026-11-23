"""Testes do cenário de experimento (F1.8, R1, R2, R4 e R5), sem simulação.

`ftl_bringup.scenario`:
- `keys()`: as chaves do esquema, com ponto (ex.: `convoy.followers`), em ordem;
- `presets()`: os nomes dos presets em `config/scenarios/`, em ordem;
- `parse_overrides(texto)`: `"a.b=valor c=valor"` vira `{'a.b': valor}`, com o valor lido como
  YAML (número, booleano ou texto);
- `resolve(preset, overrides={})`: os padrões do esquema, depois o preset, depois as sobrescritas,
  validados; devolve um dicionário aninhado;
- `write(config, caminho)` e `read(caminho)` gravam e leem o YAML;
- `convoy_arguments(config)`: os argumentos de texto do `convoy.launch.py` do `agri_gz_bringup`;
- `ScenarioError`, levantada por uma configuração inválida, com a chave na mensagem.
"""
from pathlib import Path

from ftl_bringup.scenario import (convoy_arguments, keys, parse_overrides, presets, read,
                                  resolve, ScenarioError, write)
import pytest

README = Path(__file__).resolve().parents[2] / 'README.md'


def test_presets():
    assert presets() == ['dual_160', 'single_120']
    assert resolve('single_120')['sensors']['camera_mount'] == 'single'
    assert resolve('dual_160')['sensors']['camera_mount'] == 'dual'


def test_defaults():
    config = resolve('single_120')
    assert config['world'] == 'citrus_farm.sdf'
    assert config['gui'] is False
    assert config['convoy']['followers'] == 4
    assert config['sensors']['profile'] == 'nominal'
    assert config['degradation']['gnss'] == ''
    assert config['actuator']['enabled'] is False


def test_keys_cover_the_sections():
    names = keys()
    for key in ('world', 'gui', 'convoy.leader', 'convoy.follower', 'convoy.followers',
                'convoy.spacing', 'convoy.yaw', 'sensors.profile', 'sensors.camera_mount',
                'sensors.tag_family', 'sensors.tag_size', 'degradation.gnss', 'degradation.imu',
                'degradation.odometry', 'degradation.camera', 'degradation.seed',
                'actuator.enabled', 'actuator.steering_rate', 'actuator.time_constant',
                'ftl.params_file'):
        assert key in names, key


def test_overrides_are_typed():
    parsed = parse_overrides('convoy.followers=2 actuator.enabled=true convoy.spacing=12.5 '
                             'degradation.gnss=rtk_float')
    assert parsed == {'convoy.followers': 2, 'actuator.enabled': True, 'convoy.spacing': 12.5,
                      'degradation.gnss': 'rtk_float'}
    assert parse_overrides('') == {}


def test_overrides_win_over_the_preset():
    config = resolve('single_120', parse_overrides('convoy.followers=2 degradation.gnss=dgnss'))
    assert config['convoy']['followers'] == 2
    assert config['degradation']['gnss'] == 'dgnss'


@pytest.mark.parametrize('overrides, key', [
    ({'convoy.tractors': 3}, 'convoy.tractors'),
    ({'convoy.followers': 9}, 'convoy.followers'),
    ({'convoy.followers': 'muitos'}, 'convoy.followers'),
    ({'sensors.profile': 'perfeito'}, 'sensors.profile'),
    ({'degradation.gnss': 'survey_grade'}, 'degradation.gnss'),
    ({'actuator.time_constant': 10.0}, 'actuator.time_constant'),
    ({'world': 'lua.sdf'}, 'world'),
])
def test_invalid_configuration_fails_naming_the_key(overrides, key):
    with pytest.raises(ScenarioError, match=key.replace('.', r'\.')):
        resolve('single_120', overrides)


def test_unknown_preset():
    with pytest.raises(ScenarioError, match='marte'):
        resolve('marte')


def test_round_trip(tmp_path):
    config = resolve('dual_160', parse_overrides('degradation.imu=mems degradation.seed=7'))
    path = tmp_path / 'config.yaml'
    write(config, path)
    assert read(path) == config


def test_convoy_arguments():
    args = convoy_arguments(resolve('dual_160', parse_overrides('convoy.followers=2')))
    assert args['followers'] == '2'
    assert args['camera_mount'] == 'dual'
    assert args['sensor_profile'] == 'nominal'
    assert args['gui'] == 'false'
    assert args['world'] == 'citrus_farm.sdf'
    assert all(isinstance(v, str) for v in args.values())


def test_every_key_is_documented():
    text = README.read_text()
    assert '## Cenário de experimento' in text
    section = text.split('## Cenário de experimento')[1].split('\n## ')[0]
    for key in keys():
        assert f'`{key}`' in section, key
    assert 'experiment.launch.py' in section and '/experiment/config' in section
