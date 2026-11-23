"""Esquema, presets e validação do cenário de um experimento.

A configuração resolvida é um dicionário aninhado (`convoy.followers` vira
`config['convoy']['followers']`). A ordem de precedência é: os padrões do esquema, o preset e
as sobrescritas da linha de comando.
"""
from collections import namedtuple
import math
import os
from pathlib import Path
import shlex

import yaml


class ScenarioError(ValueError):
    """Configuração inválida; a mensagem nomeia a chave e o problema."""


# kind: 'str', 'bool', 'int' ou 'float'. choices pode ser uma lista ou uma função que a devolve.
Key = namedtuple('Key', 'name default kind low high choices unit effect')


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def worlds():
    """Os mundos do `agri_gz_worlds`, em ordem."""
    return sorted(path.name for path in (_share('agri_gz_worlds') / 'worlds').glob('*.sdf'))


def _profiles(sensor):
    def names():
        from agri_sensor_degradation import profiles
        return [''] + profiles.names(sensor)
    return names


def _key(name, default, kind='str', low=None, high=None, choices=None, unit='-', effect=''):
    return Key(name, default, kind, low, high, choices, unit, effect)


SCHEMA = [
    _key('world', 'citrus_farm.sdf', choices=worlds, effect='Mundo do Gazebo.'),
    _key('gui', False, 'bool', effect='Abre a janela do Gazebo.'),
    _key('convoy.leader', 'pickup', choices=['pickup', 'autonomous_tractor'],
         effect='Tipo do veículo da frente.'),
    _key('convoy.follower', 'autonomous_tractor', effect='Tipo de cada seguidor.'),
    _key('convoy.followers', 4, 'int', 1, 4, unit='veículos', effect='Quantos seguidores.'),
    _key('convoy.spacing', 10.0, 'float', 4.0, 30.0, unit='m',
         effect='Distância entre os `base_link` de dois veículos.'),
    _key('convoy.yaw', 0.0, 'float', -math.pi, math.pi, unit='rad',
         effect='Rumo da fila inteira.'),
    _key('sensors.profile', 'nominal', choices=['nominal', 'default', 'ideal'],
         effect='Perfil de GNSS e IMU do simulador.'),
    _key('sensors.camera_mount', 'single', choices=['single', 'dual'],
         effect='Uma câmera de 120 graus ou duas de 90.'),
    _key('sensors.tag_family', '36h11', effect='Família das tags AprilTag.'),
    _key('sensors.tag_size', 0.5, 'float', 0.1, 2.0, unit='m', effect='Lado da tag.'),
    _key('degradation.gnss', '', choices=_profiles('gnss'),
         effect='Perfil de degradação do GNSS.'),
    _key('degradation.imu', '', choices=_profiles('imu'),
         effect='Perfil de degradação da IMU.'),
    _key('degradation.odometry', '', choices=_profiles('odometry'),
         effect='Perfil de degradação da odometria.'),
    _key('degradation.camera', '', choices=_profiles('camera'),
         effect='Perfil de degradação da câmera dos seguidores.'),
    _key('degradation.seed', 0, 'int',
         effect='Semente do ruído; cada veículo soma o seu índice.'),
    _key('actuator.enabled', False, 'bool',
         effect='Liga o atraso do atuador no `cmd_vel_limiter`.'),
    _key('actuator.steering_rate', 0.5, 'float', 0.05, 2.0, unit='rad/s',
         effect='Velocidade máxima de giro da direção.'),
    _key('actuator.time_constant', 0.5, 'float', 0.05, 3.0, unit='s',
         effect='Constante de tempo da resposta do atuador.'),
    _key('ftl.params_file', '',
         effect='Parâmetros dos nós do FTL; vazio usa o `ftl.yaml` padrão.'),
]
BY_NAME = {key.name: key for key in SCHEMA}


def keys():
    """Os nomes das chaves do esquema, com ponto, em ordem."""
    return [key.name for key in SCHEMA]


def presets_dir():
    return _share('ftl_bringup') / 'config' / 'scenarios'


def presets():
    """Os nomes dos presets, em ordem."""
    return sorted(path.stem for path in presets_dir().glob('*.yaml'))


def parse_overrides(text):
    """Lê `"a.b=valor c=valor"`; cada valor é YAML (número, booleano ou texto)."""
    result = {}
    for item in shlex.split(text or ''):
        name, separator, value = item.partition('=')
        if not separator or not name:
            raise ScenarioError(f"{item}: expected key=value")
        result[name] = yaml.safe_load(value) if value != '' else ''
    return result


def _check(key, value):
    """Valida e normaliza o valor de uma chave."""
    name = key.name
    if key.kind == 'bool':
        if not isinstance(value, bool):
            raise ScenarioError(f"{name}: expected true or false, got {value!r}")
        return value
    if key.kind == 'str':
        if not isinstance(value, str):
            raise ScenarioError(f"{name}: expected text, got {value!r}")
        choices = key.choices() if callable(key.choices) else key.choices
        if choices is not None and value not in choices:
            raise ScenarioError(f"{name}: {value!r} is not one of {list(choices)}")
        if name == 'ftl.params_file' and value and not os.path.isfile(value):
            raise ScenarioError(f"{name}: file {value!r} does not exist")
        return value
    is_number = isinstance(value, (int, float)) and not isinstance(value, bool)
    if not is_number or (key.kind == 'int' and not isinstance(value, int)):
        raise ScenarioError(f"{name}: expected a {'whole ' if key.kind == 'int' else ''}number, "
                            f"got {value!r}")
    if key.kind == 'float':
        value = float(value)
    if not math.isfinite(value):
        raise ScenarioError(f"{name}: {value!r} is not finite")
    if key.low is not None and not key.low <= value <= key.high:
        raise ScenarioError(f"{name}: {value} is outside {key.low}..{key.high}")
    return value


def _flatten(tree, prefix=''):
    flat = {}
    for name, value in (tree or {}).items():
        full = f'{prefix}{name}'
        if isinstance(value, dict):
            flat.update(_flatten(value, f'{full}.'))
        else:
            flat[full] = value
    return flat


def resolve(preset, overrides=None):
    """Devolve a configuração aninhada: padrões, preset e sobrescritas, validados."""
    path = presets_dir() / f'{preset}.yaml'
    if not path.is_file():
        raise ScenarioError(f"preset {preset!r} not found; available: {presets()}")
    flat = {key.name: key.default for key in SCHEMA}
    for source in (_flatten(yaml.safe_load(path.read_text(encoding='utf-8'))), overrides or {}):
        for name, value in source.items():
            if name not in BY_NAME:
                raise ScenarioError(f"{name}: unknown key; known keys: {keys()}")
            flat[name] = value
    config = {}
    for key in SCHEMA:
        value = _check(key, flat[key.name])
        node = config
        *parents, leaf = key.name.split('.')
        for parent in parents:
            node = node.setdefault(parent, {})
        node[leaf] = value
    return config


def write(config, path):
    """Grava a configuração em YAML."""
    Path(path).write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')


def read(path):
    """Lê a configuração gravada por `write`."""
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def convoy_arguments(config):
    """Os argumentos de texto do `convoy.launch.py` do `agri_gz_bringup`."""
    convoy, sensors = config['convoy'], config['sensors']
    args = {
        'world': config['world'],
        'gui': str(config['gui']).lower(),
        'leader': convoy['leader'],
        'follower': convoy['follower'],
        'followers': convoy['followers'],
        'spacing': convoy['spacing'],
        'yaw': convoy['yaw'],
        'sensor_profile': sensors['profile'],
        'camera_mount': sensors['camera_mount'],
        'tag_family': sensors['tag_family'],
        'tag_size': sensors['tag_size'],
    }
    return {name: str(value) for name, value in args.items()}
