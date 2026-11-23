"""
A lógica do pacote não depende do ROS: só os módulos de nó importam o rclpy.

Os módulos de nó são os que o setup.py instala como executáveis; os outros são
lógica pura, testável sem o ROS (seção 6 do plano). O teste vale sem mudança
num pacote copiado do modelo.
"""

import ast
from pathlib import Path
import re

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
ROS_MODULES = {'rclpy'}


def node_modules():
    """Devolve os módulos que o setup.py instala como executáveis."""
    setup = (PACKAGE / 'setup.py').read_text()
    return set(re.findall(r'=\s*[\w.]+\.(\w+):main', setup))


def logic_modules():
    """Lista os módulos do pacote que não são nós nem o __init__.py."""
    modules = sorted((PACKAGE / PACKAGE.name).glob('*.py'))
    return [m for m in modules if m.stem not in node_modules() and m.name != '__init__.py']


def imported(path):
    """Devolve os nomes de primeiro nível que um módulo importa."""
    names = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            names.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names.add(node.module.split('.')[0])
    return names


def test_package_installs_a_node():
    assert node_modules(), 'o setup.py não instala nenhum nó'


def test_package_has_logic_apart_from_the_node():
    assert logic_modules(), 'o pacote não tem módulo de lógica fora do nó'


@pytest.mark.parametrize('module', logic_modules(), ids=lambda p: p.name)
def test_logic_does_not_import_ros(module):
    assert not imported(module) & ROS_MODULES, f'{module.name} importa o ROS'
