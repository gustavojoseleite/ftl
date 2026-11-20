"""
Testes do nó do modelo: os parâmetros declarados e documentados.

Todo parâmetro de um nó do ftl diz a unidade e o efeito na descrição, e os
numéricos dizem a faixa válida, que o ROS confere a cada valor novo. Num pacote
novo, este teste fica, com o nó dele no lugar do ``LimiterNode``.
"""

from ftl_template.node import LimiterNode
import pytest
from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.parameter import Parameter

# Os parâmetros que o rclpy declara sozinho em todo nó.
ROS_BUILTINS = {'use_sim_time', 'start_type_description_service'}
NUMERIC = (ParameterType.PARAMETER_DOUBLE, ParameterType.PARAMETER_INTEGER)


@pytest.fixture
def node():
    rclpy.init()
    limiter = LimiterNode()
    yield limiter
    limiter.destroy_node()
    rclpy.shutdown()


def declared(node):
    """Lista os parâmetros que o próprio nó declara."""
    return [n for n in node.list_parameters([], 0).names if n not in ROS_BUILTINS]


def test_declares_its_parameters(node):
    assert declared(node) == ['max_rate']


def test_every_parameter_states_unit_and_effect(node):
    for name in declared(node):
        description = node.describe_parameter(name).description
        assert 'Unidade:' in description, f'{name} não diz a unidade'
        assert 'Efeito:' in description, f'{name} não diz o efeito'


def test_every_numeric_parameter_has_a_range(node):
    for name in declared(node):
        descriptor = node.describe_parameter(name)
        if descriptor.type in NUMERIC:
            has_range = descriptor.floating_point_range or descriptor.integer_range
            assert has_range, f'{name} não tem faixa'


def test_ros_rejects_a_negative_rate(node):
    result = node.set_parameters([Parameter('max_rate', value=-1.0)])[0]
    assert not result.successful
    assert node.get_parameter('max_rate').value == pytest.approx(1.0)
