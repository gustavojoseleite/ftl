"""Teste de simulação do launch de experimento (F1.8, R3). Marca `simulation`.

`ros2 launch ftl_bringup experiment.launch.py scenario:=single_120 set:="..." output:=<pasta>` sobe
sem GUI, grava `<pasta>/config.yaml`, publica a configuração em `/experiment/config` (texto YAML,
QoS retida) e o `apply_scenario` aplica o perfil de degradação pedido no nó do sensor.
"""
import os
from pathlib import Path
import random
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
import yaml

pytestmark = pytest.mark.simulation
TIMEOUT = 300.0


def test_experiment_launch(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    command = ['ros2', 'launch', 'ftl_bringup', 'experiment.launch.py', 'scenario:=single_120',
               'set:=convoy.followers=1 degradation.gnss=rtk_float', f'output:={tmp_path}']
    process = subprocess.Popen(command, env=env, start_new_session=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('test_sim_experiment')
    received = []
    node.create_subscription(String, '/experiment/config', received.append, QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(GetParameters, '/tractor_1/gnss/get_parameters')
    profile = None
    try:
        start = time.monotonic()
        while time.monotonic() - start < TIMEOUT and (not received or profile != 'rtk_float'):
            rclpy.spin_once(node, timeout_sec=0.5)
            if client.service_is_ready():
                future = client.call_async(GetParameters.Request(names=['profile']))
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    profile = future.result().values[0].string_value
        assert received, 'a configuração não foi publicada'
        published = yaml.safe_load(received[-1].data)
        assert published['convoy']['followers'] == 1
        assert published['degradation']['gnss'] == 'rtk_float'
        assert yaml.safe_load(Path(tmp_path, 'config.yaml').read_text()) == published
        assert profile == 'rtk_float'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
