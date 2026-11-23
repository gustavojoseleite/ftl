"""Teste de simulação do motorista roteirizado (F2.1, R2 e R4). Marca `simulation`.

Fica no `ftl_bringup`, e não no `ftl_route_driver`, porque testa o experimento inteiro: o ambiente
de teste de um pacote só tem as dependências dele, e quem depende dos dois é o `ftl_bringup`.

O experimento sobe no `citrus_farm_90` com um trator e a rota ligada, a 10,8 km/h. O líder
percorre a primeira reta e a primeira curva do perímetro, e o erro lateral que ele publica em
`/pickup/route/error` fica abaixo de 0,5 m.
"""
import os
import random
import signal
import subprocess
import time

import pytest
import rclpy
from std_msgs.msg import Float64

pytestmark = pytest.mark.simulation
TIMEOUT = 600.0


def test_leader_follows_the_route(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    command = ['ros2', 'launch', 'ftl_bringup', 'experiment.launch.py', 'scenario:=single_120',
               'set:=world=citrus_farm_90.sdf convoy.followers=1 route.enabled=true',
               f'output:={tmp_path}']
    process = subprocess.Popen(command, env=env, start_new_session=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('test_sim_route')
    errors, progress = [], []
    node.create_subscription(Float64, '/pickup/route/error', lambda m: errors.append(m.data), 10)
    node.create_subscription(Float64, '/pickup/route/progress',
                             lambda m: progress.append(m.data), 10)
    try:
        start = time.monotonic()
        while time.monotonic() - start < TIMEOUT and (not progress or progress[-1] < 230.0):
            rclpy.spin_once(node, timeout_sec=0.5)
        assert progress and progress[-1] >= 230.0, 'o líder não percorreu reta e curva'
        assert max(abs(e) for e in errors) < 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
