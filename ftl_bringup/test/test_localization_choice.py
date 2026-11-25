"""
A escolha entre o `pose_estimator` da B0 e o UKF (F3.1, R4).

- O cenário tem a chave `ftl.localization`, com `pose_estimator` (o padrão) e `ukf`; outro valor
  é rejeitado com a chave na mensagem.
- `ftl_bringup.scenario.ftl_arguments(config)` monta os argumentos de texto do `ftl.launch.py`:
  `use_sim_time`, `leader`, `followers`, `localization` e, se houver, `params_file`.
- `ros2 launch ftl_bringup ftl.launch.py localization:=ukf followers:=1` sobe `ukf` e
  `gnss_odometry` em `/pickup` e `/tractor_1`, e nenhum `pose_estimator`; o padrão continua
  subindo o `pose_estimator`.
"""
import os
import random
import signal
import subprocess
import time

from ftl_bringup.scenario import ftl_arguments, resolve, ScenarioError
import pytest
import rclpy


def test_scenario_key():
    assert resolve('single_120')['ftl']['localization'] == 'pose_estimator'
    assert resolve('single_120', {'ftl.localization': 'ukf'})['ftl']['localization'] == 'ukf'
    with pytest.raises(ScenarioError, match='ftl.localization'):
        resolve('single_120', {'ftl.localization': 'kalman'})


def test_ftl_arguments():
    config = resolve('single_120', {'convoy.followers': 2, 'ftl.localization': 'ukf'})
    assert ftl_arguments(config) == {'use_sim_time': 'true', 'leader': 'pickup',
                                     'followers': '2', 'localization': 'ukf'}
    default = ftl_arguments(resolve('single_120'))
    assert default['localization'] == 'pose_estimator'
    assert 'params_file' not in default


def _nodes(arguments, seconds=12.0):
    domain = str(random.randint(30, 99))
    os.environ['ROS_DOMAIN_ID'] = domain
    launch = subprocess.Popen(['ros2', 'launch', 'ftl_bringup', 'ftl.launch.py', *arguments],
                              env=dict(os.environ, ROS_DOMAIN_ID=domain),
                              start_new_session=True)
    rclpy.init()
    try:
        probe = rclpy.create_node('probe')
        found = set()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.2)
            found |= {f'{ns.rstrip("/")}/{name}'
                      for name, ns in probe.get_node_names_and_namespaces()}
        probe.destroy_node()
        return found
    finally:
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)


def test_launch_with_ukf():
    found = _nodes(['localization:=ukf', 'followers:=1'])
    for vehicle in ('pickup', 'tractor_1'):
        assert f'/{vehicle}/ukf' in found, found
        assert f'/{vehicle}/gnss_odometry' in found, found
        assert f'/{vehicle}/pose_estimator' not in found, found


def test_launch_default_keeps_the_baseline():
    found = _nodes(['followers:=1'])
    assert '/pickup/pose_estimator' in found and '/tractor_1/pose_estimator' in found, found
    assert '/pickup/ukf' not in found, found
