"""
Teste de simulação do executor de experimentos (F2.3, R1 a R3). Marca `simulation`.

`ros2 run ftl_experiments run` roda uma semente do `citrus_farm_90` com um trator e a rota ligada,
com `--duration 40` (o tempo limite, para o teste ser curto). Ao sair, a pasta da execução tem o
`run.yaml` (motivo `timeout`), a `config.yaml` resolvida e o *bag* MCAP com os tópicos da análise.
"""
import os
import random
import subprocess

import pytest
import rosbag2_py
import yaml

pytestmark = pytest.mark.simulation


def test_one_short_run(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    command = ['ros2', 'run', 'ftl_experiments', 'run', '--campaign', 'teste', '--scenario',
               'single_120', '--set',
               'world=citrus_farm_90.sdf convoy.followers=1 route.enabled=true',
               '--seeds', '1', '--root', str(tmp_path), '--duration', '40']
    subprocess.run(command, env=env, check=True, timeout=900)
    run = tmp_path / 'teste' / 'single_120' / '1'
    info = yaml.safe_load((run / 'run.yaml').read_text())
    assert info['reason'] == 'timeout' and info['seed'] == 1
    assert info['commits'], 'os commits do workspace não foram gravados'
    config = yaml.safe_load((run / 'config.yaml').read_text())
    assert config['degradation']['seed'] == 1
    metadata = rosbag2_py.Info().read_metadata(str(run / 'bag'), 'mcap')
    counts = {t.topic_metadata.name: t.message_count
              for t in metadata.topics_with_message_count}
    for name in ('/ground_truth/pickup/pose', '/ground_truth/tractor_1/pose',
                 '/tractor_1/ftl/state', '/pickup/route/progress', '/experiment/config'):
        assert counts.get(name, 0) > 0, name
