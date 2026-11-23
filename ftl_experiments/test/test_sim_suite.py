"""
Teste de simulação do conjunto básico em modo rápido (F2.5, R2 a R7). Marca `simulation`.

`ros2 run ftl_experiments suite --campaign teste --quick` roda o caso `quick` (4 tratores, 120 s
simulados) e a análise. A execução termina por `timeout` (o tempo simulado), e o resumo tem o
`lateral_rmse` dos 4 tratores na primeira reta, com o `estimation_rmse` de cada um abaixo de 1 m.
"""
import os
import random
import subprocess

import pandas as pd
import pytest
import yaml

pytestmark = pytest.mark.simulation


def test_quick_suite(tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    subprocess.run(['ros2', 'run', 'ftl_experiments', 'suite', '--campaign', 'teste', '--quick',
                    '--root', str(tmp_path)], env=env, check=True, timeout=1500)
    info = yaml.safe_load((tmp_path / 'teste' / 'quick' / '1' / 'run.yaml').read_text())
    assert info['reason'] == 'timeout'
    assert info['attempts'] >= 1
    summary = pd.read_csv(tmp_path / 'teste' / 'analysis' / 'metrics_summary.csv')
    rows = summary[(summary.metric == 'lateral_rmse') & (summary.segment == 'straight_0')
                   & (summary.label == 'quick')]
    assert set(rows.vehicle) == {'tractor_1', 'tractor_2', 'tractor_3', 'tractor_4'}
    assert (tmp_path / 'teste' / 'analysis' / 'table_lateral.tex').exists()
    # Em C0, o caminho estimado da B0 fica a 0,2-0,4 m da pose real. Um comboio que se perde
    # reprova aqui: com o IMU fora do ENU o erro chegou a 150-220 m, e com o líder arrancando
    # antes dos estimadores, a 1-5 m. A qualidade da B0 é a campanha que mede.
    estimation = summary[(summary.metric == 'estimation_rmse') & (summary.segment == 'straight_0')
                         & (summary.label == 'quick')]
    assert set(estimation.vehicle) == {'tractor_1', 'tractor_2', 'tractor_3', 'tractor_4'}
    assert (estimation['mean'] < 1.0).all(), estimation[['vehicle', 'mean']].to_string()
