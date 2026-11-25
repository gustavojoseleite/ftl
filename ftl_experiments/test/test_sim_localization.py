"""
A B0 e o UKF no mesmo conjunto de sensores de campo (F3.1, R5). Marca `simulation`.

Roda o caso `quick` (`citrus_farm_90`, 10,8 km/h, 4 tratores, 120 s simulados) duas vezes, com a
semente 1: rótulo `b0` com o `pose_estimator` e rótulo `ukf` com o UKF. Os dois usam o perfil
`nominal` (RTK fixo), a IMU `mems` e a odometria `field`. Depois, a análise da F2.4.

Nas retas, o RMSE de estimação de cada trator com o UKF fica em no máximo 0,10 m, e a média dos 4
fica abaixo da do `pose_estimator`.
"""
import os
import random
import subprocess

import pandas as pd
import pytest

pytestmark = pytest.mark.simulation

FIELD = ('world=citrus_farm_90.sdf convoy.followers=4 route.enabled=true route.cruise_kmh=10.8 '
         'sensors.camera_mount=single sensors.profile=nominal degradation.imu=mems '
         'degradation.odometry=field')
TRACTORS = {'tractor_1', 'tractor_2', 'tractor_3', 'tractor_4'}


def _env():
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    return env


def test_ukf_against_the_baseline(tmp_path):
    for label, localization in (('b0', 'pose_estimator'), ('ukf', 'ukf')):
        subprocess.run(['ros2', 'run', 'ftl_experiments', 'run', '--campaign', 'loc',
                        '--set', f'{FIELD} ftl.localization={localization}', '--seeds', '1',
                        '--root', str(tmp_path), '--label', label, '--duration', '120'],
                       env=_env(), check=True, timeout=1500)
    subprocess.run(['ros2', 'run', 'ftl_analysis', 'analyze', str(tmp_path / 'loc'),
                    '--out', str(tmp_path / 'loc' / 'analysis'), '--reasons', 'done,timeout'],
                   env=_env(), check=True, timeout=600)
    runs = pd.read_csv(tmp_path / 'loc' / 'analysis' / 'metrics_runs.csv')
    estimation = runs[(runs.metric == 'estimation_rmse') & (runs.kind == 'straight')
                      & runs.vehicle.isin(TRACTORS)]
    by_label = {label: rows.groupby('vehicle').value.mean()
                for label, rows in estimation.groupby('label')}
    assert set(by_label) == {'b0', 'ukf'}, estimation.label.unique()
    ukf, b0 = by_label['ukf'], by_label['b0']
    assert set(ukf.index) == TRACTORS and set(b0.index) == TRACTORS
    assert (ukf <= 0.10).all(), ukf.to_string()
    assert ukf.mean() < b0.mean(), f'ukf {ukf.mean():.3f} m, b0 {b0.mean():.3f} m'
