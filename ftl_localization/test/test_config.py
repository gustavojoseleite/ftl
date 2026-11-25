"""
Parâmetros do UKF e do `gnss_odometry` (F3.1, R1 e R2), lidos do `config/localization.yaml`.

O arquivo tem duas seções, `/**/ukf` e `/**/gnss_odometry`, cada uma com `ros__parameters`, para
valer em qualquer namespace de veículo. O UKF trabalha em 2D, a 20 Hz, no referencial `ftl_map` e
sem publicar TF; funde a velocidade da odometria, o rumo da IMU e a posição do GNSS convertida
pelo `gnss_odometry`. O `datum` do `gnss_odometry` é o do `pose_estimator` no
`ftl_bringup/config/ftl.yaml`: o comboio inteiro usa um referencial só. As listas de números são
todas de reais, como o ROS exige.
"""
from pathlib import Path

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parent
X, Y, YAW, VX, VYAW = 0, 1, 5, 6, 11


def _load():
    config = yaml.safe_load((PACKAGE / 'config' / 'localization.yaml').read_text())
    return (config['/**/ukf']['ros__parameters'],
            config['/**/gnss_odometry']['ros__parameters'])


def test_ukf_frames_and_rate():
    ukf, _ = _load()
    assert ukf['world_frame'] == ukf['map_frame'] == 'ftl_map'
    assert ukf['base_link_frame'] == 'base_footprint'
    assert ukf['odom_frame'] != ukf['map_frame']
    assert ukf['two_d_mode'] is True
    assert float(ukf['frequency']) == 20.0
    assert ukf['publish_tf'] is False


def test_ukf_inputs():
    ukf, _ = _load()
    assert ukf['odom0'] == 'odom'
    assert ukf['imu0'] == 'imu/data'
    assert ukf['odom1'] == 'odometry/gps'
    for name in ('odom0_config', 'imu0_config', 'odom1_config'):
        assert len(ukf[name]) == 15, name
    assert ukf['odom0_config'][VX] is True
    assert ukf['odom0_config'][X] is False and ukf['odom0_config'][Y] is False
    assert ukf['imu0_config'][YAW] is True and ukf['imu0_config'][VYAW] is True
    assert ukf['odom1_config'][X] is True and ukf['odom1_config'][Y] is True


def test_gnss_uses_the_convoy_datum():
    _, gnss = _load()
    ftl = yaml.safe_load((REPO / 'ftl_bringup' / 'config' / 'ftl.yaml').read_text())
    estimator = ftl['/**/pose_estimator']['ros__parameters']
    convoy = [estimator['datum_lat'], estimator['datum_lon']]
    assert [gnss['datum_lat'], gnss['datum_lon']] == convoy
    assert gnss['world_frame'] == 'ftl_map'
    assert gnss['base_frame'] == 'base_footprint'
    assert gnss['gnss_frame'] == 'gnss_link'
    assert float(gnss['gnss_sigma_h']) == 0.02


def test_number_lists_are_all_floats():
    ukf, _ = _load()
    for name, value in ukf.items():
        if isinstance(value, list) and any(isinstance(v, (int, float)) and
                                           not isinstance(v, bool) for v in value):
            assert all(isinstance(v, float) for v in value), name
