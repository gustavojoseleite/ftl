"""
O erro de cada veículo em relação à rota ideal, pela pose real (R1 a R4).

Execução sintética no `citrus_farm_90`: a pickup e um trator andam a 3 m/s pela rota, de s = 160
e s = 150 m, passando pela primeira curva (`curve_0`, de 90 graus à direita, entre s = 181 e
s = 218,7 m). O `ftl/state` de cada um fica sobre a rota. A pose real da pickup fica 0,20 m à
esquerda da rota (para fora da curva) e a do trator, 0,50 m à direita (para dentro). Então:

- `analyze_run` dá, além das métricas de hoje:
  - `route_lateral_mean`, `route_lateral_rmse` e `route_lateral_max` por trecho, para o líder e
    os tratores, medidos na pose real contra a rota;
  - `route_corner_cut` nas curvas: 0,50 m no trator e 0 na pickup;
  - `gamma_route` (trecho `lap`) de cada trator: o RMS do erro à rota dele sobre o do
    precedente, 0,50 / 0,20 = 2,5;
- `paths(pasta)` dá a rota ideal e o caminho real de cada veículo, no ponto de referência;
- `ros2 run ftl_analysis plot_paths <execução> --out <pdf> [--segment curve_0]` desenha a rota e os
  caminhos, com o zoom no trecho pedido;
- `ros2 run ftl_analysis analyze` grava também `table_route.tex` e `route_error.pdf`.
"""
import math
import subprocess

from ftl_analysis.analysis import analyze_run, paths
from ftl_analysis.geometry import ANTENNA
from ftl_bringup.scenario import parse_overrides, resolve, write
from ftl_route_driver.routes import route
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import String
import yaml

VEHICLES = (('pickup', 'pickup', 160.0, 0.20), ('tractor_1', 'autonomous_tractor', 150.0, -0.50))


def _stamp(msg, t):
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)


def _base(r, s, offset, robot):
    x, y, yaw = r.at(s)
    x, y = x - offset * math.sin(yaw), y + offset * math.cos(yaw)
    return x - ANTENNA[robot] * math.cos(yaw), y - ANTENNA[robot] * math.sin(yaw), yaw


def synthetic_curve_run(folder, seed=1):
    config = resolve('single_120', parse_overrides(
        'world=citrus_farm_90.sdf convoy.followers=1 route.enabled=true route.cruise_kmh=10.8'))
    folder.mkdir(parents=True)
    write(config, folder / 'config.yaml')
    (folder / 'run.yaml').write_text(yaml.safe_dump(
        {'reason': 'done', 'duration': 60.0, 'seed': seed, 'commits': {}}))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(folder / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = ['/experiment/config']
    writer.create_topic(rosbag2_py.TopicMetadata(0, topics[0], 'std_msgs/msg/String', 'cdr'))
    for name, _, _, _ in VEHICLES:
        for topic, kind in ((f'/{name}/ftl/state', 'nav_msgs/msg/Odometry'),
                            (f'/ground_truth/{name}/pose', 'geometry_msgs/msg/PoseStamped')):
            writer.create_topic(rosbag2_py.TopicMetadata(len(topics), topic, kind, 'cdr'))
            topics.append(topic)
    writer.write('/experiment/config',
                 serialize_message(String(data=(folder / 'config.yaml').read_text())), 0)
    r = route('citrus_farm_90')
    for k in range(400):
        t = 10.0 + 0.1 * k
        for name, robot, start, offset in VEHICLES:
            s = start + 0.3 * k
            x, y, yaw = _base(r, s, 0.0, robot)
            state = Odometry()
            _stamp(state, t)
            state.pose.pose.position.x, state.pose.pose.position.y = x, y
            state.pose.pose.orientation.z = math.sin(yaw / 2)
            state.pose.pose.orientation.w = math.cos(yaw / 2)
            state.twist.twist.linear.x = 3.0
            writer.write(f'/{name}/ftl/state', serialize_message(state), int(t * 1e9))
            x, y, yaw = _base(r, s, offset, robot)
            truth = PoseStamped()
            _stamp(truth, t)
            truth.pose.position.x, truth.pose.position.y = x, y
            truth.pose.orientation.z = math.sin(yaw / 2)
            truth.pose.orientation.w = math.cos(yaw / 2)
            writer.write(f'/ground_truth/{name}/pose', serialize_message(truth), int(t * 1e9))
    del writer
    return folder


def _values(frame, vehicle, segment):
    rows = frame[(frame.vehicle == vehicle) & (frame.segment == segment)]
    return dict(zip(rows.metric, rows.value))


def test_route_metrics_on_the_true_pose(tmp_path):
    frame = analyze_run(synthetic_curve_run(tmp_path / 'camp' / 'p90' / '1'))
    tractor = _values(frame, 'tractor_1', 'curve_0')
    assert tractor['route_corner_cut'] == pytest.approx(0.50, abs=0.02)
    assert tractor['route_lateral_rmse'] == pytest.approx(0.50, abs=0.02)
    assert tractor['route_lateral_mean'] == pytest.approx(-0.50, abs=0.02)
    assert tractor['route_lateral_max'] == pytest.approx(0.50, abs=0.02)
    leader = _values(frame, 'pickup', 'curve_0')
    assert leader['route_corner_cut'] == pytest.approx(0.0, abs=0.01)
    assert leader['route_lateral_rmse'] == pytest.approx(0.20, abs=0.02)
    straight = _values(frame, 'tractor_1', 'straight_1')
    assert straight['route_lateral_rmse'] == pytest.approx(0.50, abs=0.02)
    assert 'route_corner_cut' not in straight
    # As métricas de hoje continuam no caminho estimado, que fica sobre o do líder.
    assert tractor['lateral_rmse'] == pytest.approx(0.0, abs=0.02)
    gamma = _values(frame, 'tractor_1', 'lap')
    assert gamma['gamma_route'] == pytest.approx(2.5, rel=0.05)


def test_paths(tmp_path):
    data = paths(synthetic_curve_run(tmp_path / 'camp' / 'p90' / '1'))
    assert set(data) == {'route', 'pickup', 'tractor_1'}
    assert data['route'].ndim == 2 and data['route'].shape[1] == 2
    assert len(data['route']) > 100
    r = route('citrus_farm_90')
    first = data['tractor_1'][0]
    x, y, yaw = r.at(150.0)
    assert first == pytest.approx([x + 0.50 * math.sin(yaw), y - 0.50 * math.cos(yaw)], abs=0.02)
    assert len(data['pickup']) == 400 and np.isfinite(data['pickup']).all()


def test_commands(tmp_path):
    run = synthetic_curve_run(tmp_path / 'camp' / 'p90' / '1')
    synthetic_curve_run(tmp_path / 'camp' / 'p90' / '2', seed=2)
    figure = tmp_path / 'paths.pdf'
    subprocess.run(['ros2', 'run', 'ftl_analysis', 'plot_paths', str(run), '--out', str(figure),
                    '--segment', 'curve_0'], check=True, timeout=300)
    assert figure.stat().st_size > 1000
    out = tmp_path / 'out'
    subprocess.run(['ros2', 'run', 'ftl_analysis', 'analyze', str(tmp_path / 'camp'),
                    '--out', str(out)], check=True, timeout=300)
    for name in ('table_route.tex', 'route_error.pdf'):
        assert (out / name).exists(), name
    summary = pd.read_csv(out / 'metrics_summary.csv')
    rows = summary[(summary.vehicle == 'tractor_1') & (summary.metric == 'route_corner_cut')
                   & (summary.segment == 'curve_0')]
    assert rows['mean'].iloc[0] == pytest.approx(0.50, abs=0.02)
