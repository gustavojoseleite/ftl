"""
Integração da análise (F2.4, R1 a R5) com um *bag* sintético de resultado conhecido.

A execução sintética tem a pickup e um trator no `citrus_farm_90`, andando a 3 m/s pela primeira
reta da rota. O `ftl/state` (a base estimada) do trator fica 0,10 m à esquerda do caminho do
líder; a pose real de cada veículo fica 0,20 m a leste da estimada. Então:
- `analysis.analyze_run(pasta)` dá, no trecho `straight_0` do `tractor_1`, `lateral_rmse` 0,10 m
  e `estimation_rmse` 0,20 m;
- `ros2 run ftl_analysis analyze <campanha> --out <pasta>` grava `metrics_runs.csv`,
  `metrics_summary.csv`, `table_lateral.tex` e `lateral_rmse.pdf`.
"""
import math
import subprocess

from ftl_analysis.analysis import analyze_run
from ftl_analysis.geometry import ANTENNA
from ftl_bringup.scenario import parse_overrides, resolve, write
from ftl_route_driver.routes import route
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import pandas as pd
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import String
import yaml


def odometry(t, x, y, yaw, v):
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.twist.twist.linear.x = v
    return msg


def pose(t, x, y, yaw):
    msg = PoseStamped()
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def synthetic_run(folder):
    config = resolve('single_120', parse_overrides(
        'world=citrus_farm_90.sdf convoy.followers=1 route.enabled=true route.cruise_kmh=10.8'))
    folder.mkdir(parents=True)
    write(config, folder / 'config.yaml')
    (folder / 'run.yaml').write_text(yaml.safe_dump(
        {'reason': 'done', 'duration': 60.0, 'seed': 1, 'commits': {}}))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(folder / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    kinds = {'/pickup/ftl/state': 'nav_msgs/msg/Odometry',
             '/tractor_1/ftl/state': 'nav_msgs/msg/Odometry',
             '/ground_truth/pickup/pose': 'geometry_msgs/msg/PoseStamped',
             '/ground_truth/tractor_1/pose': 'geometry_msgs/msg/PoseStamped',
             '/tractor_1/cmd_vel': 'geometry_msgs/msg/Twist',
             '/experiment/config': 'std_msgs/msg/String'}
    for i, (name, kind) in enumerate(kinds.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(i, name, kind, 'cdr'))
    r = route('citrus_farm_90')
    writer.write('/experiment/config',
                 serialize_message(String(data=(folder / 'config.yaml').read_text())), 0)
    for k in range(600):
        t = 10.0 + 0.1 * k
        stamp = int(t * 1e9)
        for vehicle, robot, s, offset in (('pickup', 'pickup', 20.0 + 0.3 * k, 0.0),
                                          ('tractor_1', 'autonomous_tractor', 12.0 + 0.3 * k,
                                           0.10)):
            ax, ay, yaw = r.at(s)
            ax -= offset * math.sin(yaw)
            ay += offset * math.cos(yaw)
            bx, by = ax - ANTENNA[robot] * math.cos(yaw), ay - ANTENNA[robot] * math.sin(yaw)
            writer.write(f'/{vehicle}/ftl/state',
                         serialize_message(odometry(t, bx, by, yaw, 3.0)), stamp)
            writer.write(f'/ground_truth/{vehicle}/pose',
                         serialize_message(pose(t, bx + 0.20, by, yaw)), stamp)
        command = Twist()
        command.linear.x = 3.0
        writer.write('/tractor_1/cmd_vel', serialize_message(command), stamp)
    del writer
    return folder


def test_analyze_run(tmp_path):
    run = synthetic_run(tmp_path / 'camp' / 'single_120' / '1')
    frame = analyze_run(run)
    assert {'run', 'seed', 'segment', 'kind', 'vehicle', 'metric', 'value'} <= set(frame.columns)
    rows = frame[(frame.vehicle == 'tractor_1') & (frame.segment == 'straight_0')]
    value = dict(zip(rows.metric, rows.value))
    assert value['lateral_rmse'] == pytest.approx(0.10, abs=0.01)
    assert value['estimation_rmse'] == pytest.approx(0.20, abs=0.01)


def test_analyze_command(tmp_path):
    synthetic_run(tmp_path / 'camp' / 'single_120' / '1')
    synthetic_run(tmp_path / 'camp' / 'single_120' / '2')
    out = tmp_path / 'out'
    subprocess.run(['ros2', 'run', 'ftl_analysis', 'analyze', str(tmp_path / 'camp'),
                    '--out', str(out)], check=True, timeout=300)
    for name in ('metrics_runs.csv', 'metrics_summary.csv', 'table_lateral.tex',
                 'lateral_rmse.pdf'):
        assert (out / name).exists(), name
    summary = pd.read_csv(out / 'metrics_summary.csv')
    rows = summary[(summary.vehicle == 'tractor_1') & (summary.metric == 'lateral_rmse')
                   & (summary.segment == 'straight_0')]
    assert len(rows) == 1
    assert rows['mean'].iloc[0] == pytest.approx(0.10, abs=0.01)
    assert {'label', 'n', 'mean', 'ci95'} <= set(summary.columns)
