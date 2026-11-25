"""Sobe um experimento completo a partir de um cenário.

    ros2 launch ftl_bringup experiment.launch.py scenario:=dual_160
    ros2 launch ftl_bringup experiment.launch.py set:="convoy.followers=2 degradation.gnss=dgnss"

Resolve o cenário, grava `<output>/config.yaml`, sobe o comboio no Gazebo, os nós do FTL e o
`apply_scenario`, que publica a configuração e ajusta os nós dos veículos.
"""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ftl_bringup import scenario


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = scenario.resolve(arg('scenario'), scenario.parse_overrides(arg('set')))
    output = Path(arg('output') or
                  f"/workspace/experiments/manual/{datetime.now():%Y%m%d-%H%M%S}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = output / 'config.yaml'
    scenario.write(config, config_path)

    def include(package, name, arguments):
        source = Path(get_package_share_directory(package)) / 'launch' / name
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                        launch_arguments=arguments.items())

    actions = [
        include('agri_gz_bringup', 'convoy.launch.py', scenario.convoy_arguments(config)),
        include('ftl_bringup', 'ftl.launch.py', scenario.ftl_arguments(config)),
        Node(package='ftl_bringup', executable='apply_scenario', name='apply_scenario',
             arguments=[str(config_path)], output='screen'),
    ]
    if config['route']['enabled']:
        leader = config['convoy']['leader']
        actions.append(Node(
            package='ftl_route_driver', executable='route_driver', name='route_driver',
            namespace=leader, output='screen',
            parameters=[scenario.driver_parameters(config)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='single_120',
                              description='Preset in config/scenarios.'),
        DeclareLaunchArgument('set', default_value='',
                              description='Overrides, e.g. "convoy.followers=2 gui=true".'),
        DeclareLaunchArgument('output', default_value='',
                              description='Output folder; empty means a timestamped one.'),
        OpaqueFunction(function=launch_setup),
    ])
