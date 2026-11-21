"""Um teste do FTL no Gazebo, em um comando: o cenário e os nós do FTL.

    ros2 launch ftl_gz ftl_sim.launch.py
    ros2 launch ftl_gz ftl_sim.launch.py followers:=4 params_file:=/path/exp.yaml

O `sim.launch.py` (mundo + comboio) e o `ftl.launch.py` do ftl_bringup com
`use_sim_time` ligado. É o jeito mais simples de repetir um experimento; no
desenvolvimento, rodar os dois separados deixa reiniciar os nós do FTL sem o Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

SIM_ARGS = ("world", "gui", "leader", "follower", "followers", "spacing", "yaw")


def generate_launch_description():
    sim = PathJoinSubstitution([FindPackageShare("ftl_gz"), "launch", "sim.launch.py"])
    ftl = PathJoinSubstitution([FindPackageShare("ftl_bringup"), "launch", "ftl.launch.py"])

    return LaunchDescription([
        # Declarados sem padrão aqui: o `sim.launch.py` é dono dos padrões do
        # cenário, e só o que vem na linha de comando é repassado.
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ftl_bringup"), "config", "ftl.yaml"]),
            description="Parameters of the FTL nodes for this experiment.",
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim)),

        # O comboio dá ao líder o nome do tipo de veículo, a menos que se diga
        # outro, então o namespace do líder é o próprio `leader:=`.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ftl),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": LaunchConfiguration("params_file"),
                "leader": LaunchConfiguration("leader", default="pickup"),
                "followers": LaunchConfiguration("followers", default="2"),
            }.items(),
        ),
    ])
