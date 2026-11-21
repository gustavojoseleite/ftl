"""Um cenário de teste do FTL no Gazebo, sem os nós do FTL.

    ros2 launch ftl_gz sim.launch.py
    ros2 launch ftl_gz sim.launch.py world:=citrus_farm_60.sdf followers:=2 gui:=false

Sobe o mundo e o comboio, e nada mais. Rode os nós do FTL ao lado, com
`ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true`, que é o que o
projects/ftl/compose.yaml faz em um container próprio, para reiniciá-los sem
reiniciar o Gazebo. O `ftl_sim.launch.py` faz os dois de uma vez.

O cenário é o comboio do agri_gz_bringup (um líder, N seguidores em fila) com os
padrões do FTL: uma pickup puxando tratores pelo carreador principal da fazenda
de citros.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# (argumento, padrão do FTL, descrição). Repassados ao comboio como estão.
SCENARIO = (
    ("world", "citrus_farm.sdf", "World file, relative to agri_gz_worlds/worlds."),
    ("gui", "true", "false runs the simulator without its window."),
    ("leader", "pickup", "Vehicle type in front."),
    ("follower", "autonomous_tractor", "Vehicle type of every follower."),
    ("followers", "2", "How many followers."),
    ("spacing", "10.0", "Metres between one machine's base_link and the next."),
    ("yaw", "0.0", "Heading of the line, radians. 0 is along the origin carreador."),
)


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, desc in SCENARIO),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("agri_gz_bringup"), "launch", "convoy.launch.py",
            ])),
            launch_arguments={name: LaunchConfiguration(name) for name, _, _ in SCENARIO}.items(),
        ),
    ])
