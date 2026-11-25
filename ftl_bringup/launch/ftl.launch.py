"""Os nós do FTL, com simulador ou sem.

    ros2 launch ftl_bringup ftl.launch.py                       # máquinas reais
    ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true    # contra o Gazebo
    ros2 launch ftl_bringup ftl.launch.py followers:=4 params_file:=/path/exp.yaml
    ros2 launch ftl_bringup ftl.launch.py localization:=ukf     # UKF no lugar do pose_estimator
    ros2 launch ftl_bringup ftl.launch.py follower:=trail       # trilho único do ftl_trajectory

O que roda:

    todo veículo       pose_estimator   GNSS + IMU + odometria -> ftl/state, referencial comum
                       (com `localization:=ukf`: o UKF e o gnss_odometry do ftl_localization,
                       que publicam o mesmo ftl/state, com covariância)
    o líder            road_follow      dirige pelos carreadores usando a câmera
    cada seguidor      follower         segue o caminho do veículo da frente a `gap`
                       (com `follower:=trail`: o trail_follower do ftl_trajectory segue o
                       trilho único que o trail_publisher do primeiro trator publica)

A fila é líder, <prefixo>_1, <prefixo>_2, ... e cada seguidor segue o que vai à
sua frente: tractor_1 segue a pickup, tractor_2 segue tractor_1. Esses nomes são
os namespaces dos veículos, os mesmos que o comboio cria.

Nada aqui cria um veículo nem sabe que existe um simulador. Cada nó fala com um
veículo só pelos tópicos do namespace dele (cmd_vel, odom, imu/data, gnss/fix,
camera_front/*) e pelo seu TF, então o mesmo launch conduz um comboio no Gazebo
(o ftl_gz o sobe) ou na fazenda. `use_sim_time` é o único interruptor.

TF: cada veículo mantém sua árvore no próprio namespace (ver o módulo vehicles do
agri_gz_bringup), então um nó que lê TF recebe os mesmos dois remaps.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    leader = arg("leader").strip("/")
    followers = [f"{arg('follower_prefix')}_{n + 1}" for n in range(int(arg("followers")))]
    common = [arg("params_file"), {"use_sim_time": arg("use_sim_time").lower() == "true"}]

    nodes = [
        Node(package="road_follow", executable="road_follow", name="road_follow",
             namespace=leader, parameters=common, remappings=TF_REMAPS, output="screen"),
    ]
    localization = arg("localization")
    if localization not in ("pose_estimator", "ukf"):
        raise ValueError(f"localization: {localization!r} is not pose_estimator or ukf")
    for ns in [leader, *followers]:
        if localization == "ukf":
            source = Path(get_package_share_directory("ftl_localization")) / "launch" / \
                "localization.launch.py"
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(source)),
                launch_arguments={"namespace": ns,
                                  "use_sim_time": arg("use_sim_time").lower()}.items()))
        else:
            nodes.append(Node(package="ftl_follower_pure_pursuit", executable="pose_estimator",
                              name="pose_estimator", namespace=ns, parameters=common,
                              remappings=TF_REMAPS, output="screen"))
    follower = arg("follower")
    if follower not in ("pure_pursuit", "trail"):
        raise ValueError(f"follower: {follower!r} is not pure_pursuit or trail")
    if follower == "trail" and followers:
        # O primeiro trator percebe o líder e publica o trilho único, suavizado uma vez (D9).
        nodes.append(Node(package="ftl_trajectory", executable="trail_publisher",
                          name="trail_publisher", namespace=followers[0],
                          parameters=[*common, {"leader": f"/{leader}"}], output="screen"))
    for ahead, ns in zip([leader, *followers], followers):
        if follower == "trail":
            nodes.append(Node(package="ftl_trajectory", executable="trail_follower",
                              name="follower", namespace=ns,
                              parameters=[*common, {"ahead": f"/{ahead}",
                                                    "trail": f"/{followers[0]}/ftl/trail"}],
                              output="screen"))
        else:
            nodes.append(Node(package="ftl_follower_pure_pursuit", executable="follower",
                              name="follower", namespace=ns,
                              parameters=[*common, {"ahead": f"/{ahead}"}], output="screen"))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("leader", default_value="pickup",
                              description="Namespace of the leader (its name in the convoy)."),
        DeclareLaunchArgument("followers", default_value="2",
                              description="How many followers, named <follower_prefix>_1.._n."),
        DeclareLaunchArgument("follower_prefix", default_value="tractor",
                              description="Followers' namespaces are <prefix>_1, <prefix>_2, ..."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when the clock comes from the simulator."),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ftl_bringup"), "config", "ftl.yaml"]),
            description="Parameters of the FTL nodes for this experiment.",
        ),
        DeclareLaunchArgument("localization", default_value="pose_estimator",
                              description="pose_estimator (B0) or ukf (ftl_localization)."),
        DeclareLaunchArgument("follower", default_value="pure_pursuit",
                              description="pure_pursuit (B0, trail of the vehicle ahead) or "
                                          "trail (single trail from ftl_trajectory)."),
        OpaqueFunction(function=launch_setup),
    ])
