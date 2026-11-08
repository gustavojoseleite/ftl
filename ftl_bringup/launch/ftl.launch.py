"""The FTL nodes, simulator or not.

    ros2 launch ftl_bringup ftl.launch.py                       # real machines
    ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true    # against Gazebo
    ros2 launch ftl_bringup ftl.launch.py leader:=pickup params_file:=/path/exp.yaml

What runs today: road_follow drives the leader down the carreadores. The
follower nodes, when they exist, start here too, one per follower namespace.

Nothing here spawns a vehicle or knows a simulator exists. Every node talks to
a vehicle only through the topics in its namespace (cmd_vel, odom, gnss/fix,
camera_front/*) and its TF, so the same launch drives a convoy in Gazebo
(ftl_gz brings that up) or on the farm. use_sim_time is the only switch.

TF: each vehicle keeps its tree in its own namespace (see agro_gz_bringup's
vehicles module), so a node reading TF takes the same two remaps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "leader", default_value="pickup",
            description="ROS namespace of the leader (its name in the convoy).",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="true when the clock comes from the simulator.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ftl_bringup"), "config", "ftl.yaml"]),
            description="Parameters of the FTL nodes for this experiment.",
        ),

        Node(
            package="road_follow",
            executable="road_follow",
            name="road_follow",
            namespace=LaunchConfiguration("leader"),
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=TF_REMAPS,
            output="screen",
        ),
    ])
