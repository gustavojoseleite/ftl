"""The FTL nodes, simulator or not.

    ros2 launch ftl_bringup ftl.launch.py                       # real machines
    ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true    # against Gazebo
    ros2 launch ftl_bringup ftl.launch.py followers:=4 params_file:=/path/exp.yaml

What runs:

    every vehicle      pose_estimator   GNSS + IMU + odometry -> ftl/state, shared frame
    the leader         road_follow      drives the carreadores by its camera
    each follower      follower         drives the path of the vehicle ahead at `gap`

The line is leader, <prefix>_1, <prefix>_2, ... and each follower follows the
one in front of it: tractor_1 follows the pickup, tractor_2 follows tractor_1.
These names are the vehicles' namespaces, the same ones the convoy spawns.

Nothing here spawns a vehicle or knows a simulator exists. Every node talks to
a vehicle only through the topics in its namespace (cmd_vel, odom, imu/data,
gnss/fix, camera_front/*) and its TF, so the same launch drives a convoy in
Gazebo (ftl_gz brings that up) or on the farm. use_sim_time is the only switch.

TF: each vehicle keeps its tree in its own namespace (see agri_gz_bringup's
vehicles module), so a node reading TF takes the same two remaps.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    for ns in [leader, *followers]:
        nodes.append(Node(package="ftl_follower", executable="pose_estimator", name="pose_estimator",
                          namespace=ns, parameters=common, remappings=TF_REMAPS, output="screen"))
    for ahead, ns in zip([leader, *followers], followers):
        nodes.append(Node(package="ftl_follower", executable="follower", name="follower",
                          namespace=ns, parameters=[*common, {"ahead": f"/{ahead}"}], output="screen"))
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
        OpaqueFunction(function=launch_setup),
    ])
