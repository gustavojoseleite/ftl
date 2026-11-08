"""An FTL test in Gazebo, in one command: the scenario and the FTL nodes.

    ros2 launch ftl_gz ftl_sim.launch.py
    ros2 launch ftl_gz ftl_sim.launch.py followers:=4 params_file:=/path/exp.yaml

sim.launch.py (world + convoy) and ftl_bringup's ftl.launch.py with
use_sim_time on. The simplest way to repeat an experiment; while developing,
running the two apart lets the FTL nodes restart without Gazebo.
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
        # Declared without defaults here: sim.launch.py owns the scenario
        # defaults, and only what is given on the command line is forwarded.
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ftl_bringup"), "config", "ftl.yaml"]),
            description="Parameters of the FTL nodes for this experiment.",
        ),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim)),

        # The convoy names the leader after its vehicle type unless told
        # otherwise, so the leader's namespace is leader:= itself.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ftl),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": LaunchConfiguration("params_file"),
                "leader": LaunchConfiguration("leader", default="pickup"),
            }.items(),
        ),
    ])
