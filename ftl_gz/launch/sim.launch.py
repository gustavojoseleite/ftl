"""An FTL test scenario in Gazebo, without the FTL nodes.

    ros2 launch ftl_gz sim.launch.py
    ros2 launch ftl_gz sim.launch.py world:=citrus_farm_60.sdf followers:=2 gui:=false

Brings up the world and the convoy, and nothing else. Run the FTL nodes beside
it with `ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true`, which is
what projects/ftl/compose.yaml does in its own container so they can be
restarted without restarting Gazebo. ftl_sim.launch.py does both at once.

The scenario is the convoy of agri_gz_bringup (one leader, N followers in a
line) with FTL's defaults: a pickup leading tractors down the main carreador
of the citrus farm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# (argument, FTL default, description). Forwarded to the convoy as they are.
SCENARIO = (
    ("world", "citrus_farm.sdf", "World file, relative to agri_gz_worlds/worlds."),
    ("gui", "true", "false runs the simulator without its window."),
    ("control", "ros2_control", "ros2_control or plugin: how the vehicles are driven."),
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
