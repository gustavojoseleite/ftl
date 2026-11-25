"""
Sobe a localização de um veículo: o `gnss_odometry` e o UKF no namespace dele.

    ros2 launch ftl_localization localization.launch.py namespace:=pickup use_sim_time:=true

O `gnss_odometry` converte o `gnss/fix` na posição da base em `odometry/gps` (e reinicia o filtro
pelo `set_pose` na partida); o `ukf` funde isso com a odometria e a IMU em `ftl/state_filter`, que
o `gnss_odometry` repassa ao `ftl/state` depois que o filtro chega ao GNSS.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('ftl_localization'), 'config',
                                   'localization.yaml'])
    common = [params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Namespace do veículo.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true quando o relógio vem do simulador.'),
        Node(package='ftl_localization', executable='gnss_odometry', name='gnss_odometry',
             namespace=LaunchConfiguration('namespace'), parameters=common, output='screen',
             remappings=tf),
        Node(package='robot_localization', executable='ukf_node', name='ukf',
             namespace=LaunchConfiguration('namespace'), parameters=common, output='screen',
             remappings=[*tf, ('odometry/filtered', 'ftl/state_filter')]),
    ])
