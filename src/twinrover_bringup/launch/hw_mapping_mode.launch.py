import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('twinrover_navigation')
    bringup_share = get_package_share_directory('twinrover_bringup')

    slam_params_default = os.path.join(navigation_share, 'config', 'slam_toolbox_online.yaml')
    nav2_params_default = os.path.join(navigation_share, 'config', 'nav2_explore.yaml')

    slam_params = LaunchConfiguration('slam_params_file')
    nav2_params = LaunchConfiguration('nav2_params_file')
    sensor_launch_file = LaunchConfiguration('sensor_launch_file')
    use_sensor_launch = LaunchConfiguration('use_sensor_launch')
    sensor_launch_default = os.path.join(bringup_share, 'launch', 'noop.launch.py')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_file),
        condition=IfCondition(use_sensor_launch),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params,
        }.items(),
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=slam_params_default),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_default),
        DeclareLaunchArgument('use_sensor_launch', default_value='false'),
        DeclareLaunchArgument('sensor_launch_file', default_value=sensor_launch_default),
        sensor_launch,
        slam_launch,
        nav_launch,
        map_saver_server,
    ])
