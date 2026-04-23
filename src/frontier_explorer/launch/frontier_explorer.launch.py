import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('frontier_explorer'),
        'config',
        'exploration_hardened.yaml',
    )
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to exploration parameter file.',
        ),
        Node(
            package='frontier_explorer',
            executable='explore',
            name='frontier_explorer',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='frontier_explorer',
            executable='map_saver',
            name='map_saver',
            output='screen',
            parameters=[params_file],
        ),
    ])
