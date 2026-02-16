import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('twinrover_exploration'),
        'config',
        'exploration.yaml',
    )
    exploration_params = LaunchConfiguration('exploration_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'exploration_params',
            default_value=default_params,
            description='Path to exploration parameter YAML.',
        ),
        Node(
            package='twinrover_exploration',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[exploration_params],
        ),
        Node(
            package='twinrover_exploration',
            executable='map_saver_periodic',
            name='map_saver_periodic',
            output='screen',
            parameters=[exploration_params],
        ),
        Node(
            package='twinrover_exploration',
            executable='status_monitor',
            name='status_monitor',
            output='screen',
            parameters=[exploration_params],
        ),
        Node(
            package='twinrover_exploration',
            executable='teleop_override',
            name='teleop_override',
            output='screen',
            parameters=[exploration_params],
        ),
    ])
