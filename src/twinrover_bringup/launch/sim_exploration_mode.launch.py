import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('twinrover_bringup')
    exploration_share = get_package_share_directory('twinrover_exploration')

    exploration_params_default = os.path.join(exploration_share, 'config', 'exploration.yaml')

    exploration_params = LaunchConfiguration('exploration_params_file')

    sim_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_mapping_mode.launch.py')
        )
    )

    explorer_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_share, 'launch', 'explorer_nodes.launch.py')
        ),
        launch_arguments={'exploration_params': exploration_params}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('exploration_params_file', default_value=exploration_params_default),
        sim_mapping,
        explorer_nodes,
    ])
