from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('twinrover_gazebo')

    # 🔹 NEW WORLD
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    # 🔹 SAME ROBOT
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    return LaunchDescription([

        # Start Gazebo with warehouse world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read(),
                'use_sim_time': True
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.2'
            ],
            output='screen'
        ),
    ])
