from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('twinrover_gazebo')

    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    return LaunchDescription([

        # Start Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path, 'r').read(),
                'use_sim_time': True
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'my_robot',
                '-x', '0',
                '-y', '0',
                '-z', '0.2'
            ],
            output='screen'
        ),
    ])
