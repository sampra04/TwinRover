import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_description = get_package_share_directory('twinrover_description')
    pkg_slam        = get_package_share_directory('slam_toolbox')
    pkg_sick        = get_package_share_directory('sick_scan_xd')

    urdf_path        = os.path.join(pkg_description, 'urdf', 'myrobot_real.urdf')
    slam_params_path = os.path.join(pkg_description, 'config', 'slam_toolbox_params.yaml')
    ekf_params_path  = os.path.join(pkg_description, 'config', 'ekf.yaml')
    ros2_control_params = os.path.join(pkg_description, 'config', 'ros2_control.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    sick_lidar = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim_5xx',
        output='screen',
        arguments=[
            os.path.join(pkg_sick, 'launch', 'sick_tim_5xx.launch'),
            'hostname:=192.168.137.162',
            'frame_id:=lidar_link',
            'tf_publish_rate:=0',
            'time_increment:=0.000185185',
            'range_max:=10.0',
            'range_min:=0.1',
            'min_ang:=-2.2689',   # -130 degrees in radians
            'max_ang:=2.2689',    # +130 degrees in radians
            'use_generation_timestamp:=0',  # 0=false: use ROS reception time, not sensor clock — prevents backwards timestamps that crash slam_toolbox
        ]
    )

    # ros2_control controller manager — replaces roboclaw node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_params,
        ],
        output='screen',
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        remappings=[('/diff_drive_controller/cmd_vel', '/cmd_vel')],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path],
    )

    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp',
        output='screen',
        remappings=[('pointcloud_topic', '/cloud')],
    )

    twist_to_stamped = Node(
        package='twinrover_description',
        executable='twist_to_stamped.py',
        name='twist_to_stamped',
        output='screen',
    )

    stall_guard = Node(
        package='twinrover_description',
        executable='stall_guard.py',
        name='stall_guard',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odometry/filtered',
            'cmd_threshold': 0.05,
            'motion_threshold': 0.03,
            'stall_duration_s': 3.0,
            'recovery_duration_s': 3.0,
            'check_rate_hz': 10.0,
        }],
    )

    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'slam_params_file': slam_params_path,
                    'use_sim_time':     'false',
                }.items()
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        sick_lidar,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
        ekf_node,
        kiss_icp_node,
        twist_to_stamped,
        stall_guard,
        slam_toolbox,
    ])
