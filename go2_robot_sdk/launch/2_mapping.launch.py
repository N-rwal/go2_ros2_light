# Minimal mapping launch file - optimized for SLAM with external 2D LiDAR
# Usage: ros2 launch go2_robot_sdk mapping_minimal.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate minimal launch description for Go2 mapping with external LiDAR"""

    # Environment variables
    robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    # Package paths
    package_dir = get_package_share_directory('go2_robot_sdk')
    sllidar_package_dir = get_package_share_directory('sllidar_ros2')

    # Essential config files
    config_paths = {
        'slam': os.path.join(package_dir, 'config', 'mapper_params_online_async.yaml'),
        'twist_mux': os.path.join(package_dir, 'config', 'twist_mux.yaml'),
        'sllidar_launch': os.path.join(sllidar_package_dir, 'launch', 'sllidar_s3_launch.py'),
    }

    print("/////////////--------------------------------------------TEST--------------------------/////////////////////")
    print(f"🗺️  Minimal Mapping Configuration:")
    print(f"   Robot IP: {robot_ip}")
    print(f"   Connection: {conn_type}")
    print(f"   Using SLLIDAR S3 for 2D scanning")

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_slam = LaunchConfiguration('slam', default='true')

    launch_args = [
        DeclareLaunchArgument('slam', default_value='true',
                            description='Launch SLAM Toolbox'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
    ]

    # CORE NODES - Only essential components
    core_nodes = [
        # Main robot driver (MINIMAL configuration)
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': robot_token,
                'conn_type': conn_type,
                'enable_video': False,
                'decode_lidar': False,  # Disable internal LiDAR processing
            }],
        ),

        # Teleop capability (twist mux only)
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[
                {'use_sim_time': use_sim_time},
                config_paths['twist_mux']
            ],
        ),

        # Static transform from base_link to laser (for SLLIDAR)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),
    ]

    # SLLIDAR launch - Match old structure EXACTLY
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([config_paths['sllidar_launch']]),
        # NO condition - matches old working code
    )

    # SLAM Toolbox - For map creation (expects /scan topic from SLLIDAR)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py')
        ]),
        condition=IfCondition(with_slam),
        launch_arguments={
            'slam_params_file': config_paths['slam'],
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Combine all components - Match old structure
    return LaunchDescription(
        launch_args +
        core_nodes +
        [slam_launch]  # Combine them in a list
    )
