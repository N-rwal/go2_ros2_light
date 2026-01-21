import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate minimal launch description for Go2 navigation with external LiDAR"""
    
    # Environment variables
    robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    
    # Package paths
    package_dir = get_package_share_directory('go2_robot_sdk')
    sllidar_package_dir = get_package_share_directory('sllidar_ros2')
    
    # Hardcoded map location (your preferred path)
    map_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'go2_ros2_light', 'go2_robot_sdk', 'map')
    default_map_yaml = os.path.join(map_dir, 'office2.yaml')
    
    # Essential config files
    config_paths = {
        'nav2': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
        'twist_mux': os.path.join(package_dir, 'config', 'twist_mux.yaml'),
        'sllidar_launch': os.path.join(sllidar_package_dir, 'launch', 'sllidar_s3_launch.py'),
    }
    
    print(f"Minimal navigation stack")
    print(f"Map location: {default_map_yaml}")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_nav2 = LaunchConfiguration('nav2', default='true')
    
    launch_args = [
        DeclareLaunchArgument('nav2', default_value='true',
                            description='Launch Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
    ]
    
    # CORE NODES
    core_nodes = [
        # Main robot driver
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
                'decode_lidar': False,
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
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
        ),
    ]
    
    # SLLIDAR launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([config_paths['sllidar_launch']]),
    )
    
    # NAV2
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'localization_launch.py')
        ]),
        condition=IfCondition(with_nav2),
        launch_arguments={
            'map': default_map_yaml,
            'params_file': config_paths['nav2'],
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    # NAV2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py')
        ]),
        condition=IfCondition(with_nav2),
        launch_arguments={
            'params_file': config_paths['nav2'],
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    return LaunchDescription(
        launch_args +
        core_nodes +
        [lidar_launch, localization_launch, navigation_launch]
    )
