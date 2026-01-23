import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    # Environment variables
    robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
    robot_token = os.getenv('ROBOT_TOKEN', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    
    # Package paths
    package_dir = get_package_share_directory('go2_robot_sdk')
    sllidar_package_dir = get_package_share_directory('sllidar_ros2')
    
    # Essential config files
    config_paths = {
        'custom_nav_launch': os.path.join(package_dir, 'launch', 'custom_navigation_launch.py'),
        'nav2_params': os.path.join(package_dir, 'config', 'nav2_params_navigation.yaml'),
        'twist_mux': os.path.join(package_dir, 'config', 'twist_mux.yaml'),
        'default_map_yaml': os.path.join(package_dir, 'map', 'office.yaml'),
        'sllidar_launch': os.path.join(sllidar_package_dir, 'launch', 'sllidar_s3_launch.py'),
    }
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_nav2 = LaunchConfiguration('nav2', default='true')
    map_yaml = LaunchConfiguration('map', default=config_paths['default_map_yaml'])
    params_file = LaunchConfiguration('params_file', default=config_paths['nav2_params'])
    
    launch_args = [
        DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=config_paths['default_map_yaml'], 
                            description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=config_paths['nav2_params'],
                            description='Full path to Nav2 parameters file'),
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
        
        # TF from base_link to laser (adjust height 0.2m as needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        
        # Initial map to odom transform (AMCL will update this)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
    ]
    
    # SLLIDAR launch (with 10-second delay - good for hardware initialization)
    lidar_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([config_paths['sllidar_launch']]),
            )
        ]
    )
    
    # CUSTOM NAVIGATION LAUNCH (combines localization + navigation)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([config_paths['custom_nav_launch']]),
        condition=IfCondition(with_nav2),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Initial pose publisher (delayed to ensure AMCL is ready)
    initial_pose_timer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='go2_robot_sdk',
                executable='initial_pose_pub',
                name='initial_pose_pub',
                output='screen'
            )
        ]
    )
    
    # Combine all components
    return LaunchDescription(
        launch_args +
        core_nodes +
        [lidar_launch, nav_launch, initial_pose_timer]
    )
