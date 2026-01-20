# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


class Go2LaunchConfig:
    """Configuration container for Go2 robot launch parameters"""
    
    def __init__(self):
        # Environment variables
        self.robot_token = os.getenv('ROBOT_TOKEN', '')
        self.robot_ip = os.getenv('ROBOT_IP', '192.168.123.161')
        self.conn_type = os.getenv('CONN_TYPE', 'webrtc')
        
        # Derived configurations
        self.rviz_config = "single_robot_conf.rviz"
        
        # Package paths
        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()
        
        print(f"🚀 Go2 Launch Configuration (2D LiDAR mode):")
        print(f"   Robot IP: {self.robot_ip}")
        print(f"   Connection: {self.conn_type}")
        print(f"   LiDAR Mode: Simple 2D (scan topic)")
    
    def _get_config_paths(self) -> dict:
        """Get all configuration file paths"""
        return {
            'twist_mux': os.path.join(self.package_dir, 'config', 'twist_mux.yaml'),
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'nav2': os.path.join(self.package_dir, 'config', 'nav2_params.yaml'),
            'rviz': os.path.join(self.package_dir, 'config', self.rviz_config),
        }


class Go2NodeFactory:
    """Factory for creating Go2 robot nodes for 2D LiDAR setup"""
    
    def __init__(self, config: Go2LaunchConfig):
        self.config = config
    
    def create_launch_arguments(self) -> List[DeclareLaunchArgument]:
        """Create all launch arguments"""
        return [
            DeclareLaunchArgument('rviz2', default_value='true', description='Launch RViz2'),
            DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2'),
            DeclareLaunchArgument('slam', default_value='true', description='Launch SLAM'),
            DeclareLaunchArgument('teleop', default_value='true', description='Launch teleoperation'),
            DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        ]
    
    def create_core_nodes(self) -> List[Node]:
        """Create core Go2 robot nodes with simple LiDAR support"""
        return [
            # Main robot driver (assumed to publish /scan directly)
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                name='go2_driver_node',
                output='log',
                parameters=[{
                    'robot_ip': self.config.robot_ip,
                    'token': self.config.robot_token,
                    'conn_type': self.config.conn_type,
                    'enable_video': False,
                    'decode_lidar': True,  # Assuming driver provides /scan
                }],
                remappings=[
                    # If needed, remap the LiDAR topic to standard /scan
                    # ('original_lidar_topic', 'scan'),
                ],
            ),
        ]
    
    def create_teleop_nodes(self) -> List[Node]:
        """Create teleoperation nodes"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_teleop = LaunchConfiguration('teleop', default='true')
        
        return [
            # Twist multiplexer (for teleop capability)
            Node(
                package='twist_mux',
                executable='twist_mux',
                output='screen',
                condition=IfCondition(with_teleop),
                parameters=[
                    {'use_sim_time': use_sim_time},
                    self.config.config_paths['twist_mux']
                ],
            ),
            # Optional: Teleop node if your robot needs one
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_keyboard',
                condition=IfCondition(with_teleop),
                output='screen',
                prefix='xterm -e',
            ),
        ]
    
    def create_visualization_nodes(self) -> List[Node]:
        """Create visualization nodes (RViz only)"""
        with_rviz2 = LaunchConfiguration('rviz2', default='true')
        
        return [
            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                condition=IfCondition(with_rviz2),
                name='go2_rviz2',
                output='screen',
                arguments=['-d', self.config.config_paths['rviz']],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            ),
        ]
    
    def create_include_launches(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions for SLAM and Navigation"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_slam = LaunchConfiguration('slam', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        
        return [
            # SLAM Toolbox (for 2D LiDAR)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=IfCondition(with_slam),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # Nav2 (for 2D navigation)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ]),
                condition=IfCondition(with_nav2),
                launch_arguments={
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]


def generate_launch_description():
    """Generate the launch description for Go2 robot system with 2D LiDAR"""
    
    # Initialize configuration and factory
    config = Go2LaunchConfig()
    factory = Go2NodeFactory(config)
    
    # Create all components
    launch_args = factory.create_launch_arguments()
    core_nodes = factory.create_core_nodes()
    teleop_nodes = factory.create_teleop_nodes()
    visualization_nodes = factory.create_visualization_nodes()
    include_launches = factory.create_include_launches()
    
    # Combine all elements
    launch_entities = (
        launch_args +
        core_nodes +
        teleop_nodes +
        visualization_nodes +
        include_launches
    )
    
    return LaunchDescription(launch_entities)
