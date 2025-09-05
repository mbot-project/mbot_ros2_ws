#!/usr/bin/env python3
"""
Launches the core nodes for the MBot, including robot description and lidar drivers.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ----- Package Directories -----
    pkg_mbot_description = get_package_share_directory('mbot_description')
    pkg_sllidar_ros2 = get_package_share_directory('sllidar_ros2') 
    
    # ----- Launch Arguments -----
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ----- Robot Description -----
    urdf_file = os.path.join(pkg_mbot_description, 'urdf', 'mbot_classic.urdf.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()

    # ----- Node & Launch File Definitions -----
    
    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # 2. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Lidar Launch File Inclusion
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sllidar_ros2, 'launch', 'sllidar_a1_launch.py')
        ),
        # Pass the frame_id argument to the included launch file
        launch_arguments={'frame_id': 'lidar_link'}.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock instead of system clock'),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        lidar_launch
    ])