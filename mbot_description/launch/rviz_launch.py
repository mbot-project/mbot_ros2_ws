#!/usr/bin/env python3
"""
Combined launch file for visualization with RViz.
This launches both the robot state publisher and RViz together.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package directories
    pkg_mbot_description = get_package_share_directory('mbot_description')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot Description
    urdf_file = os.path.join(pkg_mbot_description, 'urdf', 'mbot_classic.urdf.xacro')
    # Use xacro to process the file
    robot_desc = xacro.process_file(urdf_file).toxml()

    # RViz configuration file
    rviz_config = os.path.join(pkg_mbot_description, 'rviz', 'mbot.rviz')
    
    # Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock instead of system clock')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Joint State Publisher (non-GUI version)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ]) 