#!/usr/bin/env python3
"""
Standalone launch file for Gazebo with MBot.
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
    robot_name = 'mbot'
    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mbot_description = get_package_share_directory('mbot_description')
    
    # Launch Arguments
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # Robot Description
    urdf_file = os.path.join(pkg_mbot_description, 'urdf', 'mbot_classic.urdf.xacro')
    
    # Use xacro to process the file
    robot_desc = xacro.process_file(urdf_file).toxml()
    
    # Launch Arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_mbot_description, 'worlds', 'maze.sdf'),
        description='Full path to the world model file to load'
    )
    
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x-position of the robot'
    )
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y-position of the robot'
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # 'gz_args': ['-r -v4 ', world], # verbose mode
            'gz_args': ['-r ', world],
            'exit_on_shutdown': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mbot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0'
        ],
        output='screen'
    )

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_ros_bridge',
        output='screen',
        arguments=[
            # Core Gazebo topics
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # Odometry 
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # TF
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # CMD Vel (bidirectional)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Sensor data
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ]
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridges
    ]) 