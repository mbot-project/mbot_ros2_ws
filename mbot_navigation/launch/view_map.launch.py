import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch argument for map file (required, no default)
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        description='Name of the map file (without .yaml extension) - REQUIRED'
    )
    
    # Path to your map file
    map_file_path = PathJoinSubstitution([
        get_package_share_directory('mbot_navigation'),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])
    
    # Path to RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('mbot_navigation'),
        'rviz',
        'view_map.rviz'
    )

    # Node for the map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path,
                    'use_sim_time': False}]
    )

    # Node for the lifecycle manager
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server']}]
    )
    

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        map_name_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node,
    ])