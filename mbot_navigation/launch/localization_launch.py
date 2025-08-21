import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # -----  File Paths -----
    pkg_share = get_package_share_directory('mbot_navigation')
    # Path to the AMCL parameters file
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    # Path to the RViz2 configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'localization.rviz')

    # -----  Launch Arguments -----
    # use_sim_time is false for a real robot
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Path to map file
    map_file = PathJoinSubstitution([
        get_package_share_directory('mbot_navigation'),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    return LaunchDescription([
        # -----  Launch Arguments -----
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map_name',
            description='Name of the map file (without .yaml extension) - REQUIRED'),

        # -----  Nodes -----
        # 1. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}]
        ),

        # 2. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params_file,
                        {'use_sim_time': use_sim_time}]
        ),

        # 3. Lifecycle Manager
        # This node will manage the startup and shutdown of the map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        # The list of nodes to manage
                        {'node_names': ['map_server', 'amcl']}]
        ),
        
        # 4. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])