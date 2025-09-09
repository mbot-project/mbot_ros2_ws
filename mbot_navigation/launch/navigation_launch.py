import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # -----  File Paths -----
    pkg_share = get_package_share_directory('mbot_navigation')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'navigation.rviz')

    return LaunchDescription([
        # -----  Launch Arguments -----
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'map_name',
            description='Name of the map file (without .yaml extension) - REQUIRED'
        ),

        # -----  Nodes -----
        # 1. Launch the localization stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map_name': LaunchConfiguration('map_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'launch_rviz': 'false',
            }.items()
        ),

        # 2. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
        ),

        # 3. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
        ),

        # 4. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file],
        ),
        
        # 5. Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file],
        ),
        
        # 6. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
        ),

        # 7. Lifecycle Manager to manage the Nav2 servers
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        # The list of nodes to manage
                        {'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower'
                        ]}
            ]
        ),

        # 9. RViz2 with navigation configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
