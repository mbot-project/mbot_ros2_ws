"""Launch Nav2 stack for MBot using a saved map and param file.

Usage:
ros2 launch mbot_navigation nav2_navigation.launch.py \
     map:=/home/mbot/mbot_ws/src/mbot_navigation/maps/office.yaml
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Inputs
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to the map yaml file to load (required)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('mbot_navigation'),
            'config',
            'nav2_params.yaml'),
        description='Full path to the nav2 parameters file')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(navigation_launch)

    return ld
