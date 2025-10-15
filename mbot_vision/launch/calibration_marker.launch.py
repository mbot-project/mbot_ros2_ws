#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mbot_vision',
            executable='calibration_marker',
            name='calibration_marker_publisher',
            output='screen'
        )
    ])