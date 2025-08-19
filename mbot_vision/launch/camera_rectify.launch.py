from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to camera config file
    camera_config = PathJoinSubstitution([
        FindPackageShare('mbot_vision'),
        'config',
        'camera.yaml'
    ])

    return LaunchDescription([
        # Camera node
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[camera_config],
            output='screen'
        ),
        
        # Image rectification node
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            remappings=[
                ('image', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info')
            ],
            output='screen'
        )
    ])