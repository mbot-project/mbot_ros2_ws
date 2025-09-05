from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
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

    # Define the camera node
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[camera_config],
        output='screen'
    )

    # Define the rectification node
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        remappings=[
            ('image', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        
        # This event handler waits for the camera_node process to start
        # and then launches the rectify_node.
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=camera_node,
                on_start=[rectify_node]
            )
        )
    ])