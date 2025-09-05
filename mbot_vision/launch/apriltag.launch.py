from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution([
        FindPackageShare('mbot_vision'),
        'config',
        'camera.yaml'
    ])
    
    apriltag_config = PathJoinSubstitution([
        FindPackageShare('mbot_vision'),
        'config',
        'tags_custom48h12.yaml'
    ])

    # 1. Define the camera node (the root of the pipeline)
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        parameters=[camera_config],
        output='screen'
    )
    
    # 2. Define the image rectification node
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
    
    # 3. Define the AprilTag detection node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[apriltag_config],
        remappings=[
            ('image_rect', '/image_rect'),
            ('camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,

        # This outer handler waits for the camera_node to start
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=camera_node,
                on_start=[
                    rectify_node,
                    
                    # This inner handler waits for the rectify_node to start
                    RegisterEventHandler(
                        event_handler=OnProcessStart(
                            target_action=rectify_node,
                            on_start=[apriltag_node]
                        )
                    )
                ]
            )
        )
    ])