from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    # Path to the parameter file can be overridden by the caller
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('mbot_navigation'),
            'config',
            'slam_toolbox_localization.yaml'),
        description='Full path to the slam_toolbox parameters file'
    )

    # Start slam_toolbox in localization mode (lifecycle node)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # Immediately configure the node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # Activate once it reaches the inactive state
    activate_event = OnStateTransition(
        target_lifecycle_node=slam_toolbox_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_ACTIVATE
        ))]
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file)
    ld.add_action(slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_event_handler(activate_event)

    return ld
