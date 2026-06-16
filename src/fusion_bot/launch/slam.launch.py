import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
import launch.events

def generate_launch_description():
    pkg_name = "fusion_bot"

    # Resolve the path to the SLAM configurations
    slam_config_path = os.path.join(
        get_package_share_directory(pkg_name), "config", "slam_toolbox.yaml"
    )

    # Use LifecycleNode to declare the SLAM node
    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_config_path,
            {"use_sim_time": True}
        ]
    )

    # 1. Automatically emit CONFIGURE event on startup
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # 2. Automatically emit ACTIVATE event once the node reaches the 'inactive' state
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        slam_node,
        configure_event,
        activate_event
    ])
