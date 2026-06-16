import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="visual_odometry",
                output="screen",
                parameters=[
                    {
                        "frame_id": "base_link",  # Estimate motion of base_link
                        "odom_frame_id": "odom_visual",  # Publish to a unique visual odom frame
                        "publish_tf": False,  # Set False to avoid conflict with wheel odom TF
                        "use_sim_time": True,
                        "approx_sync": True,  # Sync camera image and depth image approximately
                        "queue_size": 10,
                    }
                ],
                remappings=[
                    ("rgb/image", "/camera/image"),
                    ("depth/image", "/camera/depth_image"),
                    ("rgb/camera_info", "/camera/camera_info"),
                    (
                        "odom",
                        "/visual_odom",
                    ),  # Remap standard /odom output to /visual_odom
                ],
            )
        ]
    )
