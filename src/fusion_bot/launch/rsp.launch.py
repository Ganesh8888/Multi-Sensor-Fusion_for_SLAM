import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Locate the xacro file inside our installed package
    pkg_name = "fusion_bot"
    file_subpath = "description/robot.urdf.xacro"
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    # 2. Run the xacro command to convert macros into raw XML URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Create the Robot State Publisher node, feeding it the XML string
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # 4. Return the launch description
    return LaunchDescription([node_robot_state_publisher])
