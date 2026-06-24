import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package Directories
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_fusion_bot = get_package_share_directory('fusion_bot')

    # Paths to your custom configuration files
    map_file = os.path.join(pkg_fusion_bot, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_fusion_bot, 'config', 'nav2_params.yaml')

    # Launch Nav2 (AMCL, Planner, Controller, BT Navigator, etc.)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
