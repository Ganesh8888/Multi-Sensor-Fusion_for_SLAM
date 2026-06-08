import os
import unittest
import pytest
import launch
import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

@pytest.mark.launch_test
def generate_test_description():
    """
    Launch the sim.launch.py file and monitor it.
    """
    pkg_share = get_package_share_directory('fusion_bot')
    launch_file = os.path.join(pkg_share, 'launch', 'sim.launch.py')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file])
    )

    return launch.LaunchDescription([
        sim_launch,
        launch_testing.actions.ReadyToTest()
    ])

class TestSimLaunch(unittest.TestCase):
    def test_no_crashes(self, proc_info, proc_output):
        """
        Wait for a few seconds to ensure that no essential nodes have crashed.
        Gazebo, ros2_control, and the bridges take a few seconds to boot up.
        """
        # Wait for 10 seconds to allow everything to boot and stabilize
        proc_output.assertWaitFor('Robot initialized', timeout=15)
        
        # If we reached here without Gazebo crashing, the test passes.
        # launch_testing will automatically check for dirty exit codes upon shutdown.
