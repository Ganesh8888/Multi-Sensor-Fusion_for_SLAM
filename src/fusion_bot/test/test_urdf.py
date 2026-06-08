import os
import subprocess
import pytest
from ament_index_python.packages import get_package_share_directory

def test_urdf_xacro_compilation():
    """
    Test that the primary robot.urdf.xacro file compiles without errors.
    This catches math errors (like divide-by-zero inertia) and XML syntax errors.
    """
    pkg_share = get_package_share_directory('fusion_bot')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    assert os.path.exists(xacro_file), f"Xacro file not found at {xacro_file}"

    # Run the xacro compiler as a subprocess
    result = subprocess.run(
        ['xacro', xacro_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # If xacro fails (e.g. syntax error or math error), it returns a non-zero exit code
    assert result.returncode == 0, f"Xacro compilation failed!\nStderr:\n{result.stderr}"
    
    # Ensure the output actually contains some XML
    assert "<robot" in result.stdout, "Xacro compiled, but output does not contain <robot> tag!"
    print("URDF compiled successfully.")
