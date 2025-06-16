# ~/Nevil-picar-v2/src/nevil_bringup/package_finder.py
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution, PathJoinSubstitution

def get_python_node_executable_path(package_name: str, node_file_name_with_py_extension: str) -> PathJoinSubstitution:
    """
    Constructs the full PathJoinSubstitution for a Python node executable
    when it's installed in the ROS 2 'lib/pythonX.Y/site-packages' structure.

    Args:
        package_name (str): The name of the ROS package the node belongs to (e.g., 'nevil_interfaces_ai').
        node_file_name_with_py_extension (str): The name of the Python script file (e.g., 'ai_interface_node.py').

    Returns:
        PathJoinSubstitution: A Launch Substitution that resolves to the full path.
    """
    # Based on your previous output, Python 3.11 is used in your env.
    # Adjust 'python3.11' below if your environment is different.
    python_version_path = 'python3.11'

    # Get the package prefix (e.g., /home/dan/Nevil-picar-v2/install/nevil_interfaces_ai)
    # and construct the path to the Python module
    return PathJoinSubstitution([
        get_package_share_directory(package_name),
        '..', 'lib', python_version_path, 'site-packages', package_name,
        node_file_name_with_py_extension
    ])

def get_python_interpreter_executable() -> TextSubstitution:
    """
    Returns the appropriate Python executable for the current environment.
    """
    return TextSubstitution(text=sys.executable)