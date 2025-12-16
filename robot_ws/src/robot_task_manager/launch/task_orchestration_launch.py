import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory for the robot_task_manager package
    robot_task_manager_dir = get_package_share_directory('robot_task_manager')

    # Path to the task_orchestration_node.py executable
    # The executable name should match the entry_points in setup.py if installed as a console_script
    # Otherwise, it's python_pkg_dir/script_name.py

    task_orchestration_node = Node(
        package='robot_task_manager',
        executable='task_orchestration_node.py',
        name='task_orchestration_node',
        output='screen',
        emulate_tty=True, # For logging in terminal
    )

    return LaunchDescription([
        task_orchestration_node
    ])
