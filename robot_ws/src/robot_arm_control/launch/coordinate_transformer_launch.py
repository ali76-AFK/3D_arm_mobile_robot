import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the launch directory for the robot_arm_control package
    robot_arm_control_dir = get_package_share_directory('robot_arm_control')

    # Path to the camera info YAML file
    camera_info_yaml_path = PathJoinSubstitution(
        [robot_arm_control_dir, 'config', LaunchConfiguration('camera_info_file')]
    ).perform(context)

    # Path to the coordinate_transformer_node.py executable
    # The executable name should match the entry_points in setup.py if installed as a console_script
    # Otherwise, it's python_pkg_dir/script_name.py

    coordinate_transformer_node = Node(
        package='robot_arm_control',
        executable='coordinate_transformer_node.py',
        name='coordinate_transformer_node',
        output='screen',
        emulate_tty=True, # For logging in terminal
        parameters=[
            LaunchConfiguration('params_file'), # Main param file for other config if needed
            {'camera_info_file': LaunchConfiguration('camera_info_file')}
        ]
    )

    # Robot state publisher to publish URDF
    # This is typically launched once with the robot description.
    # Including it here for completeness if not launched elsewhere.
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    urdf_path = os.path.join(robot_description_pkg_dir, 'urdf', 'robot.urdf.xacro')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': os.popen(f'ros2 run xacro xacro {urdf_path}').read()}],
    )

    return [
        robot_state_publisher_node,
        coordinate_transformer_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_info_file',
            default_value='camera_info.yaml',
            description='Path to the camera info YAML file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('robot_arm_control'),
                'config',
                'coordinate_transformer_params.yaml' # A new parameter file for general params
            ]),
            description='Path to the ROS2 parameters file for the coordinate transformer node'
        ),
        OpaqueFunction(function = launch_setup)
    ])
