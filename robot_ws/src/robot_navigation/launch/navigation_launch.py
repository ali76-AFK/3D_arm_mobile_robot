import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    # This part will be executed by ROS2 launch when the file is run, where get_package_share_directory is available.
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to config files
    nav2_params_file = os.path.join(robot_navigation_dir, 'config', 'nav2_params.yaml')
    amcl_params_file = os.path.join(robot_navigation_dir, 'config', 'amcl_config.yaml')
    global_costmap_params_file = os.path.join(robot_navigation_dir, 'config', 'global_costmap.yaml')
    local_costmap_params_file = os.path.join(robot_navigation_dir, 'config', 'local_costmap.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='true')

    # Nav2 bringup launch
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map_file', default=''), # Map file will be passed here if pre-mapped
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'default_bt_xml_path': PathJoinSubstitution(
                [nav2_bringup_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml']),
            'map_subscribe_transient_local': map_subscribe_transient_local,
        }.items()
    )

    # AMCL Node (if using separate AMCL parameters or not via bringup's params_file)
    # The nav2_bringup_cmd above includes AMCL configuration via nav2_params_file,
    # but if specific AMCL params need overriding, uncomment and adjust this:
    # amcl_node = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[amcl_params_file],
    #     remappings=[('laser_scan', '/scan')],
    # )

    # SLAM Toolbox (for mapping)
    # For real-time SLAM, uncomment this. For pre-mapped navigation, keep commented.
    # slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'slam_toolbox_launch.py')),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'autostart': autostart,
    #     }.items()
    # )

    # Robot state publisher to publish URDF
    robot_description_pkg_dir = get_package_share_directory('robot_description')
    urdf_path = os.path.join(robot_description_pkg_dir, 'urdf', 'robot.urdf.xacro')

    # Note: robot_state_publisher should typically be launched with the robot model
    # so it might be better in a separate 'robot_description_launch.py'.
    # For simplicity, we include it here.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': os.popen(f'ros2 run xacro xacro {urdf_path}').read()}],
    )

    ld = LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start the Nav2 stack'),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Full path to a map file to load'),
        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='Whether to subscribe to the map topic with transient local QoS'),

        robot_state_publisher_node,
        nav2_bringup_cmd,
        # slam_toolbox_cmd, # Uncomment for SLAM
        # amcl_node,        # Uncomment if AMCL needs separate config
    ])

    return ld
