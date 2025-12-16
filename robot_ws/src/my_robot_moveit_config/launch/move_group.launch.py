import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the launch directory for the MoveIt2 config package
    my_robot_moveit_config_dir = get_package_share_directory('my_robot_moveit_config')
    robot_description_dir = get_package_share_directory('robot_description')

    # URDF file path - will be passed to robot_state_publisher and MoveIt
    # Note: Use PathJoinSubstitution to resolve paths during launch
    urdf_path = PathJoinSubstitution([robot_description_dir, 'urdf', 'robot.urdf.xacro'])

    # Configuration files for MoveIt
    joint_limits_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'joint_limits.yaml'])
    kinematics_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'kinematics.yaml'])
    ros2_controllers_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'ros2_controllers.yaml'])
    moveit_controllers_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'moveit_controllers.yaml'])
    fake_controllers_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'fake_controllers.yaml'])
    ompl_planning_yaml = PathJoinSubstitution([my_robot_moveit_config_dir, 'config', 'ompl_planning.yaml']) # Needs to be created

    # SRDF file path
    srdf_path = PathJoinSubstitution([my_robot_moveit_config_dir, 'srdf', 'my_robot.srdf'])

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('robot_description')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # --- Joint State Publisher (GUI) ---
    # Used to manually control joints in simulation or testing
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_jsp_gui')),
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- RViz2 --- (Optional, for visualization)
    rviz_config_file = PathJoinSubstitution([my_robot_moveit_config_dir, 'rviz', 'moveit.rviz']) # Needs to be created
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('robot_description_semantic') # For SRDF in RViz
        ]
    )

    # --- MoveIt2 Launch (move_group node) ---
    # Load the xacro content for robot_description parameter
    robot_description_content = os.popen(f'ros2 run xacro xacro {urdf_path.perform(context)}').read()

    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('moveit_configs_utils'), 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={
            'robot_description': robot_description_content,
            'robot_description_semantic': srdf_path,
            'kinematics_yaml': kinematics_yaml,
            'joint_limits_yaml': joint_limits_yaml,
            'moveit_controllers_yaml': moveit_controllers_yaml,
            'ompl_planning_yaml': ompl_planning_yaml,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'fake_controllers_yaml': fake_controllers_yaml,
            'use_fake_controller': LaunchConfiguration('use_fake_controller'),
            'publish_robot_description_also': True,
            'publish_robot_description_semantic_also': True,
            'pipeline': 'ompl'
        }.items()
    )

    # Controller Manager (if not using fake controllers)
    ros2_controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_yaml],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_real_controller')),
        arguments=['--ros-args', '--log-level', 'info'] # Or debug
    )

    # Spawn controllers (if not using fake controllers)
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('use_real_controller')),
    )

    spawn_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration('use_real_controller')),
    )


    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        move_group_node,
        ros2_controller_manager_node,
        spawn_arm_controller,
        spawn_gripper_controller
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_fake_controller',
            default_value='true',
            description='Use fake controllers for MoveIt if true. Otherwise, attempt to spawn real ones.'),
        DeclareLaunchArgument(
            'use_real_controller',
            default_value='false',
            description='Use real hardware controllers (ROS2 Control) if true. Mutually exclusive with use_fake_controller.'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz2.'),
        DeclareLaunchArgument(
            'use_jsp_gui',
            default_value='false',
            description='Whether to launch Joint State Publisher GUI.'),
        # Declare a single robot_description argument, its content will be filled by OpaqueFunction
        DeclareLaunchArgument(
            'robot_description',
            default_value='',
            description='URDF for the robot (filled by xacro parsing)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
