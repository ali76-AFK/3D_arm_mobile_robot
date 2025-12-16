import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory for the robot_vision package
    robot_vision_dir = get_package_share_directory('robot_vision')

    # Path to the vision_node.py executable
    vision_node_path = os.path.join(robot_vision_dir, 'robot_vision', 'vision_node.py')

    # Declare launch arguments for model path and confidence threshold
    yolov8_model_arg = DeclareLaunchArgument(
        'yolov8_model_path',
        default_value='yolov8n.pt',
        description='Path to the YOLOv8 model file (.pt)'
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for YOLOv8 detections'
    )

    vision_node = Node(
        package='robot_vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        emulate_tty=True, # For logging in terminal
        parameters=[
            {'yolov8_model_path': LaunchConfiguration('yolov8_model_path')},
            {'confidence_threshold': LaunchConfiguration('confidence_threshold')}
        ]
    )

    return LaunchDescription([
        yolov8_model_arg,
        confidence_threshold_arg,
        vision_node
    ])
