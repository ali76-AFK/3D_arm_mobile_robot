cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_configs_utils REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (camera_info.yaml)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts as executables
# The coordinate_transformer_node.py and arm_gripper_controller_node.py are placed inside the 'robot_arm_control' sub-directory.
install(PROGRAMS
  robot_arm_control/coordinate_transformer_node.py
  robot_arm_control/arm_gripper_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python package itself (defined in setup.py)
# This is crucial for ament_python packages.
ament_python_install_package()

# Mark the package as compliant with ament_python
# This implicitly generates rules for setup.py and other Python-related files.
ament_package()
