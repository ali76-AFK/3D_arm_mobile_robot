cmake_minimum_required(VERSION 3.8)
project(robot_task_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(launch_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts as executables
# The task_orchestration_node.py is placed inside the 'robot_task_manager' sub-directory.
install(PROGRAMS
  robot_task_manager/task_orchestration_node.py
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
