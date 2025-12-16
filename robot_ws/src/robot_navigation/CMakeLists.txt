cmake_minimum_required(VERSION 3.8 FATAL_ERROR) # ROS2 Humble often uses newer CMake
project(robot_navigation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dwb_core REQUIRED)
find_package(launch_ros REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# The ament_python_install_package() or ament_package() handles the setup.py
# and its installation rules.
# Explicit installation of the resource marker is handled by ament_package().
# Remove: install(FILES resource/robot_navigation DESTINATION share/${PROJECT_NAME})

ament_package() # This implicitly generates rules for setup.py and other Python-related files.
