cmake_minimum_required(VERSION 3.8)
project(my_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(robot_description REQUIRED) # To find the robot.urdf.xacro
find_package(moveit_configs_utils REQUIRED) # For MoveIt2 helper functions

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install srdf directory
install(DIRECTORY
  srdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install the package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install other assets like RViz configs (if created)
# For now, we'll assume a rviz directory exists and install it.
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL # Mark as optional in case it's not created yet
)

ament_export_dependencies(moveit_configs_utils) # Explicitly export dependency

ament_package()
