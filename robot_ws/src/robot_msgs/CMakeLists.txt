cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedShape.msg"
  "msg/DetectedShapes.msg"
  "srv/PickObject.srv"
  "srv/PlaceObject.srv"
  "srv/StartTask.srv"
)

# Install generated interfaces
install(DIRECTORY
  msg
  srv
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
