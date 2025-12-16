#!/bin/bash

# Source ROS2 Humble setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
source /ros_ws/install/setup.bash

# Execute the command passed to the container
exec "$@"
