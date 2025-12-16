
# Use a base image with ROS2 Humble pre-installed
FROM ros:humble-ros-base-jammy

# Set environment variables for ROS2
ENV ROS_DISTRO=humble
ENV ROS_WS=/ros_ws
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
# Replaced nav2-bringup and nav2-recoveries with the meta-package
RUN apt-get update && apt-get install -y --no-install-recommends     ros-$ROS_DISTRO-desktop     ros-$ROS_DISTRO-joint-state-publisher-gui     ros-$ROS_DISTRO-moveit     ros-$ROS_DISTRO-moveit-configs-utils     ros-$ROS_DISTRO-moveit-ros-planning-interface     ros-$ROS_DISTRO-navigation2     ros-$ROS_DISTRO-gazebo-ros-pkgs     ros-$ROS_DISTRO-ros2-control     ros-$ROS_DISTRO-ros2-controllers     ros-$ROS_DISTRO-control-msgs     ros-$ROS_DISTRO-tf2-ros     ros-$ROS_DISTRO-tf2-geometry-msgs     ros-$ROS_DISTRO-cv-bridge     ros-$ROS_DISTRO-image-transport     ros-$ROS_DISTRO-vision-opencv     ros-$ROS_DISTRO-image-geometry     ros-$ROS_DISTRO-xacro     ros-$ROS_DISTRO-slam-toolbox     python3-pip     python3-opencv     python3-yaml     build-essential     git     vim     iputils-ping     net-tools     && rm -rf /var/lib/apt/lists/*

# Install pip dependencies (YOLOv8)
RUN pip install --no-cache-dir ultralytics==8.0.0 # Specify version for stability

# Create ROS2 workspace
WORKDIR $ROS_WS
RUN mkdir -p src

# Copy custom ROS2 packages
COPY src/ $ROS_WS/src/

# Build the workspace
ARG COLCON_WS_SRC=$ROS_WS/src
ARG COLCON_WS_INSTALL=$ROS_WS/install
RUN . /opt/ros/$ROS_DISTRO/setup.sh &&     colcon build --symlink-install --packages-ignore dynamixel_sdk dynamixel_workbench_ros2_control     --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the ROS2 setup file for entrypoint
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# Set up entrypoint script
COPY docker_entrypoint.sh /usr/local/bin/docker_entrypoint.sh
RUN chmod +x /usr/local/bin/docker_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]

# Default command to run a bash shell if no command is provided
CMD ["bash"]
