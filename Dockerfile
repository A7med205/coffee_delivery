
# Use the ROS 2 Humble desktop base image on Ubuntu Focal
FROM osrf/ros:humble-desktop-full

# Install general dependencies
RUN apt-get update && apt-get install -y \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-controller-manager \
  ros-humble-ur-description \
  ros-humble-xacro \
  ros-humble-moveit \
  ros-humble-urdf \
  python3 \
  python3-opencv \
  python3-rosdep \
  python3-pip \
  ros-humble-cv-bridge \
  ros-humble-rosbridge-server \
  && rm -rf /var/lib/apt/lists/*

# Fix pip-based numpy
RUN pip install --upgrade --force-reinstall "numpy==1.24.*"

# Create a workspace and copy your package
RUN mkdir -p /home/user/ros2_ws/src/coffee_delivery
WORKDIR /home/user/ros2_ws

# Copy everything
COPY delivery_app       /home/user/ros2_ws/src/coffee_delivery/delivery_app
COPY moveit_config      /home/user/ros2_ws/src/coffee_delivery/moveit_config
COPY real_moveit_config /home/user/ros2_ws/src/coffee_delivery/real_moveit_config
COPY moveit2_scripts     /home/user/ros2_ws/src/coffee_delivery/moveit_scripts

# Use rosdep to install missing ROS dependencies
# RUN rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
  && colcon build"

# Source the workspace at startup
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
  && echo 'source /home/user/ros2_ws/install/setup.bash' >> /root/.bashrc

# Default command: bash shell
CMD ["bash"]
