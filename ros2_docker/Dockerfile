# ROS2 Humble Docker image for GordonBot SLAM
FROM ros:humble-ros-base

# Install SLAM and navigation packages
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for bridge
RUN pip3 install --no-cache-dir \
    websockets \
    aiohttp \
    python-dotenv

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy bridge nodes (will be created)
COPY bridge_nodes/ /ros2_ws/src/bridge_nodes/

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
