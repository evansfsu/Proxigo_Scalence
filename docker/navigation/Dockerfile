# Proxigo Scalence - Navigation Container
# MAVROS2 + Path planning + PX4 interface
#
# Build: docker build -t proxigo/navigation:latest -f docker/navigation/Dockerfile .

FROM proxigo/ros2-jetson-base:latest

# Install MAVROS2 and dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-geographic-msgs \
    ros-humble-geodesy \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required for MAVROS)
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    pyproj \
    pyyaml

# Clone/copy navigation packages
WORKDIR /ros2_ws/src

# Copy navigation package (placeholder)
# COPY src/navigation /ros2_ws/src/navigation

# Copy obstacle avoidance package (placeholder)
# COPY src/obstacle_avoidance /ros2_ws/src/obstacle_avoidance

# Build workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Copy configuration
COPY config/navigation /ros2_ws/config/navigation
COPY config/mission /ros2_ws/config/mission

# Create logs directory
RUN mkdir -p /ros2_ws/logs

# Set environment
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "mavros", "px4.launch.py", "fcu_url:=/dev/ttyTHS1:921600"]
