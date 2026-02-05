# Proxigo Scalence - VIO Core Container
# OpenVINS + Camera drivers + VIO-Satellite bridge
#
# Build: docker build -t proxigo/vio-core:latest -f docker/vio_core/Dockerfile .

FROM proxigo/ros2-jetson-base:latest

# Install VIO dependencies
RUN apt-get update && apt-get install -y \
    libceres-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# Install camera drivers
RUN apt-get update && apt-get install -y \
    ros-humble-v4l2-camera \
    ros-humble-image-pipeline \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for VIO bridge
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    pyyaml \
    transforms3d

# Clone OpenVINS (ROS2 compatible)
WORKDIR /ros2_ws/src
RUN git clone --depth 1 https://github.com/rpng/open_vins.git

# Copy VIO-Satellite bridge package (placeholder - will be developed)
# COPY src/vio_satellite_bridge /ros2_ws/src/vio_satellite_bridge

# Copy VIO core package
# COPY src/vio_core /ros2_ws/src/vio_core

# Build workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy configuration
COPY config/camera /ros2_ws/config/camera
COPY config/vio /ros2_ws/config/vio

# Create logs directory
RUN mkdir -p /ros2_ws/logs

# Set environment
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "ov_msckf", "subscribe.launch.py"]
