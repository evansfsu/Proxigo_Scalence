# Proxigo Scalence - Satellite Matching Container
# Aerial-to-satellite image matching for absolute positioning
#
# Build: docker build -t proxigo/satellite-match:latest -f docker/satellite_match/Dockerfile .

FROM proxigo/ros2-jetson-base:latest

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    libgdal-dev \
    gdal-bin \
    && rm -rf /var/lib/apt/lists/*

# Install Python ML and geospatial dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    opencv-contrib-python \
    scikit-image \
    rasterio \
    pyproj \
    shapely \
    pyyaml \
    pillow

# Install PyTorch for GPU-accelerated feature matching (optional)
# Uncomment for neural network-based matching (SuperGlue, etc.)
# RUN pip3 install --no-cache-dir \
#     torch \
#     torchvision \
#     kornia

# Copy satellite matching package (placeholder)
WORKDIR /ros2_ws/src
# COPY src/satellite_matching /ros2_ws/src/satellite_matching

# Build workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Create directories
RUN mkdir -p /ros2_ws/logs
RUN mkdir -p /satellite_data

# Volume mount point for satellite data
VOLUME ["/satellite_data"]

# Copy configuration
COPY config/mission /ros2_ws/config/mission

# Set environment
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]
# CMD ["ros2", "launch", "satellite_matching", "satellite_match.launch.py"]
