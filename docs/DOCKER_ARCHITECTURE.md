# Docker Architecture for Proxigo Scalence

## Overview

The system uses Docker containers to isolate components, ensure reproducibility, and enable development on both Windows 10 and Orin Nano platforms. All containers are orchestrated using Docker Compose.

---

## Container Strategy

### Container Hierarchy

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ORIN NANO HOST (JetPack 6.x)                        │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                    Docker + NVIDIA Container Toolkit                   │  │
│  │                                                                        │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │  │
│  │  │   vio_core      │  │   navigation    │  │   satellite_match   │   │  │
│  │  │                 │  │                 │  │                     │   │  │
│  │  │  - OpenVINS     │  │  - MAVROS2      │  │  - Image matching   │   │  │
│  │  │  - Camera node  │  │  - Path planner │  │  - Geo-referencing  │   │  │
│  │  │  - IMU driver   │  │  - Waypoint nav │  │  - Satellite DB     │   │  │
│  │  │                 │  │                 │  │                     │   │  │
│  │  │  GPU: ✓         │  │  GPU: ✗         │  │  GPU: ✓             │   │  │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────────┘   │  │
│  │           │                    │                     │                │  │
│  │           └────────────────────┼─────────────────────┘                │  │
│  │                                │                                      │  │
│  │                    ┌───────────▼───────────┐                          │  │
│  │                    │    ROS2 DDS Bridge    │                          │  │
│  │                    │   (Shared Network)    │                          │  │
│  │                    └───────────────────────┘                          │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Container Definitions

### 1. Base Image: `proxigo/ros2-jetson-base`

```dockerfile
# docker/base/Dockerfile
FROM nvcr.io/nvidia/l4t-jetpack:r36.2.0

# Set environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# OpenCV with CUDA support (from JetPack)
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Eigen3 for VIO
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws
```

### 2. VIO Core Container: `proxigo/vio-core`

```dockerfile
# docker/vio_core/Dockerfile
FROM proxigo/ros2-jetson-base:latest

# Install VIO dependencies
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libceres-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install camera drivers
RUN apt-get update && apt-get install -y \
    ros-humble-v4l2-camera \
    ros-humble-image-pipeline \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Clone and build OpenVINS (ROS2 version)
WORKDIR /ros2_ws/src
RUN git clone https://github.com/rpng/open_vins.git

# Copy custom satellite matching integration
COPY src/vio_satellite_bridge /ros2_ws/src/vio_satellite_bridge

# Build
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Camera configuration for Arducam IMX477
COPY config/camera/arducam_imx477.yaml /ros2_ws/config/camera.yaml

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "vio_core", "vio_core.launch.py"]
```

### 3. Navigation Container: `proxigo/navigation`

```dockerfile
# docker/navigation/Dockerfile
FROM proxigo/ros2-jetson-base:latest

# Install MAVROS2 and navigation packages
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-geographic-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Copy navigation nodes
COPY src/navigation /ros2_ws/src/navigation
COPY src/obstacle_avoidance /ros2_ws/src/obstacle_avoidance

# Build
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "navigation", "navigation.launch.py"]
```

### 4. Satellite Matching Container: `proxigo/satellite-match`

```dockerfile
# docker/satellite_match/Dockerfile
FROM proxigo/ros2-jetson-base:latest

# Install Python ML dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    torch \
    torchvision \
    kornia \
    numpy \
    scipy \
    rasterio \
    pyproj

# Install feature matching libraries
RUN pip3 install --no-cache-dir \
    opencv-contrib-python \
    scikit-image

# Copy satellite matching package
COPY src/satellite_matching /ros2_ws/src/satellite_matching

# Build
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Volume mount point for satellite data
VOLUME ["/satellite_data"]

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "satellite_matching", "satellite_match.launch.py"]
```

---

## Docker Compose Configuration

### Production: `docker-compose.yml`

```yaml
version: '3.8'

services:
  vio_core:
    image: proxigo/vio-core:latest
    container_name: vio_core
    runtime: nvidia
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /dev:/dev:rw
      - ./config:/ros2_ws/config:ro
      - ./logs/vio:/ros2_ws/logs:rw
    devices:
      - /dev/video0:/dev/video0  # Arducam camera
    depends_on:
      - satellite_match
    restart: unless-stopped

  navigation:
    image: proxigo/navigation:latest
    container_name: navigation
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./config:/ros2_ws/config:ro
      - ./logs/nav:/ros2_ws/logs:rw
    depends_on:
      - vio_core
    restart: unless-stopped

  satellite_match:
    image: proxigo/satellite-match:latest
    container_name: satellite_match
    runtime: nvidia
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - NVIDIA_VISIBLE_DEVICES=all
    volumes:
      - ./config:/ros2_ws/config:ro
      - ./satellite_data:/satellite_data:ro
      - ./logs/sat:/ros2_ws/logs:rw
    restart: unless-stopped

  # Optional: Visualization (for development/debugging)
  rviz:
    image: proxigo/ros2-jetson-base:latest
    container_name: rviz
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./config/rviz:/rviz_config:ro
    command: ros2 run rviz2 rviz2 -d /rviz_config/default.rviz
    profiles:
      - debug
```

### Development/Simulation: `docker-compose.sim.yml`

```yaml
version: '3.8'

services:
  px4_sitl:
    image: px4io/px4-dev-simulation-jammy:latest
    container_name: px4_sitl
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - PX4_SIM_HOST_ADDR=127.0.0.1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./sitl_config:/px4_config:ro
    command: >
      bash -c "cd /root/PX4-Autopilot && 
               make px4_sitl_default jmavsim"
    profiles:
      - simulation

  vio_core_sim:
    image: proxigo/vio-core:dev
    container_name: vio_core_sim
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - SIM_MODE=true
    volumes:
      - ./config:/ros2_ws/config:ro
      - ./test_data:/test_data:ro
      - ./logs:/ros2_ws/logs:rw
    command: >
      ros2 launch vio_core vio_core.launch.py 
        sim_mode:=true 
        bag_file:=/test_data/sample.db3
    depends_on:
      - px4_sitl
    profiles:
      - simulation

  navigation_sim:
    image: proxigo/navigation:dev
    container_name: navigation_sim
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./config:/ros2_ws/config:ro
    depends_on:
      - vio_core_sim
    profiles:
      - simulation
```

---

## Cross-Platform Development

### Windows 10 Development Setup

```powershell
# Install WSL2 with Ubuntu 22.04
wsl --install -d Ubuntu-22.04

# Install Docker Desktop with WSL2 backend
# Enable "Use WSL 2 based engine" in Docker Desktop settings

# Install VS Code with Remote - WSL extension
code --install-extension ms-vscode-remote.remote-wsl
```

### WSL2 Docker Compose Override: `docker-compose.windows.yml`

```yaml
version: '3.8'

services:
  vio_core:
    # Override for Windows/WSL2 development
    build:
      context: .
      dockerfile: docker/vio_core/Dockerfile.dev
    volumes:
      - ./src:/ros2_ws/src:rw  # Bind mount for live development
      - ./config:/ros2_ws/config:ro
    environment:
      - SIM_MODE=true

  satellite_match:
    volumes:
      - ./src/satellite_matching:/ros2_ws/src/satellite_matching:rw
      - ./satellite_data:/satellite_data:ro
```

### Combined Development Command

```bash
# On Windows (PowerShell)
docker-compose -f docker-compose.yml -f docker-compose.windows.yml up vio_core satellite_match

# On Orin Nano
docker-compose up -d
```

---

## Build Scripts

### `scripts/build_all.sh`

```bash
#!/bin/bash
set -e

PLATFORM=${1:-"jetson"}  # jetson or amd64

echo "Building for platform: $PLATFORM"

if [ "$PLATFORM" == "jetson" ]; then
    BASE_IMAGE="nvcr.io/nvidia/l4t-jetpack:r36.2.0"
else
    BASE_IMAGE="ros:humble"
fi

# Build base image
docker build \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    -t proxigo/ros2-jetson-base:latest \
    -f docker/base/Dockerfile .

# Build VIO core
docker build \
    -t proxigo/vio-core:latest \
    -f docker/vio_core/Dockerfile .

# Build navigation
docker build \
    -t proxigo/navigation:latest \
    -f docker/navigation/Dockerfile .

# Build satellite matching
docker build \
    -t proxigo/satellite-match:latest \
    -f docker/satellite_match/Dockerfile .

echo "Build complete!"
```

### `scripts/deploy_orin.sh`

```bash
#!/bin/bash
# Deploy to Orin Nano via SSH

ORIN_HOST=${1:-"orin.local"}
ORIN_USER=${2:-"nvidia"}

echo "Deploying to $ORIN_USER@$ORIN_HOST..."

# Sync configuration
rsync -avz --delete \
    ./config/ \
    $ORIN_USER@$ORIN_HOST:~/proxigo/config/

# Sync satellite data
rsync -avz --delete \
    ./satellite_data/ \
    $ORIN_USER@$ORIN_HOST:~/proxigo/satellite_data/

# Deploy and restart containers
ssh $ORIN_USER@$ORIN_HOST << 'EOF'
    cd ~/proxigo
    docker-compose pull
    docker-compose down
    docker-compose up -d
EOF

echo "Deployment complete!"
```

---

## Volume Management

### Satellite Data Volume Structure

```
satellite_data/
├── regions/
│   ├── region_001/
│   │   ├── metadata.json        # Bounds, resolution, CRS
│   │   ├── satellite.tif        # GeoTIFF imagery
│   │   ├── features.bin         # Pre-computed features
│   │   └── elevation.tif        # DEM (optional)
│   └── region_002/
│       └── ...
├── index.json                   # Region catalog
└── calibration/
    └── geo_transform.yaml       # Coordinate transform params
```

### Configuration Volume Structure

```
config/
├── camera/
│   ├── arducam_imx477.yaml      # Camera intrinsics
│   └── camera_imu_extrinsics.yaml
├── vio/
│   ├── openvins_config.yaml     # VIO parameters
│   └── estimator_config.yaml
├── navigation/
│   ├── mavros_config.yaml       # MAVROS settings
│   └── px4_config.yaml
└── mission/
    └── initial_pose.yaml        # Startup position
```

---

## Health Monitoring

### Docker Healthchecks

```yaml
# In docker-compose.yml
services:
  vio_core:
    healthcheck:
      test: ["CMD", "ros2", "topic", "hz", "/vio/odom", "--window", "5"]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 30s
```

### Monitoring Stack (Optional)

```yaml
  prometheus:
    image: prom/prometheus:latest
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml
    ports:
      - "9090:9090"
    profiles:
      - monitoring

  grafana:
    image: grafana/grafana:latest
    volumes:
      - ./monitoring/grafana:/var/lib/grafana
    ports:
      - "3000:3000"
    profiles:
      - monitoring
```
