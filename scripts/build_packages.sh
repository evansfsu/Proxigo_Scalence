#!/bin/bash
# Proxigo Scalence - Build ROS2 Packages
#
# Usage:
#   ./scripts/build_packages.sh              # Build all packages
#   ./scripts/build_packages.sh vio_bridge   # Build specific package

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "Proxigo Scalence - Build ROS2 Packages"
echo "=============================================="
echo "Project: $PROJECT_DIR"
echo ""

# Check if we're in Docker
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: This script must be run inside the Docker container"
    echo ""
    echo "Usage:"
    echo "  docker compose -f docker-compose.dev.yml run --rm dev_shell bash"
    echo "  ./scripts/build_packages.sh"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Go to workspace
cd /ros2_ws

# Link source packages if not already linked
if [ ! -L /ros2_ws/src/vio_bridge ]; then
    echo "Linking source packages..."
    rm -rf /ros2_ws/src/*
    ln -sf /ros2_ws/src_custom/* /ros2_ws/src/ 2>/dev/null || true
    
    # If src_custom doesn't exist, copy from project
    if [ -d "$PROJECT_DIR/src" ]; then
        cp -r "$PROJECT_DIR/src/"* /ros2_ws/src/ 2>/dev/null || true
    fi
fi

# Determine what to build
if [ -n "$1" ]; then
    PACKAGES="--packages-select $@"
    echo "Building packages: $@"
else
    PACKAGES=""
    echo "Building all packages..."
fi
echo ""

# Install dependencies
echo ">>> Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true
echo ""

# Build
echo ">>> Building..."
colcon build --symlink-install $PACKAGES --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=============================================="
echo "Build complete!"
echo "=============================================="
echo ""
echo "To use the packages:"
echo "  source /ros2_ws/install/setup.bash"
echo ""
echo "To run simulation:"
echo "  ros2 launch proxigo_bringup simulation.launch.py"
