#!/bin/bash
# Proxigo Scalence - Build All Docker Images
#
# Usage:
#   ./scripts/build_all.sh          # Build for Jetson (default)
#   ./scripts/build_all.sh amd64    # Build for x86_64 (development)
#   ./scripts/build_all.sh jetson   # Build for Jetson explicitly

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

PLATFORM=${1:-"jetson"}
TAG=${2:-"latest"}

echo "=============================================="
echo "Proxigo Scalence - Docker Image Builder"
echo "=============================================="
echo "Platform: $PLATFORM"
echo "Tag: $TAG"
echo "Project Dir: $PROJECT_DIR"
echo "=============================================="

# Set base image based on platform
if [ "$PLATFORM" == "jetson" ]; then
    BASE_IMAGE="nvcr.io/nvidia/l4t-jetpack:r36.2.0"
    echo "Building for NVIDIA Jetson (ARM64)"
elif [ "$PLATFORM" == "amd64" ]; then
    BASE_IMAGE="ros:humble"
    echo "Building for x86_64 (Development)"
else
    echo "Unknown platform: $PLATFORM"
    echo "Use 'jetson' or 'amd64'"
    exit 1
fi

# Build base image
echo ""
echo ">>> Building base image..."
docker build \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    -t proxigo/ros2-jetson-base:$TAG \
    -f docker/base/Dockerfile \
    .

# Build VIO core
echo ""
echo ">>> Building VIO core image..."
docker build \
    -t proxigo/vio-core:$TAG \
    -f docker/vio_core/Dockerfile \
    .

# Build navigation
echo ""
echo ">>> Building navigation image..."
docker build \
    -t proxigo/navigation:$TAG \
    -f docker/navigation/Dockerfile \
    .

# Build satellite matching
echo ""
echo ">>> Building satellite matching image..."
docker build \
    -t proxigo/satellite-match:$TAG \
    -f docker/satellite_match/Dockerfile \
    .

echo ""
echo "=============================================="
echo "Build complete!"
echo "=============================================="
echo "Images built:"
docker images | grep proxigo
echo ""
echo "To run:"
echo "  docker-compose up -d"
