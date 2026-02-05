#!/bin/bash
# Proxigo Scalence - Camera Calibration Helper
#
# This script helps with camera intrinsic calibration using ROS2 camera_calibration
#
# Requirements:
#   - Checkerboard pattern (e.g., 9x6, 25mm squares)
#   - Camera connected and publishing images
#
# Usage:
#   ./scripts/calibrate_camera.sh

set -e

echo "=============================================="
echo "Proxigo Scalence - Camera Calibration"
echo "=============================================="

# Configuration
CHECKERBOARD_SIZE=${1:-"9x6"}   # Columns x Rows of inner corners
SQUARE_SIZE=${2:-"0.025"}       # Square size in meters
CAMERA_TOPIC=${3:-"/vio/camera/image_raw"}

echo "Checkerboard: $CHECKERBOARD_SIZE"
echo "Square size: $SQUARE_SIZE meters"
echo "Camera topic: $CAMERA_TOPIC"
echo ""

# Check if running in Docker
if [ -f /.dockerenv ]; then
    echo "Running inside Docker container"
else
    echo "Running on host system"
fi

# Check if camera topic exists
echo "Checking for camera topic..."
if ! ros2 topic list | grep -q "$CAMERA_TOPIC"; then
    echo "ERROR: Camera topic $CAMERA_TOPIC not found"
    echo "Available image topics:"
    ros2 topic list | grep -i image
    exit 1
fi

echo "Camera topic found. Starting calibration..."
echo ""
echo "Instructions:"
echo "  1. Hold the checkerboard in front of the camera"
echo "  2. Move it around to cover the entire field of view"
echo "  3. Tilt the board to get different angles"
echo "  4. Once enough samples are collected, click 'Calibrate'"
echo "  5. Click 'Save' to save the calibration"
echo ""
echo "Press Enter to start..."
read

# Run camera calibration
ros2 run camera_calibration cameracalibrator \
    --size "$CHECKERBOARD_SIZE" \
    --square "$SQUARE_SIZE" \
    --camera "$CAMERA_TOPIC" \
    --no-service-check

echo ""
echo "Calibration complete!"
echo "The calibration file should be saved in /tmp/calibration*.yaml"
echo ""
echo "Copy the intrinsics to config/camera/arducam_imx477.yaml"
