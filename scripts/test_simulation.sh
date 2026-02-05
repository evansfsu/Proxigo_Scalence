#!/bin/bash
# Proxigo Scalence - Test Simulation Stack
#
# Tests the simulation by:
# 1. Starting the camera/IMU simulators
# 2. Starting satellite matching
# 3. Starting state fusion
# 4. Verifying topics are publishing

set -e

echo "=============================================="
echo "Proxigo Scalence - Simulation Test"
echo "=============================================="
echo ""

# Check if we're in Docker with ROS2
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: Run inside Docker container"
    exit 1
fi

source /opt/ros/humble/setup.bash

# Check if packages are built
if [ ! -f /ros2_ws/install/setup.bash ]; then
    echo "Packages not built. Building now..."
    cd /ros2_ws
    colcon build --symlink-install --packages-select vio_bridge satellite_matching state_fusion
fi

source /ros2_ws/install/setup.bash

echo ">>> Starting simulation nodes..."
echo ""

# Start nodes in background
ros2 run vio_bridge camera_simulator &
CAM_PID=$!
echo "Started camera_simulator (PID: $CAM_PID)"

ros2 run vio_bridge imu_simulator &
IMU_PID=$!
echo "Started imu_simulator (PID: $IMU_PID)"

sleep 2

ros2 run vio_bridge vio_bridge_node &
VIO_PID=$!
echo "Started vio_bridge_node (PID: $VIO_PID)"

sleep 2

ros2 run satellite_matching satellite_matcher_node &
SAT_PID=$!
echo "Started satellite_matcher_node (PID: $SAT_PID)"

sleep 2

# Verify topics
echo ""
echo ">>> Checking topics..."
echo ""

TOPICS=(
    "/vio/camera/image_raw"
    "/vio/imu/data"
    "/sat_match/query_trigger"
)

ALL_OK=true
for topic in "${TOPICS[@]}"; do
    if ros2 topic list | grep -q "$topic"; then
        echo "[OK] $topic"
    else
        echo "[FAIL] $topic - not found"
        ALL_OK=false
    fi
done

echo ""

# Check topic rates
echo ">>> Checking topic rates (5 second sample)..."
echo ""

ros2 topic hz /vio/camera/image_raw --window 10 &
HZ_PID=$!
sleep 5
kill $HZ_PID 2>/dev/null || true

echo ""
echo ">>> Listing all active topics..."
ros2 topic list

# Cleanup
echo ""
echo ">>> Cleaning up..."
kill $CAM_PID $IMU_PID $VIO_PID $SAT_PID 2>/dev/null || true

echo ""
if [ "$ALL_OK" = true ]; then
    echo "=============================================="
    echo "Simulation test PASSED!"
    echo "=============================================="
else
    echo "=============================================="
    echo "Simulation test FAILED - some topics missing"
    echo "=============================================="
    exit 1
fi
