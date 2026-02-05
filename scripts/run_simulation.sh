#!/bin/bash
# Proxigo Scalence - Run Full System Simulation
#
# This script launches the complete simulation stack for testing.
#
# Usage:
#   ./scripts/run_simulation.sh           # Basic simulation
#   ./scripts/run_simulation.sh --px4     # With PX4 SITL
#   ./scripts/run_simulation.sh --test    # Run automated tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Proxigo Scalence - Full System Simulation${NC}"
echo -e "${BLUE}============================================${NC}"

# Parse arguments
WITH_PX4=false
RUN_TESTS=false
REGION_ID="test_region"

while [[ $# -gt 0 ]]; do
    case $1 in
        --px4)
            WITH_PX4=true
            shift
            ;;
        --test)
            RUN_TESTS=true
            shift
            ;;
        --region)
            REGION_ID="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--px4] [--test] [--region REGION_ID]"
            exit 1
            ;;
    esac
done

# Check if satellite data exists
if [ ! -d "satellite_data/regions/${REGION_ID}" ]; then
    echo -e "${YELLOW}Warning: Satellite region '${REGION_ID}' not found${NC}"
    echo "Creating test satellite data..."
    python3 scripts/create_test_satellite_data.py 2>/dev/null || \
        echo "Note: Run this inside Docker for full functionality"
fi

# Ensure Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running${NC}"
    echo "Please start Docker Desktop and try again."
    exit 1
fi

echo ""
echo -e "${GREEN}Starting simulation stack...${NC}"
echo "  Mode: Simulation (no hardware)"
echo "  Region: ${REGION_ID}"
echo "  PX4: ${WITH_PX4}"
echo ""

if [ "$WITH_PX4" = true ]; then
    echo -e "${BLUE}Launching with PX4 SITL...${NC}"
    
    # Start PX4 SITL in background
    docker compose -f docker-compose.sitl.yml up -d px4_sitl
    
    # Wait for PX4 to be ready
    echo "Waiting for PX4 SITL to start..."
    sleep 10
    
    # Run the full system with MAVROS connection
    docker compose -f docker-compose.dev.yml run --rm \
        -e ROS_DOMAIN_ID=0 \
        dev_shell bash -c "
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/setup.bash
            ros2 launch proxigo_bringup full_system.launch.py \
                sim_mode:=true \
                enable_mavros:=true \
                fcu_url:='udp://:14540@px4_sitl:14557' \
                region_id:=${REGION_ID}
        "
else
    # Run simulation without PX4
    if [ "$RUN_TESTS" = true ]; then
        echo -e "${BLUE}Running automated tests...${NC}"
        docker compose -f docker-compose.dev.yml run --rm dev_shell bash -c "
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/setup.bash
            
            echo 'Starting nodes in background...'
            
            # Start camera simulator
            ros2 run vio_bridge camera_simulator &
            CAM_PID=\$!
            
            # Start IMU simulator
            ros2 run vio_bridge imu_simulator &
            IMU_PID=\$!
            
            # Start VIO bridge
            ros2 run vio_bridge vio_bridge_node &
            VIO_PID=\$!
            
            # Start satellite matcher
            ros2 run satellite_matching satellite_matcher_node &
            SAT_PID=\$!
            
            # Start fusion
            ros2 run state_fusion fusion_node &
            FUSION_PID=\$!
            
            sleep 5
            
            echo ''
            echo 'Checking topics...'
            ros2 topic list
            
            echo ''
            echo 'Checking camera output...'
            timeout 5 ros2 topic hz /vio/camera/image_raw --window 3 || echo 'Camera topic check complete'
            
            echo ''
            echo 'Checking IMU output...'
            timeout 5 ros2 topic hz /vio/imu/data --window 3 || echo 'IMU topic check complete'
            
            echo ''
            echo 'Checking fusion output...'
            timeout 5 ros2 topic hz /fused_pose --window 3 || echo 'Fusion topic check complete'
            
            echo ''
            echo 'Killing test nodes...'
            kill \$CAM_PID \$IMU_PID \$VIO_PID \$SAT_PID \$FUSION_PID 2>/dev/null || true
            
            echo ''
            echo 'Tests complete!'
        "
    else
        echo -e "${BLUE}Launching simulation (interactive)...${NC}"
        echo ""
        echo "This will launch all ROS2 nodes. Press Ctrl+C to stop."
        echo ""
        
        docker compose -f docker-compose.dev.yml run --rm dev_shell bash -c "
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/setup.bash
            
            echo 'Launching full system simulation...'
            echo ''
            
            ros2 launch proxigo_bringup full_system.launch.py \
                sim_mode:=true \
                region_id:=${REGION_ID}
        "
    fi
fi

echo ""
echo -e "${GREEN}Simulation complete${NC}"
