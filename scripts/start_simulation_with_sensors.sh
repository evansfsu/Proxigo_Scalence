#!/bin/bash
# Start PX4 SITL with sensor data injection
# This provides sensors for calibration even with "none" simulator

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}================================================${NC}"
echo -e "${CYAN}  PX4 Simulation with Sensor Injection${NC}"
echo -e "${CYAN}================================================${NC}"
echo ""

# Start the base simulation (do NOT pipe to grep - SIGPIPE can kill script before MAVProxy starts)
echo -e "${YELLOW}[1/3] Starting PX4 simulation (this may take ~2 min)...${NC}"
"$SCRIPT_DIR/start_simulation_simple.sh" || { echo -e "\033[0;31mFailed to start simulation\033[0m"; exit 1; }

# Wait for PX4 to be ready
echo -e "${YELLOW}[2/3] Waiting for PX4 to initialize...${NC}"
sleep 15

# Ensure MAVProxy is running (PX4 GCS is on 18570, not 14540)
echo -e "${YELLOW}Ensuring QGC connection (MAVProxy on PX4 port 18570)...${NC}"
docker exec px4_gazebo_plane pkill -f mavproxy 2>/dev/null || true
sleep 2
# Send UDP to host so QGC on Windows/host can receive
docker exec -d px4_gazebo_plane bash -c "mavproxy.py --master=udp:127.0.0.1:18570 --out=udp:host.docker.internal:14550 --out=tcpin:0.0.0.0:5760 --daemon"
sleep 5
if docker exec px4_gazebo_plane pgrep -f mavproxy >/dev/null 2>&1; then
    echo -e "  ${GREEN}MAVProxy running (18570 -> host:14550, 5760)${NC}"
else
    echo -e "  ${YELLOW}WARNING: MAVProxy may not be running; QGC try UDP 18570 directly${NC}"
fi

# Check if container is running
if ! docker ps | grep -q px4_gazebo_plane; then
    echo -e "${YELLOW}Container not running${NC}"
    exit 1
fi

# Install dependencies if needed
echo -e "${YELLOW}[3/3] Starting sensor data injection...${NC}"
docker exec px4_gazebo_plane bash -c "
    pip3 install pymavlink --quiet 2>/dev/null || {
        apt-get update -qq > /dev/null 2>&1 &&
        apt-get install -y python3-pip --no-install-recommends -qq > /dev/null 2>&1 &&
        pip3 install pymavlink --quiet 2>/dev/null
    }
" 2>/dev/null || true

# Start sensor injection in background
docker exec -d px4_gazebo_plane bash -c "python3 /scripts/inject_sensor_data.py 36.2329 -116.8276 50 > /tmp/sensor_injection.log 2>&1"

sleep 2

# Verify sensor injection is running
if docker exec px4_gazebo_plane bash -c "pgrep -f inject_sensor_data > /dev/null" 2>/dev/null; then
    echo -e "  ${GREEN}✓ Sensor injection running${NC}"
    echo ""
    echo -e "${GREEN}================================================${NC}"
    echo -e "${GREEN}  Setup Complete!${NC}"
    echo -e "${GREEN}================================================${NC}"
    echo ""
    echo "Sensors now available for calibration:"
    echo "  ✓ Accelerometer"
    echo "  ✓ Gyroscope"
    echo "  ✓ Magnetometer"
    echo "  ✓ Barometer"
    echo "  ✓ GPS"
    echo ""
    echo "In QGroundControl:"
    echo "  1. Go to Vehicle Setup → Sensors"
    echo "  2. Calibrate Accelerometer"
    echo "  3. Calibrate Compass (Magnetometer)"
    echo ""
    echo "Sensor injection is running in background"
    echo "To stop: docker exec px4_gazebo_plane pkill -f inject_sensor_data"
    echo ""
    echo -e "${CYAN}QGroundControl connection (try in order):${NC}"
    echo "  1. Comm Links -> Add UDP, Listening Port 14550"
    echo "  2. If no link: Add UDP, Listening Port 18570"
    echo "  3. Or: Add TCP, Host 127.0.0.1, Port 5760"
else
    echo -e "  ${YELLOW}⚠ Sensor injection may not be running${NC}"
    echo "  Check logs: docker exec px4_gazebo_plane cat /tmp/sensor_injection.log"
fi
