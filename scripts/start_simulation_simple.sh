#!/bin/bash
# Simplified PX4 SITL startup - reverted to working method
# Based on the confirmed working start_simulation.ps1 approach

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}================================================${NC}"
echo -e "${CYAN}  PX4 Fixed-Wing Simulation Startup${NC}"
echo -e "${CYAN}================================================${NC}"
echo ""

# Step 1: Stop any existing containers
echo -e "${YELLOW}[1/6] Stopping existing containers...${NC}"
docker rm -f px4_gazebo_plane gazebo_gui 2>/dev/null || true
sleep 2

# Step 2: Start PX4 SITL container (direct docker run, not compose)
echo -e "${YELLOW}[2/6] Starting PX4 SITL (this takes ~2 minutes on first run)...${NC}"

# Detect WSL2 vs Linux
if grep -qi microsoft /proc/version 2>/dev/null; then
    # WSL2: Get Windows host IP
    WIN_HOST=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
    DISPLAY_VAL="${WIN_HOST}:0"
else
    # Linux: Use local display
    DISPLAY_VAL=":0"
    xhost +local:docker 2>/dev/null || true
fi

docker run -d \
    --name px4_gazebo_plane \
    -p 5760:5760 \
    -p 14540:14540/udp \
    -p 14550:14550/udp \
    -p 18570:18570/udp \
    -e DISPLAY=${DISPLAY_VAL} \
    -e PX4_HOME_LAT=36.2329 \
    -e PX4_HOME_LON=-116.8276 \
    -e PX4_HOME_ALT=50 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v "${PROJECT_DIR}/satellite_data:/satellite_data:ro" \
    -v "${PROJECT_DIR}/scripts:/scripts:ro" \
    -v "${PROJECT_DIR}/config/mission:/mission:ro" \
    proxigo/px4-gazebo:harmonic \
    bash -c "cd /root/PX4-Autopilot && export PX4_HOME_LAT=36.2329 && export PX4_HOME_LON=-116.8276 && export PX4_HOME_ALT=50 && make px4_sitl none"

if [ $? -ne 0 ]; then
    echo -e "\033[0;31mERROR: Failed to start PX4 container\033[0m"
    exit 1
fi

# Step 3: Wait for PX4 to be ready
echo -e "${YELLOW}[3/6] Waiting for PX4 to initialize...${NC}"
MAX_WAIT=180
WAITED=0
READY=false

while [ $WAITED -lt $MAX_WAIT ] && [ "$READY" = false ]; do
    sleep 10
    WAITED=$((WAITED + 10))
    
    if docker logs --tail 5 px4_gazebo_plane 2>&1 | grep -q "pxh>"; then
        READY=true
        echo -e "  ${GREEN}PX4 is ready!${NC}"
    else
        echo -e "  Waiting... ($WAITED seconds, checking for PX4 shell)"
    fi
done

if [ "$READY" = false ]; then
    echo -e "${YELLOW}WARNING: PX4 may not be fully ready, continuing anyway...${NC}"
fi

# Step 4: Install and start MAVProxy for QGC connection
echo -e "${YELLOW}[4/6] Setting up MAVLink forwarding for QGroundControl...${NC}"
docker exec px4_gazebo_plane pip3 install mavproxy pymavlink --quiet 2>/dev/null || {
    docker exec px4_gazebo_plane bash -c "
        apt-get update -qq > /dev/null 2>&1 &&
        apt-get install -y python3-pip --no-install-recommends -qq > /dev/null 2>&1 &&
        pip3 install mavproxy pymavlink --quiet 2>/dev/null
    " 2>/dev/null || echo -e "${YELLOW}  WARNING: MAVProxy installation may have failed${NC}"
}

# Start MAVProxy to forward from PX4's offboard port
# For WSL2, use 0.0.0.0 which maps to host via port mapping
docker exec -d px4_gazebo_plane bash -c "mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:0.0.0.0:14550 --out=tcpin:0.0.0.0:5760 --daemon"
sleep 5

# Verify MAVProxy is running
if docker exec px4_gazebo_plane bash -c "ps aux | grep mavproxy | grep -v grep" 2>&1 | grep -q mavproxy; then
    echo -e "  ${GREEN}MAVProxy forwarding active${NC}"
else
    echo -e "  ${YELLOW}WARNING: MAVProxy may not be running${NC}"
fi

# Step 5: Skip Gazebo GUI for now (can add later if needed)
echo -e "${YELLOW}[5/6] Gazebo GUI skipped (can be added separately)${NC}"

# Step 6: Connection instructions
echo -e "${GREEN}[6/6] Setup complete!${NC}"
echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}  QGroundControl Connection Options${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo -e "${YELLOW}OPTION 1 - UDP (Try First):${NC}"
echo "  1. Open QGroundControl"
echo "  2. Q icon -> Application Settings -> Comm Links"
echo "  3. Delete all existing links"
echo "  4. Add new: Type=UDP, Port=14550"
echo "  5. Connect"
echo ""
echo -e "${YELLOW}OPTION 2 - TCP (If UDP fails):${NC}"
echo "  1. Add new: Type=TCP, Host=127.0.0.1, Port=5760"
echo ""
echo -e "${YELLOW}Mission file location:${NC}"
echo "  ${PROJECT_DIR}/config/mission/death_valley_simple.plan"
echo ""
echo -e "To stop simulation: docker rm -f px4_gazebo_plane gazebo_gui"
echo ""
