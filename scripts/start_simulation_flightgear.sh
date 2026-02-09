#!/bin/bash
# Start PX4 SITL with FlightGear (Robust Fixed-Wing & VTOL)
# FlightGear is excellent for fixed-wing aircraft simulation
# More robust than jMAVSim, supports fixed-wing and VTOL
#
# Usage:
#   ./scripts/start_simulation_flightgear.sh                    # Fixed-wing (default)
#   ./scripts/start_simulation_flightgear.sh --vehicle vtol     # VTOL

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
COMPOSE_FILE="$PROJECT_DIR/docker/simulation/docker-compose.flightgear.yml"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Parse arguments
VEHICLE="${1:-advanced_plane}"  # advanced_plane or standard_vtol
if [ "$VEHICLE" = "vtol" ] || [ "$VEHICLE" = "standard_vtol" ]; then
    VEHICLE="standard_vtol"
    SYS_AUTOSTART=4001
    VEHICLE_NAME="VTOL"
else
    VEHICLE="advanced_plane"
    SYS_AUTOSTART=4008
    VEHICLE_NAME="Fixed-Wing"
fi

export PX4_VEHICLE=$VEHICLE
export SYS_AUTOSTART=$SYS_AUTOSTART

echo -e "${CYAN}================================================${NC}"
echo -e "${CYAN}  PX4 ${VEHICLE_NAME} Simulation (FlightGear)${NC}"
echo -e "${CYAN}  Robust - Excellent for Fixed-Wing${NC}"
echo -e "${CYAN}================================================${NC}"
echo ""

# Stop existing
echo -e "${YELLOW}[1/5] Stopping existing containers...${NC}"
docker compose -f "$COMPOSE_FILE" down 2>/dev/null || true
docker rm -f px4_flightgear 2>/dev/null || true
sleep 2

# Start simulation
echo -e "${YELLOW}[2/5] Starting PX4 SITL with FlightGear...${NC}"
echo -e "  Vehicle: ${VEHICLE_NAME} (${VEHICLE})"
echo -e "  Airframe: SYS_AUTOSTART=${SYS_AUTOSTART}"
export PX4_HOME_LAT=${PX4_HOME_LAT:-36.2329}
export PX4_HOME_LON=${PX4_HOME_LON:--116.8276}
export PX4_HOME_ALT=${PX4_HOME_ALT:-50}

docker compose -f "$COMPOSE_FILE" up -d

echo -e "${YELLOW}[3/5] Waiting for PX4 to initialize...${NC}"
sleep 30

# Wait for PX4 to be ready
MAX_WAIT=180
WAITED=0
READY=false

while [ $WAITED -lt $MAX_WAIT ] && [ "$READY" = false ]; do
    sleep 5
    WAITED=$((WAITED + 5))
    
    if docker logs px4_flightgear 2>&1 | grep -q "pxh>"; then
        READY=true
        echo -e "  ${GREEN}PX4 is ready!${NC}"
    else
        echo -ne "\r  Waiting... ($WAITED seconds)"
    fi
done
echo ""

if [ "$READY" = false ]; then
    echo -e "${YELLOW}WARNING: PX4 may not be fully ready, continuing anyway...${NC}"
fi

# Install and start MAVProxy
echo -e "${YELLOW}[4/5] Setting up MAVLink forwarding for QGroundControl...${NC}"
docker exec px4_flightgear pip3 install mavproxy pymavlink --quiet 2>/dev/null || {
    docker exec px4_flightgear bash -c "
        apt-get update -qq > /dev/null 2>&1 &&
        apt-get install -y python3-pip --no-install-recommends -qq > /dev/null 2>&1 &&
        pip3 install mavproxy pymavlink --quiet 2>/dev/null
    " 2>/dev/null || echo -e "${YELLOW}  WARNING: MAVProxy installation may have failed${NC}"
}

docker exec -d px4_flightgear bash -c "mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:0.0.0.0:14550 --out=tcpin:0.0.0.0:5760 --daemon"
sleep 5

if docker exec px4_flightgear bash -c "pgrep -f mavproxy > /dev/null" 2>/dev/null; then
    echo -e "  ${GREEN}MAVProxy forwarding active${NC}"
else
    echo -e "  ${YELLOW}WARNING: MAVProxy may not be running${NC}"
fi

# Set EKF origin and vehicle position
echo -e "${YELLOW}[5/5] Configuring vehicle position...${NC}"
docker exec px4_flightgear python3 /scripts/fix_ekf_origin_startup.py ${PX4_HOME_LAT:-36.2329} ${PX4_HOME_LON:--116.8276} ${PX4_HOME_ALT:-50} 2>&1 | grep -E "(SUCCESS|ERROR)" || true
docker exec px4_flightgear python3 /scripts/set_vehicle_position.py ${PX4_HOME_LAT:-36.2329} ${PX4_HOME_LON:--116.8276} ${PX4_HOME_ALT:-50} 2>&1 | grep -E "(SUCCESS|ERROR)" || true

echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}  Setup Complete!${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo "Container: px4_flightgear"
echo "Vehicle: ${VEHICLE_NAME} (${VEHICLE})"
echo "Airframe: SYS_AUTOSTART=${SYS_AUTOSTART}"
echo "Simulator: FlightGear (robust, excellent for fixed-wing)"
echo ""
echo "QGroundControl Connection:"
echo "  UDP: udp://127.0.0.1:14550 (auto-detect)"
echo "  TCP: tcp://127.0.0.1:5760 (manual)"
echo ""
echo "This setup includes:"
echo "  ✓ Sensors (IMU, GPS, Barometer)"
echo "  ✓ Airframe configuration (Fixed-wing/VTOL)"
echo "  ✓ Position estimates"
echo "  ✓ Robust physics simulation"
echo ""
echo "To stop: docker compose -f $COMPOSE_FILE down"
echo ""
