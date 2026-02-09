#!/bin/bash
# Start PX4 SITL and MAVProxy bridge together
# This is the proven working method from start_simulation.ps1

set -e

cd /root/PX4-Autopilot

# Export PX4 home position
export PX4_HOME_LAT=${PX4_HOME_LAT:-36.2329}
export PX4_HOME_LON=${PX4_HOME_LON:--116.8276}
export PX4_HOME_ALT=${PX4_HOME_ALT:-0}

# Get vehicle model from environment (set by docker-compose)
VEHICLE=${PX4_VEHICLE:-advanced_plane}
PX4_SIM_MODEL="gz_${VEHICLE}"

echo "Starting PX4 SITL with ${PX4_SIM_MODEL}..."
echo "Home: ${PX4_HOME_LAT}, ${PX4_HOME_LON}, ${PX4_HOME_ALT}"

# Start PX4 in background
make px4_sitl ${PX4_SIM_MODEL} &
PX4_PID=$!

# Wait for PX4 to initialize (give it time to start)
echo "Waiting for PX4 to initialize..."
sleep 45

# Install MAVProxy
echo "Installing MAVProxy..."
pip3 install mavproxy pymavlink --quiet 2>/dev/null || {
    echo "Installing pip3 first..."
    apt-get update -qq > /dev/null 2>&1
    apt-get install -y python3-pip --no-install-recommends -qq > /dev/null 2>&1
    pip3 install mavproxy pymavlink --quiet 2>/dev/null || {
        echo "WARNING: MAVProxy installation failed, continuing without bridge"
        wait $PX4_PID
        exit 1
    }
}

# Start MAVProxy bridge (proven working method)
echo "Starting MAVProxy bridge..."
echo "  Master: udp:127.0.0.1:14540 (PX4 offboard)"
echo "  Output: udp:0.0.0.0:14550 (QGC UDP)"
echo "  Output: tcpin:0.0.0.0:5760 (QGC TCP)"

mavproxy.py \
    --master=udp:127.0.0.1:14540 \
    --out=udp:0.0.0.0:14550 \
    --out=tcpin:0.0.0.0:5760 \
    --daemon

sleep 2

# Verify MAVProxy is running
if pgrep -f mavproxy > /dev/null; then
    echo "✓ MAVProxy bridge is running"
else
    echo "✗ MAVProxy failed to start"
fi

# Wait for PX4 (keeps container alive)
wait $PX4_PID
