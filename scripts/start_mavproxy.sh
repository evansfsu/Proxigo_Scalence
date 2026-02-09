#!/bin/bash
# Start MAVProxy bridge inside PX4 container
# This script waits for PX4 to be ready, then starts MAVProxy

set -e

echo "Waiting for PX4 to be ready..."
max_wait=180
waited=0

while [ $waited -lt $max_wait ]; do
    if pgrep -f px4 > /dev/null; then
        echo "PX4 is running, waiting a bit more for initialization..."
        sleep 10
        break
    fi
    sleep 2
    waited=$((waited + 2))
done

echo "Installing MAVProxy..."
pip3 install mavproxy pymavlink --quiet 2>/dev/null || {
    echo "WARNING: Failed to install MAVProxy, trying with apt-get..."
    apt-get update -qq && apt-get install -y python3-pip --no-install-recommends -qq > /dev/null 2>&1
    pip3 install mavproxy pymavlink --quiet 2>/dev/null || echo "WARNING: MAVProxy installation failed"
}

echo "Starting MAVProxy bridge..."
echo "  Master: udp:127.0.0.1:14540 (PX4 offboard)"
echo "  Output: udp:0.0.0.0:14550 (QGC UDP)"
echo "  Output: tcpin:0.0.0.0:5760 (QGC TCP)"

# Start MAVProxy in background
mavproxy.py \
    --master=udp:127.0.0.1:14540 \
    --out=udp:0.0.0.0:14550 \
    --out=tcpin:0.0.0.0:5760 \
    --daemon

sleep 2

# Verify it's running
if pgrep -f mavproxy > /dev/null; then
    echo "✓ MAVProxy bridge is running"
else
    echo "✗ MAVProxy failed to start"
    exit 1
fi

# Keep script running
sleep infinity
