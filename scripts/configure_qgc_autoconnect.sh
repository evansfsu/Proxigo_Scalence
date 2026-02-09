#!/bin/bash
# Configure QGroundControl Auto-Connect (Linux/WSL version)
# Based on QGC container documentation

set -e

CONNECTION_TYPE="${1:-UDP}"
PORT="${2:-14550}"
HOST="${3:-127.0.0.1}"

echo "================================================"
echo "  QGroundControl Auto-Connect Configuration"
echo "================================================"
echo ""

# QGC settings location (Linux)
QGC_SETTINGS_DIR="$HOME/.config/QGroundControl.org"
QGC_SETTINGS_FILE="$QGC_SETTINGS_DIR/QGroundControl.ini"

# Create settings directory if it doesn't exist
mkdir -p "$QGC_SETTINGS_DIR"

echo "[INFO] QGC Settings: $QGC_SETTINGS_FILE"

# Configure connection
if [ "$CONNECTION_TYPE" = "UDP" ]; then
    LINK_NAME="PX4_SITL_UDP"
    LINK_STRING="UDP:$PORT"
    echo "[INFO] Configuring UDP connection on port $PORT"
else
    LINK_NAME="PX4_SITL_TCP"
    LINK_STRING="TCP:$HOST:$PORT"
    echo "[INFO] Configuring TCP connection to $HOST:$PORT"
fi

# Create/update CommLinks section
if [ -f "$QGC_SETTINGS_FILE" ]; then
    # Remove existing link if present
    sed -i "/^\[CommLinks\]/,/^\[/ { /^$LINK_NAME=/d; /^${LINK_NAME}_AutoConnect=/d; /^${LINK_NAME}_HighLatency=/d; }" "$QGC_SETTINGS_FILE" 2>/dev/null || true
    echo "[INFO] Updated existing QGC settings"
else
    echo "[INFO] Creating new QGC settings file"
fi

# Add CommLinks section if it doesn't exist
if ! grep -q "^\[CommLinks\]" "$QGC_SETTINGS_FILE" 2>/dev/null; then
    echo "" >> "$QGC_SETTINGS_FILE"
    echo "[CommLinks]" >> "$QGC_SETTINGS_FILE"
fi

# Add connection configuration
{
    echo "$LINK_NAME=$LINK_STRING"
    echo "${LINK_NAME}_AutoConnect=true"
    echo "${LINK_NAME}_HighLatency=false"
} >> "$QGC_SETTINGS_FILE"

echo "[OK] QGC settings configured successfully!"
echo ""
echo "Connection configured:"
echo "  Type: $CONNECTION_TYPE"
[ "$CONNECTION_TYPE" = "TCP" ] && echo "  Host: $HOST"
echo "  Port: $PORT"
echo "  Auto Connect: Enabled"
echo ""
echo "Next steps:"
echo "  1. Open QGroundControl"
echo "  2. QGC should automatically connect to PX4 SITL"
echo ""
