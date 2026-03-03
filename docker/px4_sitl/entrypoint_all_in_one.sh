#!/bin/bash
# Run PX4 SITL + Gazebo + MAVROS (+ optional ROS2 nodes) in a single container.
# All processes use localhost; no inter-container networking.

set -e

export PX4_HOME_LAT=${PX4_HOME_LAT:-36.2329}
export PX4_HOME_LON=${PX4_HOME_LON:--116.8276}
export PX4_HOME_ALT=${PX4_HOME_ALT:-50}
export HEADLESS=${HEADLESS:-1}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Optional: run VIO/satellite/fusion (set RUN_ROS2_NODES=1 and mount src to /ros2_ws/src)
RUN_ROS2_NODES=${RUN_ROS2_NODES:-0}

echo "[all-in-one] PX4 home: $PX4_HOME_LAT, $PX4_HOME_LON, $PX4_HOME_ALT"
echo "[all-in-one] Starting PX4 SITL + Gazebo Harmonic (headless=$HEADLESS)..."

# 1) Start PX4 + Gazebo in background (same process: make px4_sitl gz_*)
cd /root/PX4-Autopilot
export PX4_HOME_LAT PX4_HOME_LON PX4_HOME_ALT
if [ "$HEADLESS" = "1" ]; then
  make px4_sitl gz_advanced_plane &
else
  make px4_sitl gz_advanced_plane &
fi
PX4_PID=$!

# Wait for PX4 to be ready (process + a few seconds for MAVLink)
echo "[all-in-one] Waiting for PX4 to be ready..."
for i in $(seq 1 60); do
  if pgrep -f px4 >/dev/null 2>&1; then
    sleep 10
    if pgrep -f px4 >/dev/null 2>&1; then
      echo "[all-in-one] PX4 is up."
      break
    fi
  fi
  sleep 2
done

# 2) MAVProxy: listen on 14550 (UDP) and 5760 (TCP) so QGC can connect via port mapping
if command -v mavproxy.py >/dev/null 2>&1; then
  echo "[all-in-one] Starting MAVProxy (udpin:14550, tcpin:5760) for QGC..."
  mavproxy.py --master=udp:127.0.0.1:18570 --out=udpin:0.0.0.0:14550 --out=tcpin:0.0.0.0:5760 --daemon &
  sleep 3
fi

# 3) MAVROS (ROS2 <-> PX4) - use px4.launch (not .py); give PX4 time to open MAVLink port
echo "[all-in-one] Starting MAVROS..."
source /opt/ros/humble/setup.bash
sleep 5
if [ -f /opt/ros/humble/share/mavros/launch/px4.launch.py ]; then
  ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@localhost:14557 &
elif [ -f /opt/ros/humble/share/mavros/launch/px4.launch ]; then
  ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557 &
else
  echo "[all-in-one] WARNING: MAVROS px4 launch not found, skipping"
fi
sleep 5

# 4) Optional: build and run VIO / satellite / fusion from workspace
if [ "$RUN_ROS2_NODES" = "1" ] && [ -d /ros2_ws/src ] && [ -n "$(ls -A /ros2_ws/src 2>/dev/null)" ]; then
  echo "[all-in-one] Building and launching ROS2 nodes..."
  cd /ros2_ws
  if [ ! -f install/setup.bash ]; then
    colcon build --symlink-install 2>/dev/null || true
  fi
  if [ -f install/setup.bash ]; then
    source install/setup.bash
    ros2 launch vio_bridge vio_bridge.launch.py sim_mode:=true &
    sleep 3
    ros2 launch satellite_matching satellite_matching.launch.py 2>/dev/null &
    ros2 launch state_fusion state_fusion.launch.py 2>/dev/null &
  fi
fi

echo "[all-in-one] All services started. PX4 PID=$PX4_PID. Use Ctrl+C to stop."
wait $PX4_PID
