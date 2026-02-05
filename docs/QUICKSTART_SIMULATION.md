# Quickstart: Simulation Testing on Windows

## Current Status

Your development environment is set up with:
- ✅ Docker Desktop running
- ✅ `proxigo/ros2-base:dev` - Base ROS2 Humble image
- ✅ `proxigo/navigation:dev` - MAVROS2 for PX4 communication
- ✅ Test satellite data created (`satellite_data/regions/test_region/`)

## Testing Options

### Option 1: Quick ROS2/MAVROS Test (No PX4 Required)

This tests that ROS2 and MAVROS are working correctly:

```powershell
# Terminal 1: Start development shell
cd C:\Users\eschn\OneDrive\Documents\GitHub\Proxigo_Scalence
docker compose -f docker-compose.dev.yml run --rm dev_shell bash

# Inside container:
ros2 topic list
ros2 node list
```

```powershell
# Terminal 2: Start MAVROS (will wait for PX4 connection)
cd C:\Users\eschn\OneDrive\Documents\GitHub\Proxigo_Scalence
docker compose -f docker-compose.dev.yml run --rm navigation_dev bash

# Inside container:
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@localhost:14557

# MAVROS will show "FCU: not connected" - this is expected without PX4
# Press Ctrl+C to stop
```

### Option 2: QGroundControl with UDP Connection

1. Download [QGroundControl](https://qgroundcontrol.com/downloads/)
2. Install and launch QGroundControl
3. QGroundControl includes a built-in simulation mode that can test basic MAVLink communication

### Option 3: Full PX4 SITL (Advanced)

For full PX4 Software-in-the-Loop simulation, you need to build PX4 from source:

```powershell
# This runs PX4 SITL inside Docker (first time takes 20-30 minutes to build)
cd C:\Users\eschn\OneDrive\Documents\GitHub\Proxigo_Scalence

# Start PX4 SITL build container
docker run -it --rm --network host px4io/px4-dev-simulation-jammy:latest bash

# Inside the container:
cd /root
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl_default none_iris

# This will start PX4 SITL with no simulator (headless)
# It listens on UDP port 14540 for MAVLink connections
```

Then in another terminal:
```powershell
# Start MAVROS to connect to PX4
docker compose -f docker-compose.dev.yml run --rm navigation_dev \
  ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@localhost:14557
```

## Recommended Development Flow

Since full PX4 SITL is time-consuming to set up, here's the recommended approach:

### 1. Develop VIO and Satellite Matching First

Work on the VIO and satellite matching algorithms using recorded data:

```powershell
# Start development shell
docker compose -f docker-compose.dev.yml run -it dev_shell bash

# Inside container - you have:
# - ROS2 Humble
# - OpenCV, NumPy, etc.
# - Your source code mounted at /ros2_ws/src
# - Satellite data at /satellite_data
# - Config at /ros2_ws/config
```

### 2. Test with ROS2 Bag Files

Download sample VIO datasets (EuRoC, TUM-VI) and replay them:

```bash
# Inside dev_shell container
ros2 bag play /test_data/euroc_mh01.db3 --loop
```

### 3. Integrate with PX4 Later

Once VIO is working, integrate with PX4 SITL for full system testing.

## File Locations in Containers

| Host Path | Container Path | Description |
|-----------|----------------|-------------|
| `./src` | `/ros2_ws/src` | Your ROS2 packages |
| `./config` | `/ros2_ws/config` | Configuration files |
| `./satellite_data` | `/satellite_data` | Satellite imagery |
| `./logs` | `/ros2_ws/logs` | Log files |
| `./test_data` | `/test_data` | Test datasets |

## Next Development Steps

1. **Create VIO Bridge Package**
   ```bash
   cd /ros2_ws/src
   ros2 pkg create vio_bridge --build-type ament_python
   ```

2. **Create Satellite Matching Package**
   ```bash
   ros2 pkg create satellite_matching --build-type ament_python
   ```

3. **Create State Fusion Package**
   ```bash
   ros2 pkg create state_fusion --build-type ament_python
   ```

4. **Build and Test**
   ```bash
   cd /ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Useful ROS2 Commands

```bash
# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /vio/odom

# Check topic frequency
ros2 topic hz /vio/odom

# List all nodes
ros2 node list

# Get node info
ros2 node info /mavros

# List available parameters
ros2 param list
```
