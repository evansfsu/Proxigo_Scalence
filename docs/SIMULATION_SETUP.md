# Simulation Setup Guide

## Overview

This guide covers setting up a PX4 Software-in-the-Loop (SITL) simulation environment **without Gazebo**. The simulation uses jMAVSim (lightweight Java-based simulator) or AirSim for visual testing.

---

## Simulation Options

| Simulator | Pros | Cons | Use Case |
|-----------|------|------|----------|
| **jMAVSim** | Lightweight, fast, no GPU needed | Basic visuals, no camera sim | Algorithm testing, CI/CD |
| **AirSim** | Realistic visuals, camera simulation | Heavy, requires GPU | Full VIO pipeline testing |
| **ROS2 Bag Playback** | Real sensor data, reproducible | No flight control | VIO algorithm development |

---

## Option 1: PX4 SITL with jMAVSim (Recommended for Development)

### Setup on Windows (WSL2)

```powershell
# Install WSL2 Ubuntu 22.04
wsl --install -d Ubuntu-22.04

# Inside WSL2
sudo apt update
sudo apt install -y openjdk-11-jdk ant

# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# Build and run SITL
make px4_sitl_default jmavsim
```

### Using Docker (Cross-Platform)

```bash
# Start PX4 SITL container
docker-compose -f docker-compose.dev.yml --profile simulation up px4_sitl

# In another terminal, start navigation
docker-compose -f docker-compose.dev.yml --profile simulation up navigation_dev
```

### Connect QGroundControl

1. Download QGroundControl from [qgroundcontrol.com](https://qgroundcontrol.com/)
2. Launch QGroundControl
3. It will auto-connect to `localhost:14550`

---

## Option 2: AirSim with PX4 (For VIO Testing)

AirSim provides camera simulation which is essential for testing the VIO pipeline.

### Prerequisites

- Windows 10/11 or Ubuntu 20.04+
- NVIDIA GPU with CUDA support
- Unreal Engine 4.27 (for custom environments)

### Setup Steps

1. **Install AirSim**
```bash
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
./setup.sh
./build.sh
```

2. **Configure for PX4**

Edit `~/Documents/AirSim/settings.json`:
```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "PX4": {
      "VehicleType": "PX4Multirotor",
      "UseSerial": false,
      "LockStep": true,
      "UseTcp": true,
      "TcpPort": 4560,
      "ControlIp": "127.0.0.1",
      "ControlPortLocal": 14540,
      "ControlPortRemote": 14580,
      "Sensors": {
        "Imu": {
          "SensorType": 2,
          "Enabled": true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled": false
        }
      },
      "Cameras": {
        "front_camera": {
          "CaptureSettings": [
            {
              "ImageType": 0,
              "Width": 1920,
              "Height": 1080,
              "FOV_Degrees": 60
            }
          ],
          "X": 0.05, "Y": 0, "Z": -0.02,
          "Pitch": 0, "Roll": 0, "Yaw": 0
        }
      }
    }
  }
}
```

3. **Start PX4 SITL with AirSim**
```bash
cd PX4-Autopilot
make px4_sitl_default none_iris
```

4. **Launch AirSim**
```bash
./Blocks.sh -ResX=1280 -ResY=720 -windowed
```

---

## Option 3: ROS2 Bag Playback (For VIO Development)

Use recorded sensor data to test VIO algorithms without a full simulator.

### Recording Data

```bash
# Start recording
ros2 bag record -a -o flight_data

# Or record specific topics
ros2 bag record \
    /vio/camera/image_raw \
    /vio/imu/data \
    /mavros/local_position/pose \
    -o flight_data
```

### Playback

```bash
# Play recorded data
ros2 bag play flight_data.db3

# With loop
ros2 bag play flight_data.db3 --loop

# With time scaling
ros2 bag play flight_data.db3 --rate 0.5
```

### Sample Datasets

For VIO testing, consider using public datasets:
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [TUM VI Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)

---

## Simulation Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    DEVELOPMENT MACHINE                           │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                    Docker Environment                     │    │
│  │                                                           │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │    │
│  │  │  px4_sitl   │    │  vio_core   │    │ navigation  │  │    │
│  │  │             │◄──►│   _dev      │◄──►│    _dev     │  │    │
│  │  │  jMAVSim    │    │  OpenVINS   │    │   MAVROS2   │  │    │
│  │  └──────┬──────┘    └─────────────┘    └──────┬──────┘  │    │
│  │         │                                      │         │    │
│  │         │ UDP:14540                           │ UDP:14550│    │
│  │         │                                      │         │    │
│  └─────────┼──────────────────────────────────────┼─────────┘    │
│            │                                      │              │
│            ▼                                      ▼              │
│  ┌─────────────────────┐            ┌────────────────────────┐  │
│  │   PX4 Firmware      │            │    QGroundControl      │  │
│  │   (SITL Mode)       │            │    (Ground Station)    │  │
│  └─────────────────────┘            └────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Test Procedures

### 1. Basic Connectivity Test

```bash
# Check MAVROS connection
ros2 topic echo /mavros/state

# Expected output:
# connected: true
# armed: false
# mode: "MANUAL"
```

### 2. Vision Pose Injection Test

```bash
# Publish test vision pose
ros2 topic pub /mavros/vision_pose/pose geometry_msgs/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
" --rate 30
```

### 3. Arm and Takeoff Test

```bash
# Set mode to OFFBOARD
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

---

## PX4 Parameters for Simulation

Configure these parameters in PX4 for vision-based navigation:

```bash
# In QGroundControl or via MAVLink
param set EKF2_EV_CTRL 15      # Enable vision pos/vel/yaw
param set EKF2_HGT_REF 3       # Height from vision
param set EKF2_GPS_CTRL 0      # Disable GPS (GPS-denied mode)
param set COM_ARM_WO_GPS 1     # Allow arming without GPS
```

---

## Troubleshooting

### jMAVSim Won't Start

```bash
# Check Java version
java -version  # Should be 11+

# Install dependencies
sudo apt install ant openjdk-11-jdk

# Rebuild
cd PX4-Autopilot
make clean
make px4_sitl_default jmavsim
```

### MAVROS Can't Connect

```bash
# Check PX4 is running
netstat -tuln | grep 14540

# Check firewall (WSL2)
# Ensure UDP 14540, 14550, 14560 are accessible

# Try explicit connection
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
```

### Vision Pose Not Fused

```bash
# Check EKF2 parameters
param show EKF2_EV_*

# Monitor EKF2 status
listener estimator_status

# Check vision pose is being received
ros2 topic hz /mavros/vision_pose/pose
```

---

## CI/CD Integration

Example GitHub Actions workflow for automated testing:

```yaml
# .github/workflows/simulation-test.yml
name: Simulation Test

on: [push, pull_request]

jobs:
  sitl-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      
      - name: Build Docker images
        run: ./scripts/build_all.sh amd64
        
      - name: Start simulation
        run: |
          docker-compose -f docker-compose.dev.yml up -d px4_sitl
          sleep 30  # Wait for SITL to initialize
          
      - name: Run tests
        run: |
          docker-compose -f docker-compose.dev.yml run vio_core_dev \
            ros2 test vio_core --verbose
            
      - name: Cleanup
        run: docker-compose -f docker-compose.dev.yml down
```
