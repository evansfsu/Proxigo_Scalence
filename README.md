# Proxigo Scalence

**Autonomous Fixed-Wing UAV Platform with GPS-Denied Navigation**

An open-source software architecture for autonomous fixed-wing UAV navigation using Visual Inertial Odometry (VIO) with satellite region referencing, designed for GPS-denied environments.

---

## Overview

Proxigo Scalence combines:
- **Visual Inertial Odometry (VIO)** - Real-time position estimation using camera + IMU
- **Satellite Region Referencing** - Absolute position correction from pre-downloaded satellite imagery
- **PX4 Autopilot Integration** - Full autonomous flight control
- **Docker Containerization** - Portable deployment on NVIDIA Orin Nano

### Key Features

| Feature | Description |
|---------|-------------|
| GPS-Denied Navigation | Full autonomous operation without GPS dependency |
| Satellite Matching | Correlate aerial views with satellite imagery for absolute positioning |
| Fixed-Wing Optimized | Designed for high-speed, high-altitude fixed-wing flight |
| VTOL Compatible | Architecture supports VTOL configurations |
| Modular Design | Each component runs in isolated Docker containers |
| Cross-Platform Development | Develop on Windows 10, deploy to Orin Nano |

---

## Hardware Requirements

### Companion Computer
- **NVIDIA Orin Nano** (8GB Developer Kit)
- JetPack 6.x

### Camera
- **Arducam IMX477 HQ Camera**
- 12.3MP Sony IMX477 sensor
- 6mm CS lens
- [Product Link](https://www.arducam.com/arducam-high-quality-ir-cut-camera-for-jetson-nano-xavier-nx-12-3mp-1-2-3-inch-imx477-hq-camera-module-with-6mm-cs-lens.html)

### Flight Controller
- **PX4 Autopilot** (Pixhawk 6X/6C recommended)
- UART connection to Orin Nano

### IMU (Optional External)
- High-rate IMU (200-400Hz) for improved VIO performance
- VectorNav VN-100 or Bosch BMI088 recommended

---

## Software Stack

```
┌─────────────────────────────────────────────────────┐
│                 APPLICATION LAYER                    │
│  Mission Planner │ Obstacle Avoidance │ Sat Match   │
├─────────────────────────────────────────────────────┤
│                 PERCEPTION LAYER                     │
│  OpenVINS VIO │ Feature Extraction │ Image Matching │
├─────────────────────────────────────────────────────┤
│                 MIDDLEWARE LAYER                     │
│  ROS2 Humble │ MAVROS2 │ Camera Drivers             │
├─────────────────────────────────────────────────────┤
│                  SYSTEM LAYER                        │
│  Docker │ JetPack 6.x │ PX4 SITL (Simulation)       │
└─────────────────────────────────────────────────────┘
```

---

## Quick Start

### Prerequisites

```bash
# On Orin Nano
sudo apt update
sudo apt install docker.io docker-compose nvidia-container-toolkit

# On Windows (for development)
# Install Docker Desktop with WSL2 backend
```

### Clone Repository

```bash
git clone https://github.com/your-org/Proxigo_Scalence.git
cd Proxigo_Scalence
```

### Build Docker Images

```bash
# On Orin Nano (Jetson)
./scripts/build_all.sh jetson

# On Windows/Linux (for development/simulation)
./scripts/build_all.sh amd64
```

### Run System

```bash
# Production (on Orin Nano)
docker-compose up -d

# Development/Simulation
docker-compose -f docker-compose.dev.yml --profile simulation up -d
```

---

## Project Structure

```
Proxigo_Scalence/
├── docs/                           # Documentation
│   ├── ARCHITECTURE.md             # System architecture
│   ├── DOCKER_ARCHITECTURE.md      # Container design
│   ├── VIO_SATELLITE_DESIGN.md     # VIO + satellite matching
│   └── VPS_MODULE_ROADMAP.md       # Future hardware module
├── docker/                         # Dockerfiles
│   ├── base/                       # Base ROS2 image
│   ├── vio_core/                   # VIO container
│   ├── navigation/                 # MAVROS container
│   └── satellite_match/            # Satellite matching
├── config/                         # Configuration files
│   ├── camera/                     # Camera calibration
│   ├── vio/                        # OpenVINS parameters
│   ├── navigation/                 # MAVROS config
│   └── mission/                    # Mission parameters
├── scripts/                        # Helper scripts
│   ├── build_all.sh                # Build Docker images
│   ├── deploy_orin.sh              # Deploy to Orin
│   ├── calibrate_camera.sh         # Camera calibration
│   └── prepare_satellite_data.py   # Satellite preprocessing
├── src/                            # Source code (ROS2 packages)
├── satellite_data/                 # Pre-downloaded satellite imagery
├── docker-compose.yml              # Production deployment
└── docker-compose.dev.yml          # Development/simulation
```

---

## Configuration

### 1. Camera Calibration

Before first flight, calibrate the camera intrinsics:

```bash
# Inside vio_core container
./scripts/calibrate_camera.sh 9x6 0.025
```

Update `config/camera/arducam_imx477.yaml` with calibration results.

### 2. Initial Position

Set your takeoff coordinates in `config/mission/initial_pose.yaml`:

```yaml
initial_position:
  latitude: 39.678123      # Your latitude
  longitude: -75.750456    # Your longitude
  altitude_msl: 50.0       # Altitude MSL
  heading_deg: 90.0        # Heading (0=North)
```

### 3. Satellite Data

Prepare satellite imagery for your mission area:

```bash
python scripts/prepare_satellite_data.py \
    mission_area.tif \
    satellite_data/regions/my_mission \
    --resolution 0.3 \
    --altitudes 50,100,200,400
```

---

## Documentation

- **[System Architecture](docs/ARCHITECTURE.md)** - Complete system design
- **[Docker Architecture](docs/DOCKER_ARCHITECTURE.md)** - Container structure
- **[VIO + Satellite Design](docs/VIO_SATELLITE_DESIGN.md)** - Algorithm details
- **[VPS Module Roadmap](docs/VPS_MODULE_ROADMAP.md)** - Future standalone module

---

## Simulation

Run PX4 SITL simulation without Gazebo:

```bash
# Start simulation stack
docker-compose -f docker-compose.dev.yml --profile simulation up -d

# Connect QGroundControl to localhost:14550
```

---

## Development

### Windows 10 Development Setup

1. Install WSL2 with Ubuntu 22.04
2. Install Docker Desktop with WSL2 backend
3. Install VS Code with Remote - WSL extension
4. Clone repository in WSL2

```powershell
wsl --install -d Ubuntu-22.04
```

### Deploy to Orin Nano

```bash
# From development machine
./scripts/deploy_orin.sh orin.local nvidia
```

---

## Roadmap

### Phase 1: Software Validation (Current)
- [x] Architecture documentation
- [x] ROS2 package structure (vio_bridge, satellite_matching, state_fusion)
- [x] Camera/IMU simulators for testing
- [x] Satellite matching algorithm
- [x] State fusion EKF
- [x] PX4/MAVROS integration
- [ ] OpenVINS integration
- [ ] Full system testing

### Phase 2: Flight Testing
- [ ] Hardware integration on Orin Nano
- [ ] Camera calibration
- [ ] First flight tests
- [ ] Performance tuning

### Phase 3: VPS Module
- [ ] Standalone hardware design
- [ ] Custom carrier board
- [ ] Production module

See [VPS Module Roadmap](docs/VPS_MODULE_ROADMAP.md) for details.

---

## ROS2 Packages

| Package | Description |
|---------|-------------|
| `vio_bridge` | VIO interface, camera/IMU simulators |
| `satellite_matching` | Aerial-to-satellite image matching |
| `state_fusion` | VIO + satellite fusion EKF |
| `proxigo_bringup` | Main launch files |

### Building Packages

```bash
# Inside Docker container
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running Simulation

```bash
# Start simulation with camera/IMU simulators
ros2 launch proxigo_bringup simulation.launch.py

# Or run individual nodes
ros2 run vio_bridge camera_simulator
ros2 run vio_bridge imu_simulator
ros2 run satellite_matching satellite_matcher_node
ros2 run state_fusion fusion_node
```

---

## References

### Core Technologies
- [OpenVINS](https://github.com/rpng/open_vins) - Visual-inertial navigation
- [PX4 Autopilot](https://px4.io/) - Flight control
- [ROS2 Humble](https://docs.ros.org/en/humble/) - Robot middleware
- [MAVROS2](https://github.com/mavlink/mavros) - MAVLink bridge

### Reference Projects
- [Aerial Autonomy Stack](https://github.com/JacopoPan/aerial-autonomy-stack)
- [Visual Localization](https://github.com/TerboucheHacene/visual_localization)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Contributing

Contributions are welcome! Please read our contributing guidelines before submitting PRs.

---

## Contact

For questions or collaboration inquiries, please open an issue on GitHub.
