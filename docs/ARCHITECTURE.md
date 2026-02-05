# Proxigo Scalence - Autonomous Fixed-Wing UAV Architecture

## Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Components](#hardware-components)
3. [Software Stack](#software-stack)
4. [System Architecture Diagram](#system-architecture-diagram)
5. [Communication Architecture](#communication-architecture)
6. [Development Workflow](#development-workflow)

---

## System Overview

**Proxigo Scalence** is an autonomous fixed-wing UAV platform designed for GPS-denied navigation using Visual Inertial Odometry (VIO) with satellite region referencing. The system is built on PX4, ROS2, and runs on an NVIDIA Orin Nano companion computer with Docker containerization.

### Key Capabilities
- **Visual Inertial Odometry (VIO)** - Position estimation using camera + IMU fusion
- **Satellite Region Referencing** - Pre-downloaded satellite imagery for absolute position correlation
- **Obstacle Avoidance** - Real-time obstacle detection and path planning
- **GPS-Denied Navigation** - Full autonomous operation without GPS dependency
- **Modular Architecture** - Designed for future Visual Positioning System (VPS) hardware module

### Target Configurations
| Configuration | Description | Use Case |
|--------------|-------------|----------|
| Fixed-Wing | Traditional fixed-wing airframe | Long-range, efficient flight |
| VTOL | Vertical takeoff/landing capable | Flexible deployment, payload delivery |

---

## Hardware Components

### Flight Controller
- **PX4 Autopilot** (Pixhawk 6X/6C recommended)
  - Handles low-level flight control
  - MAVLink communication with companion computer
  - Sensor fusion for attitude estimation

### Companion Computer
- **NVIDIA Orin Nano** (8GB Developer Kit)
  - 40 TOPS AI performance
  - 6-core ARM Cortex-A78AE CPU
  - 1024-core NVIDIA Ampere GPU
  - Native MIPI CSI-2 camera support

### Camera System
- **Arducam IMX477 HQ Camera**
  - 12.3MP 1/2.3" Sony IMX477 sensor
  - 6mm CS lens (included)
  - IR-Cut filter for daylight operation
  - MIPI CSI-2 interface (direct Orin connection)
  - Resolution: 4056 × 3040 @ 30fps (full), 1080p @ 60fps

### IMU (External High-Rate)
- **VectorNav VN-100** or **Bosch BMI088** (recommended for VIO)
  - High-rate IMU data (200-400Hz) for VIO
  - Lower noise than flight controller IMU
  - Time-synchronized with camera

### Future Hardware Additions
- **LiDAR Module** - Short-range distance verification (e.g., Garmin LIDAR-Lite v4)
- **Downward Camera** - Terrain-relative navigation
- **RTK GPS** - Ground truth for development/testing

---

## Software Stack

### Layer Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                          │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐   │
│  │   Mission   │ │  Obstacle   │ │  Satellite Region       │   │
│  │   Planner   │ │  Avoidance  │ │  Matching               │   │
│  └─────────────┘ └─────────────┘ └─────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      PERCEPTION LAYER                           │
│  ┌─────────────────────────────┐ ┌─────────────────────────┐   │
│  │  Visual Inertial Odometry   │ │  Feature Extraction     │   │
│  │  (OpenVINS-based)           │ │  & Matching             │   │
│  └─────────────────────────────┘ └─────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      MIDDLEWARE LAYER                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐   │
│  │    ROS2     │ │   MAVROS2   │ │  Camera Driver          │   │
│  │   Humble    │ │             │ │  (v4l2/gstreamer)       │   │
│  └─────────────┘ └─────────────┘ └─────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      SYSTEM LAYER                               │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐   │
│  │   Docker    │ │  JetPack    │ │  PX4 SITL               │   │
│  │  Containers │ │    6.x      │ │  (Simulation)           │   │
│  └─────────────┘ └─────────────┘ └─────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                      HARDWARE LAYER                             │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────────────┐   │
│  │ Orin Nano   │ │  Arducam    │ │  PX4 Flight Controller  │   │
│  │             │ │  IMX477     │ │                         │   │
│  └─────────────┘ └─────────────┘ └─────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Core Software Components

| Component | Technology | Purpose |
|-----------|------------|---------|
| Flight Control | PX4 v1.14+ | Autopilot firmware |
| Middleware | ROS2 Humble | Inter-process communication |
| VIO Engine | OpenVINS (modified) | Visual-inertial state estimation |
| MAVLink Bridge | MAVROS2 | PX4 ↔ ROS2 communication |
| Camera Driver | v4l2_camera / gscam2 | Arducam image capture |
| Containerization | Docker + NVIDIA Container Toolkit | Deployment isolation |
| Simulation | PX4 SITL + jMAVSim/AirSim | Development testing |

---

## System Architecture Diagram

```
                                    ┌──────────────────────┐
                                    │   Ground Station     │
                                    │   (QGroundControl)   │
                                    └──────────┬───────────┘
                                               │ MAVLink (WiFi/Radio)
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ORIN NANO (Docker Host)                          │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                    Docker Container: vio_core                          │ │
│  │  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────┐ │ │
│  │  │  OpenVINS    │◄───│   IMU Data   │    │  Satellite Matching      │ │ │
│  │  │  VIO Node    │    │   Subscriber │    │  Node                    │ │ │
│  │  │              │◄───┤              │    │  ┌────────────────────┐  │ │ │
│  │  │  Outputs:    │    │  Camera      │    │  │ Pre-loaded Sat.    │  │ │ │
│  │  │  - Pose      │◄───│  Subscriber  │    │  │ Imagery Database   │  │ │ │
│  │  │  - Velocity  │    └──────────────┘    │  └────────────────────┘  │ │ │
│  │  │  - Covariance│                        └──────────────────────────┘ │ │
│  │  └──────┬───────┘                                     ▲               │ │
│  │         │ /vio/odom                                   │               │ │
│  │         ▼                                             │               │ │
│  │  ┌──────────────────────────────────────────────────┐ │               │ │
│  │  │              State Estimator Fusion              │ │               │ │
│  │  │  - VIO pose + Satellite absolute correction      │─┘               │ │
│  │  │  - Outputs fused position to PX4                 │                 │ │
│  │  └──────────────────┬───────────────────────────────┘                 │ │
│  └─────────────────────┼────────────────────────────────────────────────┘ │
│                        │                                                   │
│  ┌─────────────────────┼────────────────────────────────────────────────┐ │
│  │  Docker Container: │navigation                                        │ │
│  │  ┌──────────────────▼───────────────┐    ┌────────────────────────┐  │ │
│  │  │         MAVROS2 Node             │    │   Obstacle Avoidance   │  │ │
│  │  │  - /mavros/vision_pose/pose      │◄───│   Node                 │  │ │
│  │  │  - /mavros/setpoint_position     │    │   (Future: LiDAR)      │  │ │
│  │  └──────────────────┬───────────────┘    └────────────────────────┘  │ │
│  └─────────────────────┼────────────────────────────────────────────────┘ │
│                        │ MAVLink (Serial/UDP)                              │
└────────────────────────┼───────────────────────────────────────────────────┘
                         ▼
              ┌──────────────────────┐
              │   PX4 Autopilot      │
              │   (Pixhawk 6X)       │
              │                      │
              │  - EKF2 sensor fusion│
              │  - Flight control    │
              │  - Motor mixing      │
              └──────────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │   Aircraft Systems   │
              │   - ESCs / Motors    │
              │   - Servos           │
              │   - Power System     │
              └──────────────────────┘
```

---

## Communication Architecture

### ROS2 Topic Structure

```yaml
# VIO System Topics
/vio/camera/image_raw          # sensor_msgs/Image - Raw camera frames
/vio/camera/camera_info        # sensor_msgs/CameraInfo - Intrinsics
/vio/imu/data                  # sensor_msgs/Imu - High-rate IMU
/vio/odom                      # nav_msgs/Odometry - VIO output
/vio/pose                      # geometry_msgs/PoseStamped - VIO pose

# Satellite Matching Topics
/sat_match/query_image         # sensor_msgs/Image - Current view
/sat_match/matched_pose        # geometry_msgs/PoseWithCovarianceStamped
/sat_match/confidence          # std_msgs/Float32 - Match confidence

# Fused State Topics
/state/fused_odom              # nav_msgs/Odometry - Final fused state
/state/global_position         # geographic_msgs/GeoPoseStamped

# MAVROS Bridge Topics
/mavros/vision_pose/pose       # geometry_msgs/PoseStamped - To PX4
/mavros/local_position/pose    # geometry_msgs/PoseStamped - From PX4
/mavros/state                  # mavros_msgs/State - FCU state

# Obstacle Avoidance Topics
/obstacle/pointcloud           # sensor_msgs/PointCloud2 (future LiDAR)
/obstacle/trajectory           # mavros_msgs/Trajectory - Avoidance path
```

### MAVLink Message Flow

```
Orin Nano ──► PX4
  VISION_POSITION_ESTIMATE (VIO pose)
  SET_POSITION_TARGET_LOCAL_NED (Commands)
  
PX4 ──► Orin Nano
  ATTITUDE (Aircraft attitude)
  LOCAL_POSITION_NED (EKF estimate)
  HIGHRES_IMU (IMU data backup)
```

---

## Development Workflow

### Dual-Platform Development

```
┌─────────────────────────────────────────────────────────────────┐
│                    DEVELOPMENT ENVIRONMENT                       │
│                                                                  │
│  ┌─────────────────────┐       ┌─────────────────────────────┐  │
│  │   Windows 10 PC     │       │      Orin Nano              │  │
│  │                     │       │                             │  │
│  │  - VS Code + Remote │◄─────►│  - Docker Host              │  │
│  │  - Docker Desktop   │  SSH  │  - Native ARM builds        │  │
│  │  - WSL2 (Ubuntu)    │       │  - Hardware testing         │  │
│  │  - PX4 SITL         │       │  - Camera/IMU access        │  │
│  └─────────────────────┘       └─────────────────────────────┘  │
│           │                                │                     │
│           ▼                                ▼                     │
│  ┌─────────────────────┐       ┌─────────────────────────────┐  │
│  │  Simulation Testing │       │    Hardware-in-Loop         │  │
│  │  - PX4 SITL         │       │    - Real camera feed       │  │
│  │  - jMAVSim/AirSim   │       │    - Real IMU data          │  │
│  │  - Rosbag playback  │       │    - PX4 via MAVLink        │  │
│  └─────────────────────┘       └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Git Workflow
- `main` - Stable releases
- `develop` - Integration branch
- `feature/*` - Feature development
- `simulation/*` - Simulation-specific changes
- `hardware/*` - Hardware-specific adaptations

---

## Next Steps

1. **[Docker Architecture](./DOCKER_ARCHITECTURE.md)** - Container design and orchestration
2. **[VIO System Design](./VIO_SATELLITE_DESIGN.md)** - Visual Inertial Odometry with satellite referencing
3. **[Simulation Setup](./SIMULATION_SETUP.md)** - PX4 SITL without Gazebo
4. **[VPS Module Roadmap](./VPS_MODULE_ROADMAP.md)** - Future standalone hardware module

---

## References

- [OpenVINS](https://github.com/rpng/open_vins) - Visual-inertial navigation research platform
- [Aerial Autonomy Stack](https://github.com/JacopoPan/aerial-autonomy-stack) - UAV autonomy reference
- [Visual Localization](https://github.com/TerboucheHacene/visual_localization) - Visual localization techniques
- [PX4 Documentation](https://docs.px4.io/) - Flight controller documentation
- [MAVROS2](https://github.com/mavlink/mavros/tree/ros2) - MAVLink to ROS2 bridge
