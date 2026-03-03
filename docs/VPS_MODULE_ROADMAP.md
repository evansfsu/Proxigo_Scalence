# Visual Positioning System (VPS) Module Roadmap

## Vision

Create a **standalone hardware module** that combines IMU, Camera, and Compute to estimate position in GPS-denied environments. The VPS module should be plug-and-play compatible with both **PX4** (priority) and **ArduPilot** autopilots.

---

## Product Concept

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    VISUAL POSITIONING SYSTEM MODULE                         │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        HARDWARE ENCLOSURE                             │  │
│  │   ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────────┐    │  │
│  │   │  Camera   │  │    IMU    │  │  Compute  │  │   Connectors  │    │  │
│  │   │  Module   │  │   Module  │  │  Module   │  │               │    │  │
│  │   │           │  │           │  │           │  │  • UART/MAVLink│   │  │
│  │   │ IMX477 or │  │ BMI088 or │  │ Orin NX   │  │  • USB-C      │    │  │
│  │   │ OV9281    │  │ VN-100   │  │ or Nano   │  │  • Power In   │    │  │
│  │   │           │  │           │  │           │  │  • SD Card    │    │  │
│  │   └───────────┘  └───────────┘  └───────────┘  └───────────────┘    │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                         SOFTWARE STACK                                │  │
│  │   ┌─────────────┐  ┌─────────────┐  ┌──────────────────────────┐    │  │
│  │   │  OpenVINS   │  │  Satellite  │  │  MAVLink Interface       │    │  │
│  │   │    VIO      │→ │  Matching   │→ │  (VISION_POSITION_ESTIMATE)│  │  │
│  │   │   Engine    │  │   Engine    │  │                          │    │  │
│  │   └─────────────┘  └─────────────┘  └──────────────────────────┘    │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  OUTPUT: Position, Velocity, Attitude @ 50Hz via MAVLink                    │
│  INPUT:  Initial coordinates, satellite imagery (SD card)                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Development Phases

### Phase 1: Software Validation (Current)
**Timeline: 3-6 months**

| Milestone | Description | Status |
|-----------|-------------|--------|
| 1.1 | Basic VIO with OpenVINS on Orin Nano | 🔄 In Progress |
| 1.2 | Satellite matching algorithm | 📋 Planned |
| 1.3 | Fusion layer implementation | 📋 Planned |
| 1.4 | PX4 integration testing | 📋 Planned |
| 1.5 | Fixed-wing flight validation | 📋 Planned |

**Deliverables:**
- Working software stack in Docker containers
- Validated accuracy metrics (< 5m position error)
- Documentation for users

---

### Phase 2: Hardware Consolidation
**Timeline: 6-9 months**

| Milestone | Description | Status |
|-----------|-------------|--------|
| 2.1 | Camera module selection (IMX477 vs OV9281) | 📋 Planned |
| 2.2 | IMU selection and integration | 📋 Planned |
| 2.3 | Compute module selection (Orin Nano vs Orin NX) | 📋 Planned |
| 2.4 | Custom carrier board design | 📋 Planned |
| 2.5 | Thermal management design | 📋 Planned |

**Hardware Candidates:**

| Component | Option A | Option B | Decision Criteria |
|-----------|----------|----------|-------------------|
| Camera | Arducam IMX477 | OV9281 Global Shutter | Motion blur sensitivity |
| IMU | BMI088 | VectorNav VN-100 | Cost vs accuracy |
| Compute | Orin Nano 8GB | Orin NX 16GB | Power vs performance |
| Interface | UART MAVLink | CAN + UART | Bandwidth requirements |

---

### Phase 3: Standalone Module Development
**Timeline: 9-15 months**

| Milestone | Description | Status |
|-----------|-------------|--------|
| 3.1 | PCB design for carrier board | 📋 Planned |
| 3.2 | Enclosure design (3D printed → injection) | 📋 Planned |
| 3.3 | Firmware optimization | 📋 Planned |
| 3.4 | MAVLink plugin for PX4/ArduPilot | 📋 Planned |
| 3.5 | User configuration interface (web/app) | 📋 Planned |

---

### Phase 4: Production & Certification
**Timeline: 15-24 months**

| Milestone | Description | Status |
|-----------|-------------|--------|
| 4.1 | Beta testing with partners | 📋 Planned |
| 4.2 | FCC/CE certification | 📋 Planned |
| 4.3 | Manufacturing setup | 📋 Planned |
| 4.4 | Documentation & support | 📋 Planned |
| 4.5 | Initial product release | 📋 Planned |

---

## Technical Specifications (Target)

### Hardware Specifications

| Specification | Target Value |
|--------------|--------------|
| Dimensions | 80 × 60 × 30 mm |
| Weight | < 100g |
| Power consumption | < 15W typical, < 25W peak |
| Input voltage | 5V DC (USB-C PD) or 5-12V DC |
| Operating temp | -10°C to +50°C |
| Storage | MicroSD (satellite data) |
| Interfaces | UART (MAVLink), USB-C, I2C |

### Performance Specifications

| Metric | Target Value |
|--------|--------------|
| Position accuracy | < 5m RMSE (with satellite matching) |
| Attitude accuracy | < 1° RMSE |
| Update rate | 50 Hz (VIO), 1 Hz (satellite) |
| Startup time | < 30 seconds |
| VIO drift | < 1% of distance traveled |

### Compatibility

| Autopilot | Protocol | Status |
|-----------|----------|--------|
| PX4 | MAVLink v2 VISION_POSITION_ESTIMATE | ✅ Primary |
| ArduPilot | MAVLink v2 VISION_POSITION_ESTIMATE | ✅ Supported |
| Custom | ROS2 Odometry topic | ✅ Supported |

---

## Interface Design

### MAVLink Output Messages

```
VISION_POSITION_ESTIMATE (#102)
├── usec: Timestamp (microseconds)
├── x: X position (NED frame, meters)
├── y: Y position (NED frame, meters)
├── z: Z position (NED frame, meters)
├── roll: Roll angle (radians)
├── pitch: Pitch angle (radians)
├── yaw: Yaw angle (radians)
├── covariance: Position/attitude covariance (21 floats)
└── reset_counter: Estimate reset counter

ATT_POS_MOCAP (#138) [Alternative]
├── time_usec: Timestamp
├── q: Attitude quaternion
├── x, y, z: Position
└── covariance: 21 floats
```

### Configuration Interface

```yaml
# VPS Module Configuration (stored on SD card)
# /config/vps_config.yaml

system:
  module_id: "VPS-001"
  mavlink_sys_id: 1
  mavlink_comp_id: 197  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
  
camera:
  resolution: [1920, 1080]
  fps: 30
  exposure_mode: "auto"
  
imu:
  rate_hz: 400
  accel_range: 16  # g
  gyro_range: 2000  # deg/s
  
vio:
  feature_count: 200
  use_stereo: false
  
satellite_matching:
  enabled: true
  match_interval_sec: 1.0
  confidence_threshold: 0.7
  
output:
  mavlink_uart: "/dev/ttyTHS1"
  mavlink_baud: 921600
  ros2_enabled: false
  
initial_position:
  use_gps_init: false
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
```

---

## Satellite Data Management

### User Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│                    SATELLITE DATA PREPARATION                    │
│                                                                  │
│  1. Define Mission Area                                         │
│     ┌─────────────────────────────────────────────────────┐     │
│     │  Web Tool: Draw polygon on map                       │     │
│     │  Export: GeoJSON bounds                              │     │
│     └─────────────────────────────────────────────────────┘     │
│                              │                                   │
│                              ▼                                   │
│  2. Download Satellite Imagery                                  │
│     ┌─────────────────────────────────────────────────────┐     │
│     │  Sources: Google Earth, Bing, USGS, Sentinel-2       │     │
│     │  Resolution: 0.3m - 1m per pixel                     │     │
│     │  Format: GeoTIFF                                     │     │
│     └─────────────────────────────────────────────────────┘     │
│                              │                                   │
│                              ▼                                   │
│  3. Preprocess (Desktop Tool)                                   │
│     ┌─────────────────────────────────────────────────────┐     │
│     │  - Extract features at multiple altitude scales      │     │
│     │  - Build spatial index                               │     │
│     │  - Generate metadata                                 │     │
│     │  Output: .vps package file                           │     │
│     └─────────────────────────────────────────────────────┘     │
│                              │                                   │
│                              ▼                                   │
│  4. Load to VPS Module                                          │
│     ┌─────────────────────────────────────────────────────┐     │
│     │  Copy .vps file to SD card                           │     │
│     │  Insert SD card into VPS module                      │     │
│     └─────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

### Companion Desktop Application (Future)

```
┌─────────────────────────────────────────────────────────────────┐
│                    VPS MISSION PLANNER                          │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                                                          │   │
│  │     [Map View - OpenStreetMap / Satellite]              │   │
│  │                                                          │   │
│  │              ┌──────────────┐                           │   │
│  │              │ Mission Area │                           │   │
│  │              │   (Polygon)  │                           │   │
│  │              └──────────────┘                           │   │
│  │                                                          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                  │
│  [Download Imagery]  [Preprocess]  [Export to SD Card]          │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Area: 2.5 km²    Resolution: 0.5m    Size: 450 MB      │   │
│  │  Altitudes: 50m, 100m, 200m, 400m                       │   │
│  │  Features: 125,000                                       │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Future Enhancements

### Short-term (< 1 year)

1. **Multi-camera support** - Stereo or wide-angle + telephoto
2. **Neural network features** - SuperPoint/SuperGlue for matching
3. **Terrain-relative navigation** - DEM correlation
4. **Real-time map updates** - Download satellite data in-flight (with connectivity)

### Medium-term (1-2 years)

1. **LiDAR integration** - Hybrid VIO + LiDAR for close-range accuracy
2. **Loop closure** - Graph-based SLAM for long missions
3. **Multi-vehicle coordination** - Shared map updates
4. **Indoor/outdoor transition** - Automatic mode switching

### Long-term (2+ years)

1. **Neuromorphic sensing** - Event cameras for high-speed flight
2. **ASIC acceleration** - Custom silicon for VIO
3. **Swarm localization** - Relative positioning between drones
4. **Certification** - DO-178C for commercial aviation

---

## Competitive Landscape

| Product | Technology | Price | Notes |
|---------|------------|-------|-------|
| **Theseus** | Visual Positioning System | N/A | Commercial VPS; config install: `curl -fsSL https://packages.theseus.us/install.sh \| sudo bash` |
| Intel RealSense T265 | Stereo VIO | $199 | Discontinued |
| ModalAI VOXL 2 | VIO + AI | $850 | Full autonomy stack |
| Auterion Skynode | VIO + Compute | ~$2000 | Enterprise focus |
| Skydio X2 | Proprietary VIO | N/A | Integrated drone only |
| **Proxigo VPS** | VIO + Satellite | TBD | GPS-denied focus |

### Differentiators

1. **Satellite referencing** - Unique absolute position correction
2. **Open architecture** - Based on OpenVINS, extensible
3. **Fixed-wing optimized** - Designed for high-speed, high-altitude
4. **Standalone module** - Plug-and-play with any autopilot
5. **Affordable** - Target < $500 for module

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Satellite matching accuracy | Medium | High | Multi-scale features, fallback to VIO-only |
| Compute power insufficient | Low | High | Test on Orin NX, optimize algorithms |
| Hardware integration issues | Medium | Medium | Modular design, off-the-shelf components |
| Competition releases similar | Medium | Medium | Focus on fixed-wing niche, open-source community |
| Regulatory changes | Low | High | Design for adaptability, follow DO standards |

---

## Resource Requirements

### Phase 1 (Software Validation)
- **Personnel**: 1-2 developers
- **Hardware**: Orin Nano dev kit, Arducam, test drone
- **Budget**: ~$2,000

### Phase 2-3 (Hardware Development)
- **Personnel**: 2-3 developers, 1 hardware engineer
- **Hardware**: Multiple prototypes, test equipment
- **Budget**: ~$20,000-50,000

### Phase 4 (Production)
- **Personnel**: Full team + manufacturing partner
- **Certification**: ~$50,000-100,000
- **Manufacturing setup**: ~$50,000-100,000

---

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Position accuracy | < 5m RMSE | Ground truth comparison |
| Reliability | > 99% uptime | Field testing hours |
| Startup time | < 30 seconds | Benchmark testing |
| Power efficiency | < 15W average | Power monitoring |
| User satisfaction | > 4.5/5 rating | Beta tester feedback |
| PX4/ArduPilot compatibility | 100% | Integration testing |

---

## Next Steps

1. ✅ Complete software architecture documentation
2. 🔄 Implement basic VIO with OpenVINS
3. 📋 Develop satellite matching algorithm
4. 📋 Integrate with PX4 SITL for testing
5. 📋 Conduct first flight tests
6. 📋 Evaluate hardware options for standalone module
