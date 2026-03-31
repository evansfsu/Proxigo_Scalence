# Proxigo Scalence

**GPS-Denied Visual Positioning System for Autonomous UAVs**

Proxigo Scalence is a Visual Positioning System (VPS) that enables autonomous UAV navigation without GPS. It uses onboard camera imagery matched against pre-downloaded satellite maps to estimate position in real time, deployed on an NVIDIA Jetson Orin Nano.

---

## How It Works

```
Camera Frame (live)          Satellite Reference (pre-loaded)
       |                              |
       v                              v
  ORB Feature                   ORB Feature
  Extraction                    Extraction
       |                              |
       +---------> FLANN <------------+
                 Matching
                    |
                    v
            K-Means Clustering
            + Continuity Filter
                    |
                    v
          Geo-Transform: pixel
          offsets -> (lat, lon)
                    |
                    v
            Position Estimate
```

The VPS extracts ORB features from both the live camera frame and satellite reference imagery, matches them with FLANN and Lowe's ratio test, clusters matches with K-means for outlier rejection, and converts pixel offsets to geographic coordinates using a flat-earth approximation. No GPS or IMU is required.

---

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Companion Computer | NVIDIA Orin Nano 8GB | Runs VPS pipeline |
| Camera | Arducam IMX477 (12.3MP, 6mm CS lens) | Downward-facing aerial imagery |
| Flight Controller | PX4 Autopilot (Pixhawk 6X/6C) | Autopilot, UART to Orin Nano |
| IMU (optional) | VectorNav VN-100 or BMI088 | Future VIO integration |

---

## Quick Start

### 1. Install Dependencies

```bash
pip install opencv-python opencv-contrib-python numpy scikit-learn Pillow
```

### 2. Prepare Satellite Reference Data

Download satellite imagery for your flight area and save it as a Proxigo region:

```bash
python scripts/prepare_region.py \
    --lat 36.23 --lon -116.81 --radius-m 500 \
    --output-dir satellite_data/regions/death_valley
```

This creates `satellite.png` + `metadata.json` in the output directory.

### 3. Run VPS Live

With a camera:

```bash
python scripts/vps_live.py \
    --source /dev/video0 \
    --reference satellite_data/regions/death_valley \
    --altitude 50
```

With a pre-recorded video:

```bash
python scripts/vps_live.py \
    --source flight_recording.mp4 \
    --reference satellite_data/regions/death_valley \
    --altitude 50 \
    --output-csv results.csv
```

The display window shows the camera feed with detected keypoints, feature match visualization against the satellite reference, estimated position, and a position trail on a mini-map.

Press `q` to quit.

### 4. Run the Web Simulator

Test the VPS algorithm on any region without flying:

```bash
cd scripts/vps_sim_web
pip install -r requirements.txt
python app.py
```

Open `http://localhost:5000/3d` for the 3D CesiumJS simulator with trajectory simulation, VO vs VPS comparison, and debug match visualization.

---

## Test Data: UAV-VisLoc Dataset

The project uses the [UAV-VisLoc](https://github.com/IntelliSensing/UAV-VisLoc) dataset for benchmarking -- 6,742 drone images with GPS ground truth across 11 regions, each paired with a georeferenced satellite map.

Download the dataset (16.4 GB): [Google Drive](https://drive.google.com/file/d/1xYODANyilEMM3CfWh85APwkTHQeLTcCT/view?usp=sharing) or [Kaggle](https://www.kaggle.com/datasets/hailong1610/uav-visloc-dataset)

Place the zip at the project root as `UAV_VisLoc_dataset.zip`, then extract a section:

```bash
# Extract section 07 (smallest, 30 images, ~29 MB)
python scripts/setup_uav_visloc.py --section 07

# Run VPS against it with ground-truth comparison
python scripts/vps_live.py \
    --source-dir test_data/uav_visloc/07/drone \
    --source-csv test_data/uav_visloc/07/07.csv \
    --reference test_data/uav_visloc/07 \
    --altitude 689 --output-csv results.csv
```

---

## Test Data: UAV-AVL / AnyVisLoc (Low-Altitude Multi-View)

The [UAV-AVL Benchmark](https://github.com/UAV-AVL/Benchmark?tab=readme-ov-file#baseline) provides a low-altitude, multi-view UAV visual localization dataset with reference maps and per-frame metadata (pose/camera fields).

This repo includes a **separate benchmark-style runner** that mirrors the key baseline ideas:

- **Deep retrieval** to reduce search space on large maps
- **Local matching** on retrieved reference chips
- **Planar pose fix** (homography decomposition using intrinsics) to generate an absolute measurement
- **EKF fusion + gating** to reject outliers and smooth trajectory

### Setup

Install additional dependencies for deep retrieval (optional, benchmark runner only):

```bash
pip install torch torchvision
```

Download the UAV-AVL (1/25) dataset per their instructions and place it in your workspace. The dataset structure and tips are documented in the upstream repo:

- `https://github.com/UAV-AVL/Benchmark?tab=readme-ov-file#baseline`

### Run the benchmark runner

Example (AnyVisLoc-style metadata JSON + an extracted image directory):

```bash
python scripts/vps_avl_benchmark.py \
  --source-dir test_data/anyvis_qz/images_nadir \
  --reference test_data/anyvis_qz \
  --metadata-json test_data/anyvis_temp/QZ_Town.json \
  --output-csv test_data/anyvis_qz/results_avl.csv \
  --chip-size 768 --stride 512 --topk 5 \
  --map-match-every 5 --min-pnp-inliers 12
```

Outputs a CSV with fused position, retrieval score, pose-fix diagnostics, and GT error (when metadata is provided).

The output CSV includes `truth_lat`, `truth_lon`, and `error_m` columns for accuracy analysis.

> **Citation**: Yuxuan Zhou et al., "UAV-VisLoc: A Large-scale Dataset for UAV Visual Localization," arXiv:2405.11936, 2024.

See [Testing with Video](docs/TESTING_WITH_VIDEO.md) for more details on pre-recorded testing.

---

## Project Structure

```
Proxigo_Scalence/
├── src/
│   ├── vps_device/              # Core VPS library (no ROS2 dependency)
│   │   ├── estimator.py         # VPSEstimator: image + altitude -> (lat, lon)
│   │   ├── features.py          # ORB extraction, FLANN matching, K-means filtering
│   │   ├── geo_transform.py     # Pixel <-> geographic coordinate conversion
│   │   ├── reference_loader.py  # Load satellite reference images + metadata
│   │   ├── config.py            # Camera FOV, resolution, matcher parameters
│   │   ├── continuity.py        # Cluster selection with position history
│   │   ├── simulation.py        # Synthetic view generation, trajectory simulation
│   │   └── vps_device_node.py   # ROS2 node wrapper
│   ├── proxigo_bringup/         # ROS2 launch files
│   ├── satellite_matching/      # ROS2 satellite matcher node
│   ├── state_fusion/            # VIO + satellite fusion EKF
│   └── vio_bridge/              # VIO interface, camera/IMU simulators
├── scripts/
│   ├── vps_live.py              # Standalone VPS runner (camera, video, or image dir)
│   ├── prepare_region.py        # Download satellite imagery for a region
│   ├── setup_uav_visloc.py      # Extract UAV-VisLoc test data from zip
│   └── vps_sim_web/             # Web-based VPS simulator (Flask + CesiumJS)
├── config/                      # Camera calibration, VIO params, mission config
├── docker/                      # Dockerfiles for Orin Nano deployment
├── satellite_data/              # Pre-downloaded satellite reference imagery
├── docs/                        # Architecture and design documentation
├── docker-compose.yml           # Production deployment (Orin Nano)
└── docker-compose.dev.yml       # Development / simulation
```

---

## VPS Algorithm

### Pipeline

1. **Feature Extraction** -- ORB keypoints and descriptors (configurable count, default 250) from both camera frame and satellite reference.
2. **Feature Matching** -- FLANN-based matching with Lowe's ratio test (threshold 0.95).
3. **Outlier Rejection** -- K-means clustering (6 clusters) on reference-image match coordinates. Select the cluster closest to the last known position. Discard matches far from the cluster median.
4. **Position Estimation** -- For each inlier match, convert the reference pixel to geographic coordinates, then offset by the camera pixel position relative to frame center. Take the median of all estimates.
5. **Continuity** -- The previous position estimate seeds the next cluster selection, providing frame-to-frame consistency without an IMU.

### Configuration

Key parameters in `VPSDeviceConfig`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `h_fov_deg` | 71.5 | Camera horizontal field of view |
| `d_fov_deg` | 79.5 | Camera diagonal field of view |
| `width_px` / `height_px` | 1920 / 1080 | Camera resolution |
| `nfeatures_orb` | 250 | ORB features to extract |
| `ratio_threshold` | 0.95 | Lowe's ratio test threshold |
| `n_clusters` | 6 | K-means clusters for outlier rejection |
| `min_matches` | 10 | Minimum matches to produce an estimate |

---

## Deployment on Orin Nano

### Direct Execution (Recommended for Initial Testing)

```bash
# SSH into Orin Nano
ssh nvidia@orin.local

# Copy project
scp -r Proxigo_Scalence/ nvidia@orin.local:~/

# Install dependencies (JetPack includes OpenCV and NumPy)
pip3 install scikit-learn Pillow

# Prepare reference data for your test area
python3 scripts/prepare_region.py \
    --lat YOUR_LAT --lon YOUR_LON --radius-m 500 \
    --output-dir ~/satellite_data/my_region

# Run VPS live
python3 scripts/vps_live.py \
    --source /dev/video0 \
    --reference ~/satellite_data/my_region \
    --altitude 50
```

### Docker Deployment (Full Stack)

```bash
./scripts/build_all.sh jetson
docker-compose up -d
```

### Deploy Script

```bash
./scripts/deploy_orin.sh orin.local nvidia
```

---

## ROS2 Integration

The VPS core library has no ROS2 dependency. For integration with the full ROS2 stack:

| Package | Description |
|---------|-------------|
| `vps_device` | VPS estimator node: subscribes to image + altitude, publishes position + confidence + debug image |
| `satellite_matching` | Satellite matcher with region management |
| `state_fusion` | EKF fusing VIO + satellite matches |
| `vio_bridge` | VIO interface, camera/IMU simulators |
| `proxigo_bringup` | Launch files for simulation and hardware |

```bash
# Build ROS2 packages
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash

# Launch simulation
ros2 launch proxigo_bringup simulation.launch.py

# Launch on hardware (Orin Nano)
ros2 launch proxigo_bringup hardware.launch.py
```

---

## Roadmap

### Phase 1: Software Validation -- COMPLETE
- [x] System architecture and documentation
- [x] ROS2 package structure
- [x] VPS estimator library (ORB + FLANN + K-means + geo-transform)
- [x] Satellite matching algorithm with continuity filtering
- [x] Web-based VPS simulator with trajectory simulation
- [x] State fusion EKF
- [x] PX4/MAVROS integration scaffolding

### Phase 2: Live Testing -- IN PROGRESS
- [x] Standalone VPS live runner (camera + video input)
- [x] Satellite reference preparation tool
- [ ] Camera calibration on Orin Nano
- [ ] Ground-level position accuracy testing
- [ ] First flight tests at 30-100m altitude
- [ ] Performance tuning (feature count, matching threshold)

### Phase 3: VPS Hardware Module
- [ ] OpenVINS VIO integration
- [ ] IMU fusion for inter-frame odometry
- [ ] Standalone hardware enclosure design
- [ ] MAVLink VISION_POSITION_ESTIMATE output
- [ ] Custom carrier board

See [VPS Module Roadmap](docs/VPS_MODULE_ROADMAP.md) for the full hardware module plan.

---

## Documentation

- [Testing with Video](docs/TESTING_WITH_VIDEO.md) -- Pre-recorded video and image sequence testing
- [System Architecture](docs/ARCHITECTURE.md)
- [Docker Architecture](docs/DOCKER_ARCHITECTURE.md)
- [VIO + Satellite Design](docs/VIO_SATELLITE_DESIGN.md)
- [VPS Module Roadmap](docs/VPS_MODULE_ROADMAP.md)
- [Simulation Setup](docs/SIMULATION_SETUP.md)
- [Calibration Guide](docs/CALIBRATION_GUIDE.md)

---

## References

- [OpenVINS](https://github.com/rpng/open_vins) -- Visual-inertial navigation
- [PX4 Autopilot](https://px4.io/) -- Flight control
- [ROS2 Humble](https://docs.ros.org/en/humble/) -- Robot middleware
- [MAVROS2](https://github.com/mavlink/mavros) -- MAVLink bridge for ROS2

---

## License

MIT License. See [LICENSE](LICENSE) for details.
