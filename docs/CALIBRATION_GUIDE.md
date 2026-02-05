# Calibration Guide

This document explains the calibration requirements and what can/cannot be done without physical hardware.

## TL;DR - Can I Calibrate Without Hardware?

| Calibration Type | Without Hardware | Notes |
|-----------------|------------------|-------|
| Camera Intrinsics | ❌ Not accurate | Use datasheet estimates for development |
| Camera-IMU Extrinsics | ❌ Not accurate | Use CAD/design measurements |
| IMU Intrinsics | ❌ Not accurate | Use datasheet values |
| Time Sync | ❌ Not possible | Requires actual data streams |
| VIO Tuning | ⚠️ Partially | Can tune with public datasets |
| Satellite Matching | ✅ Yes | Can fully test in simulation |

## Calibration Files Structure

```
config/
├── camera/
│   ├── imx477_calibration_estimated.yaml    # Estimated intrinsics (dev)
│   ├── imx477_calibration_actual.yaml       # Actual calibration (hardware)
│   ├── camera_imu_extrinsics_estimated.yaml # Estimated extrinsics (dev)
│   └── camera_imu_extrinsics_actual.yaml    # Actual calibration (hardware)
├── vio/
│   └── openvins_proxigo.yaml                # OpenVINS configuration
└── navigation/
    └── mavros_config.yaml                   # MAVROS settings
```

## Step 1: Camera Intrinsic Calibration

### What It Does
Corrects for lens distortion and determines the focal length in pixels.

### Without Hardware (Development)
We've provided estimated values in `config/camera/imx477_calibration_estimated.yaml` based on:
- Sony IMX477 sensor datasheet
- 6mm CS-mount lens specifications
- Standard 1920x1080 resolution

**Estimated accuracy:** ±10-20% on focal length, ±50% on distortion

### With Hardware (Required for Flight)

1. **Print calibration target:**
   - Download: https://docs.opencv.org/4.x/pattern.png
   - Checkerboard: 9x6 inner corners, 25mm squares
   - Print on rigid material (foam board)

2. **Capture images:**
   ```bash
   # Inside Docker container
   ros2 run camera_calibration cameracalibrator \
     --size 9x6 --square 0.025 \
     image:=/vio/camera/image_raw
   ```

3. **Follow on-screen instructions:**
   - Move checkerboard through entire FOV
   - Vary distance (0.5m to 2m)
   - Include tilted angles
   - Capture 20-30 images

4. **Expected results:**
   - Reprojection error: < 0.5 pixels
   - Update `config/camera/imx477_calibration_actual.yaml`

## Step 2: Camera-IMU Extrinsic Calibration

### What It Does
Determines the physical relationship (rotation + translation) between camera and IMU.

### Without Hardware (Development)
We've provided estimated values in `config/camera/camera_imu_extrinsics_estimated.yaml` based on:
- Standard forward-facing camera mount
- Typical drone geometry assumptions

**Estimated accuracy:** ±2cm translation, ±5° rotation

### With Hardware (Required for Flight)

**Option A: Physical Measurement**
1. Measure camera position relative to IMU with calipers
2. Measure camera orientation using level/protractor
3. Works for rough estimates (±1cm, ±2°)

**Option B: Kalibr (Recommended)**
1. **Install Kalibr:**
   ```bash
   # Already included in openvins container
   docker compose -f docker-compose.dev.yml run --rm openvins bash
   ```

2. **Create target config:**
   ```yaml
   # april_6x6.yaml
   target_type: 'aprilgrid'
   tagCols: 6
   tagRows: 6
   tagSize: 0.088   # meters
   tagSpacing: 0.3  # ratio
   ```

3. **Record calibration bag:**
   ```bash
   ros2 bag record -o calibration \
     /vio/camera/image_raw \
     /vio/imu/data
   ```
   - Wave AprilGrid target for ~60 seconds
   - Include varied motions (rotation, translation)

4. **Run Kalibr:**
   ```bash
   rosrun kalibr kalibr_calibrate_imu_camera \
     --target april_6x6.yaml \
     --cam camera.yaml \
     --imu imu.yaml \
     --bag calibration.bag
   ```

**Option C: Online Calibration**
- OpenVINS supports online extrinsics calibration
- Set `do_calib_camera_extrinsics: true` in config
- Requires good initial estimate (within ±30°)

## IMU Calibration

### What It Does
Determines noise characteristics and biases.

### Without Hardware
Use datasheet values (already configured in OpenVINS config):
- Accelerometer noise density: 0.002 m/s²/√Hz
- Gyroscope noise density: 0.00016 rad/s/√Hz

### With Hardware
1. **Static test (bias):**
   - Place drone stationary for 5 minutes
   - Record IMU data
   - Calculate mean values (bias)

2. **Allan variance (noise):**
   - Requires specialized equipment
   - Usually datasheet values are sufficient

## Time Synchronization

### What It Does
Aligns camera and IMU timestamps.

### Without Hardware
Cannot be done - requires actual sensor data streams.

### With Hardware
1. **Hardware sync (ideal):**
   - Use trigger signal from camera to timestamp IMU
   - Typical jitter: < 1ms

2. **Software estimation (Kalibr):**
   - Kalibr estimates time offset during extrinsics calibration
   - Typical accuracy: ±5ms

3. **OpenVINS online:**
   - Set `do_calib_camera_timeoffset: true`
   - Estimates during operation

## VIO Parameter Tuning

### Without Hardware (Partial)
You can tune VIO parameters using public datasets:

```bash
# Download EuRoC dataset
python scripts/download_test_datasets.py --dataset euroc_mh01

# Run OpenVINS with dataset
ros2 launch vio_bridge openvins.launch.py \
  config_file:=/ros2_ws/config/openvins.yaml
```

Tune parameters like:
- `num_pts`: Number of features (100-300)
- `fast_threshold`: Feature detection sensitivity (10-30)
- `max_clones`: Sliding window size (8-15)

### With Hardware
Fine-tune on actual flight data:
1. Record test flight
2. Run offline analysis
3. Adjust parameters based on trajectory error

## Satellite Matching Calibration

### Without Hardware (Fully Testable!)
Satellite matching can be fully tested without hardware:

1. **Download satellite imagery:**
   ```bash
   python scripts/download_satellite_imagery.py \
     --lat 39.678 --lon -75.750 --size 1000
   ```

2. **Run simulation:**
   ```bash
   ./scripts/run_simulation.sh --test
   ```

3. **Tune parameters:**
   - `confidence_threshold`: 0.2-0.5
   - `min_matches`: 8-20
   - `altitude_bins`: Varies by mission

### With Hardware
Validate matching accuracy:
1. Fly over surveyed area with GPS
2. Compare satellite position estimates to GPS truth
3. Adjust parameters for your camera/altitude

## Development Workflow

### Phase 1: Simulation (No Hardware)
```
1. Use estimated calibration files ✓
2. Test with synthetic data ✓
3. Tune satellite matching ✓
4. Develop fusion algorithms ✓
```

### Phase 2: Lab Testing (With Hardware)
```
1. Perform camera intrinsic calibration
2. Perform camera-IMU extrinsic calibration
3. Validate with static tests
```

### Phase 3: Flight Testing
```
1. Ground tests with VIO running
2. Short test flights
3. Compare VIO + satellite to GPS
4. Fine-tune all parameters
```

## Calibration Checklist

Before first flight, ensure:

- [ ] Camera intrinsics calibrated (reprojection error < 0.5px)
- [ ] Camera-IMU extrinsics calibrated (Kalibr or measured)
- [ ] Time offset estimated
- [ ] IMU biases recorded
- [ ] VIO tested on ground (walking around)
- [ ] Satellite matching tested with local imagery
- [ ] Fusion validated against GPS

## Files to Update After Hardware Calibration

1. `config/camera/imx477_calibration_actual.yaml` - Replace estimated values
2. `config/camera/camera_imu_extrinsics_actual.yaml` - Replace estimated values
3. `config/vio/openvins_proxigo.yaml` - Update cam0 intrinsics and extrinsics
4. `config/mission/initial_pose.yaml` - Set your operating area coordinates
