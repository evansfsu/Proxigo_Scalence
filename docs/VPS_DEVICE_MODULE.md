# VPS Device Module

## Purpose

The **VPS (Visual Positioning System) device module** is a standalone, device-focused component that estimates position by comparing live imaging and inertial/altitude data to downloaded satellite reference imagery. It is designed to be testable without the full PX4/ROS2/SITL stack and aligns with the [VPS Module Roadmap](VPS_MODULE_ROADMAP.md) and [VIO + Satellite Design](VIO_SATELLITE_DESIGN.md).

## Reference

This implementation is based on:

- **UAVs@Berkeley 106a GPS-denied project**: [uav.studentorg.berkeley.edu/106a-fa24-gps](https://uav.studentorg.berkeley.edu/106a-fa24-gps/)
- **Code reference**: [UAVs-at-Berkeley/suas_2022, fa24_106a](https://github.com/UAVs-at-Berkeley/suas_2022/tree/main/fa24_106a)

Their approach: ORB keypoints + BEBLID descriptors, FLANN matcher, Lowe's ratio test (0.95), K-means clustering to select a consistent set of matches, and GPS transform from reference image metadata plus camera FOV and altitude.

## Algorithm Summary

1. **Reference loading**: Load one or more reference images with known geographic metadata (center lat/lon and metres per pixel).
2. **Feature extraction**: ORB keypoints; descriptors from BEBLID (if opencv-contrib is available) or ORB.
3. **Matching**: FLANN for binary descriptors; Lowe's ratio test to filter ambiguous matches.
4. **Continuity**: K-means cluster match coordinates on the reference image; select the cluster whose median position (in geo) is within a configurable distance (e.g. 50 m) of the last predicted position; keep only matches within 5 m of that cluster median.
5. **GPS transform**: For each selected match: reference pixel → (lat, lon) using reference image m/px; then camera pixel offset from center → camera (lat, lon) using altitude and FOV. Average to get final estimate.

## Reference Data Formats

### Proxigo-style (recommended)

- Directory: `satellite_data/regions/<region_id>/`
- **metadata.json**: Must include `center.latitude`, `center.longitude`, and `imagery.resolution_m`, `imagery.file` (and optionally `imagery.width_pixels`, `imagery.height_pixels`).
- One reference image per region; uniform metres per pixel from `resolution_m`.

See [satellite_data/regions/test_region/metadata.json](../satellite_data/regions/test_region/metadata.json) for an example.

### Berkeley-style

- Single image path plus numeric metadata: center lat, center lon, physical height (m), width (m).
- Metres per pixel = (height_m / image_height_px, width_m / image_width_px).
- Filenames often encode this (e.g. `37.872310N_122.322454W_231.23H_297.8W.png`).

The standalone script supports Berkeley-style via the `--berkeley lat lon height_m width_m path` arguments.

## Usage

### Standalone (no ROS2)

Run on a video file, RTSP stream, or directory of images with a constant or per-frame altitude:

```bash
# From repo root, with Proxigo region
python src/vps_device/scripts/run_vps_standalone.py \
  --video path/to/video.mp4 \
  --reference satellite_data/regions/test_region \
  --altitude 30 \
  --output estimates.csv -v

# With live visualization (match overlay + position on frame; press 'q' to quit)
python src/vps_device/scripts/run_vps_standalone.py \
  --video path/to/video.mp4 \
  --reference satellite_data/regions/test_region \
  --altitude 30 \
  --output estimates.csv --visualize

# RTSP
python src/vps_device/scripts/run_vps_standalone.py \
  --rtsp "rtsp://192.168.1.1:8554/stream" \
  --reference satellite_data/regions/test_region \
  --altitude 50 -v

# Berkeley-style single reference
python src/vps_device/scripts/run_vps_standalone.py \
  --video flight.MOV \
  --berkeley 37.872 122.322 231.23 297.8 path/to/ref.png \
  --altitude 30 -v
```

Optional: `--altitude-file` CSV (per-frame altitude), `--frame-skip`, `--fov-h`, `--fov-d`, `--width`, `--height` to match your camera. Use `--visualize` to show live windows: current frame with position/confidence overlay, and reference-vs-query with matched keypoints (green lines). Press **q** to quit.

**Plot trajectory after a run** (from CSV):

```bash
python src/vps_device/scripts/plot_vps_trajectory.py estimates.csv --out trajectory.png --show
```

This plots (lat, lon) path and confidence over frame index. Use `--show` to open an interactive window, or `--out path.png` to save the figure.

**View test visualizations on your laptop:** When you run the tests, one test saves visualization images to `src/vps_device/test/output/` (overlay and ref-vs-query matches). To generate and optionally pop up the same visuals in a window, run:

```bash
python src/vps_device/scripts/view_test_viz.py          # save only
python src/vps_device/scripts/view_test_viz.py --show   # save and open windows (press key to close)
```

### Global map simulation (web app)

An interactive **satellite map** lets you pick a region, download imagery, and run the VPS in software simulation over different altitudes and angles:

1. **Start the app** (from repo root):
   ```bash
   pip install -r scripts/vps_sim_web/requirements.txt
   python scripts/vps_sim_web/app.py
   ```
2. Open **http://127.0.0.1:5000** in your browser.
3. The map uses **Esri World Imagery** (satellite) by default. You can swap to Mapbox satellite by changing the tile URL in `static/index.html` if you have a Mapbox token.
4. **Draw a rectangle**: click one corner, then the opposite corner. The rectangle is your region.
5. Click **"Download & Run Simulation"**. The backend downloads satellite tiles for that region, builds a reference image, then runs the VPS simulator over multiple **altitudes** (e.g. 30, 50, 100 m) and **positions** (random points inside the region). For each run it generates a synthetic camera view, runs the estimator, and records error vs ground truth.
6. **Results**: summary (success count, average error in m), a table (per run: altitude, truth/estimated lat/lon, error, success), and on the map **green** markers (ground truth) and **red** markers (estimated position).

You can change **Altitudes (m)** (e.g. `30, 50, 100`) and **Positions** (number of random positions) in the panel. This gives a quick way to see how the VPS behaves at different altitudes and viewing angles without flying.

### ROS2 node

1. Build and source the workspace (include `vps_device`).
2. Launch with a Proxigo region path:

```bash
ros2 launch vps_device vps_device.launch.py \
  reference_path:=/path/to/satellite_data/regions/test_region \
  image_topic:=/vio/camera/image_raw \
  altitude_topic:=/mavros/global_position/rel_alt
```

Parameters:

- **reference_path**: (required) Path to region directory containing `metadata.json` and the reference image.
- **image_topic**: Input image (default: `/vio/camera/image_raw`).
- **altitude_topic**: Float64 relative altitude (default: `/mavros/global_position/rel_alt`).
- **match_interval_sec**: Minimum interval between estimates (default: 1.0).
- **fov_h**, **fov_d**, **width_px**, **height_px**: Camera parameters (defaults: 71.5°, 79.5°, 1920, 1080).

Output topics:

- **/vps_device/position**: `geometry_msgs/PoseWithCovarianceStamped` (position.x = lat, position.y = lon, position.z = altitude).
- **/vps_device/confidence**: `std_msgs/Float32`.

### Python API

```python
from vps_device import VPSEstimator, VPSDeviceConfig
from vps_device.estimator import create_estimator_from_proxigo_region

config = VPSDeviceConfig(width_px=1920, height_px=1080)
estimator = create_estimator_from_proxigo_region("satellite_data/regions/test_region", config)
result = estimator.estimate(cv2.imread("frame.jpg"), altitude_m=30.0)
if result.success:
    print(result.lat, result.lon, result.confidence)
```

## FOV and altitude calibration

- **Horizontal and diagonal FOV** (e.g. 71.5° and 79.5° for Siyi ZR10) and image **width/height in pixels** define the camera’s metres-per-pixel at a given altitude: `cam_x = 2 * alt * tan(h_fov/2)`, then `x_m_per_px = cam_x / width_px` (and similarly for y).
- **Altitude** should be relative to ground (AGL). Use barometric or range sensor when available; for standalone tests a constant or CSV is sufficient.

## Related companies and references

- **Theseus** – Drone company that produces a Visual Positioning System. For reference, one of their model config/software installs can be run with:
  ```bash
  curl -fsSL https://packages.theseus.us/install.sh | sudo bash
  ```
  (Use only on systems you control; review the script before piping to shell.)

## Related docs

- [VPS_MODULE_ROADMAP.md](VPS_MODULE_ROADMAP.md) – Standalone VPS device vision and phases.
- [VIO_SATELLITE_DESIGN.md](VIO_SATELLITE_DESIGN.md) – VIO + satellite matching pipeline and fusion.
