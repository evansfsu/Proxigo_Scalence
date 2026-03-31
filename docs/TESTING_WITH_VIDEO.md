# Testing VPS with Pre-Recorded Video

Test the Visual Positioning System on the Orin Nano (or any machine) using a pre-recorded drone video instead of a live camera.

---

## Prerequisites

On the Orin Nano (JetPack includes OpenCV and NumPy):

```bash
pip3 install scikit-learn Pillow
```

On a desktop/laptop:

```bash
pip install opencv-python opencv-contrib-python numpy scikit-learn Pillow
```

---

## Step 1: Get a Drone Video

You need a downward-facing video recorded from a drone at a known location and altitude. Options:

- **Record your own** -- fly over a known area with the camera pointing straight down, note the GPS coordinates and altitude.
- **Download a sample** -- search YouTube for "drone top down flight" or "nadir drone footage" and download with `yt-dlp`:

```bash
# Example: download a top-down drone video
pip install yt-dlp
yt-dlp -f "best[height<=1080]" -o drone_flight.mp4 "VIDEO_URL_HERE"
```

**Important**: The video must be **nadir (straight down)**, not angled. The VPS assumes the camera is looking directly at the ground.

Record or note:
- The **latitude and longitude** of the area in the video
- The **flight altitude** in metres (AGL)

---

## Step 2: Prepare Satellite Reference Data

Download satellite imagery for the area shown in the video. You need the approximate center coordinates.

```bash
cd ~/Proxigo_Scalence

# For a specific location (example: Death Valley)
python3 scripts/prepare_region.py \
    --lat 36.23 --lon -116.81 \
    --radius-m 500 \
    --output-dir satellite_data/regions/my_test_area

# For a larger area or custom bounds
python3 scripts/prepare_region.py \
    --north 36.235 --south 36.225 \
    --east -116.805 --west -116.815 \
    --output-dir satellite_data/regions/my_test_area \
    --zoom 18
```

This downloads Esri World Imagery tiles and saves:
- `satellite_data/regions/my_test_area/satellite.png` -- the reference image
- `satellite_data/regions/my_test_area/metadata.json` -- geographic metadata

Verify the reference was created:

```bash
ls -la satellite_data/regions/my_test_area/
cat satellite_data/regions/my_test_area/metadata.json
```

---

## Step 3: Run VPS on the Video

### With display (monitor connected to Orin Nano or desktop):

```bash
python3 scripts/vps_live.py \
    --source drone_flight.mp4 \
    --reference satellite_data/regions/my_test_area \
    --altitude 50 \
    --output-csv results.csv
```

This opens a window showing:
- **Top left**: Camera frame with detected ORB keypoints (green circles)
- **Top right**: Feature match visualization (reference left, query right, green lines connecting matches)
- **Bottom**: Stats bar (position, match count, confidence, FPS) and mini-map with position trail

Press `q` or `Esc` to stop.

### Headless (no monitor, SSH into Orin Nano):

```bash
python3 scripts/vps_live.py \
    --source drone_flight.mp4 \
    --reference satellite_data/regions/my_test_area \
    --altitude 50 \
    --headless \
    --output-csv results.csv
```

Position estimates are printed to the terminal and logged to CSV.

### Adjusting parameters:

```bash
python3 scripts/vps_live.py \
    --source drone_flight.mp4 \
    --reference satellite_data/regions/my_test_area \
    --altitude 50 \
    --rate 5 \
    --nfeatures 500 \
    --output-csv results.csv
```

| Flag | Default | Description |
|------|---------|-------------|
| `--rate` | 2.0 | VPS estimates per second (Hz) |
| `--nfeatures` | 250 | ORB features to extract (more = slower but potentially more matches) |
| `--fov-h` | 71.5 | Horizontal FOV in degrees (match your camera) |
| `--fov-d` | 79.5 | Diagonal FOV in degrees |
| `--width` | 1920 | Expected frame width |
| `--height` | 1080 | Expected frame height |
| `--display-width` | 1400 | Max display window width |

---

## Step 4: Review Results

### CSV output

The `results.csv` file contains one row per VPS estimate:

```
frame,timestamp_s,lat,lon,confidence,n_matches,success
42,2.100,36.23012345,-116.80987654,0.7500,18,True
84,4.200,36.23015678,-116.80982345,0.6800,14,True
126,6.300,,,,3,False
```

### What to look for

- **`success=True`** with consistent lat/lon values means the VPS is working.
- **`n_matches` > 10** indicates good feature matching.
- **`confidence` > 0.5** indicates reliable estimates.
- **`success=False`** means too few matches -- the area may lack texture, the altitude/FOV is wrong, or the video doesn't match the satellite reference location.

---

## Troubleshooting

### "No match data" on every frame

- The video location doesn't match the satellite reference coordinates. Double-check lat/lon.
- The altitude parameter is wrong, causing a scale mismatch between the camera view and reference.
- The area has low texture (water, uniform pavement, dense forest canopy).

### Very few matches (< 10)

- Try increasing features: `--nfeatures 500` or `--nfeatures 1000`
- The reference zoom level may be too low. Re-run `prepare_region.py` with `--zoom 19` for higher resolution.
- Check that the FOV parameters match your camera.

### Position jumps around

- This is expected without continuity history at the start. After a few successful estimates, the K-means continuity filter stabilizes the output.
- If it persists, the matching may be ambiguous (repeated patterns in the scene).

### Video won't open

- Check the video codec: `ffprobe drone_flight.mp4`
- Install codec support: `pip3 install opencv-python-headless` (or the full `opencv-python`)
- Try converting: `ffmpeg -i input.mp4 -c:v libx264 -pix_fmt yuv420p output.mp4`

---

## Quick Test Without a Real Drone Video

If you don't have drone footage yet, you can create a synthetic test video from the satellite reference itself:

```bash
python3 -c "
import cv2, sys
sys.path.insert(0, 'src/vps_device')
from vps_device.estimator import create_estimator_from_proxigo_region
from vps_device.simulation import generate_synthetic_view
from vps_device.config import VPSDeviceConfig

config = VPSDeviceConfig()
est = create_estimator_from_proxigo_region('satellite_data/regions/my_test_area', config)
ref = est._ref

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('synthetic_flight.mp4', fourcc, 5.0, (config.width_px, config.height_px))

import math
lat, lon = ref.center_lat, ref.center_lon
m_per_deg_lon = 6378137 * math.pi / 180 * math.cos(math.radians(lat))

for i in range(50):
    frame_lon = lon + (i * 2.0) / m_per_deg_lon
    view = generate_synthetic_view(ref, lat, frame_lon, 50.0, 0.0, config)
    out.write(view)

out.release()
print('Wrote synthetic_flight.mp4 (50 frames, 10s at 5fps)')
"
```

Then test with:

```bash
python3 scripts/vps_live.py \
    --source synthetic_flight.mp4 \
    --reference satellite_data/regions/my_test_area \
    --altitude 50 \
    --output-csv synthetic_results.csv
```

---

## Next Steps

Once video-based testing works:

1. Fix the camera connection and switch `--source` to the camera device
2. Tune `--nfeatures`, `--rate`, and FOV for your specific camera
3. Prepare satellite data for your actual flight area
4. Test at the flight location before flying
