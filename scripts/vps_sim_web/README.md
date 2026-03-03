# VPS Simulation Web App

Interactive global satellite map to test the Visual Positioning System in software: pick a region, download imagery, run simulation over different altitudes and viewing angles.

## Quick start

From the **repository root**:

```bash
pip install -r scripts/vps_sim_web/requirements.txt
python scripts/vps_sim_web/app.py
```

Then open **http://127.0.0.1:5000** in your browser.

**After code changes:** Restart the app (stop with Ctrl+C, then run `python scripts/vps_sim_web/app.py` again) so changes take effect.

**Mapbox globe (recommended):** Without a Mapbox token you get the flat Esri map. To use the **globe view** (satellite, Death Valley):

- **Option A:** Create `scripts/vps_sim_web/.env` with one line:  
  `MAPBOX_ACCESS_TOKEN=pk.your_mapbox_token_here`  
  (Copy from `.env.example`; get a token at https://account.mapbox.com/access-tokens/)
- **Option B:** Set the env var before running, e.g.  
  `set MAPBOX_ACCESS_TOKEN=pk.your_token` (Windows) or `export MAPBOX_ACCESS_TOKEN=pk.your_token` (Linux/Mac)

Then restart the app. On startup it will print whether the Mapbox token was found.

## Usage

1. **Map**: With a valid Mapbox token, the app uses **Mapbox globe** view (satellite, centered on Death Valley). Without a token, it uses Esri World Imagery (flat).
2. **Draw region**: Click the first corner of the area you want, then click the opposite corner. A blue rectangle appears.
3. **Run simulation**: Click **"Download & Run Simulation"**. The server will:
   - Fetch satellite tiles for the rectangle
   - Build a reference image and metadata
   - Run the VPS estimator on synthetic camera views at several altitudes (e.g. 30, 50, 100 m) and random positions inside the region
   - Return success rate and position error
4. **Results**: Summary (e.g. 12/15 successful, avg error 3.2 m), a table of each run, and on the map **green** markers (ground truth) and **red** markers (estimated position).

Options in the panel:

- **Region**: Preset dropdown to jump the map to Death Valley, Mojave Desert, Grand Canyon, or Custom (draw on map).
- **Altitudes (m)**: Comma-separated list, e.g. `30, 50, 100`
- **Positions**: Number of random positions to test (default 1, max 20)
- **Advanced (ORB / match)**: Expand to tune estimator parameters: ORB features (50–500), min matches, ratio threshold, clusters. Useful for tuning success rate vs speed.
- **Export**: After a run, **Export CSV** and **Export JSON** download the results (summary + per-run data). JSON includes full payload; CSV is a flat table for spreadsheets.

**Imagery:** If the server returns "Satellite imagery could not be loaded for this region", try a smaller rectangle or a different location; some areas return "map data not available" tiles that are rejected.

## API

- `POST /api/simulate` with JSON body:
  - `north`, `south`, `east`, `west`: Bounding box (degrees)
  - `altitudes`: Array of altitudes in metres (default `[50]`)
  - `yaw_angles`: Array of yaw angles in degrees (default `[0]`)
  - `num_positions`: Number of random positions (default 1, max 20)
  - `config_override`: Optional `{ nfeatures_orb, min_matches, ratio_threshold, n_clusters }` to tune the estimator

Returns `{ results: [...], summary: { total, success_count, avg_error_m, center_lat, center_lon } }`. Each result includes `debug_image_b64` (base64 PNG of ref vs query match view) when available.

## Dependencies

- Flask, flask-cors (optional)
- vps_device package (repo `src/vps_device`): reference_loader, simulation
- opencv-python, numpy, scikit-learn, Pillow (for tile fetch)
