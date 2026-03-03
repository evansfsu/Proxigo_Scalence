#!/usr/bin/env python3
"""
VPS Simulation Web App: Mapbox globe view (Death Valley), draw a region,
download imagery, run software simulation. Run: python app.py
Set MAPBOX_ACCESS_TOKEN in the environment for Mapbox globe/satellite.
"""

import json
import os
import sys
import tempfile
import importlib.util
import base64
from pathlib import Path

# Repo root (app.py is in scripts/vps_sim_web/); vps_device package is src/vps_device/vps_device/
REPO_ROOT = Path(__file__).resolve().parents[2]
_script_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT / "src" / "vps_device"))

# Load fetch_region from this app's directory so it always uses the right code and finds PIL/numpy
_fetch_region_path = _script_dir / "fetch_region.py"
_fetch_region_spec = importlib.util.spec_from_file_location("fetch_region", _fetch_region_path)
_fetch_region_mod = importlib.util.module_from_spec(_fetch_region_spec)
sys.modules["fetch_region"] = _fetch_region_mod
_fetch_region_spec.loader.exec_module(_fetch_region_mod)
fetch_imagery_bbox = _fetch_region_mod.fetch_imagery_bbox
# Startup check: if PIL/numpy failed in fetch_region, 503 will occur on first simulate
if getattr(_fetch_region_mod, "Image", None) is None or getattr(_fetch_region_mod, "np", None) is None:
    print("WARNING: fetch_region missing PIL or numpy. Run: pip install Pillow numpy")
_env_file = _script_dir / ".env"
try:
    from dotenv import load_dotenv
    load_dotenv(_env_file)
except ImportError:
    pass
# Fallback: read .env manually if token still missing (no python-dotenv or not loaded)
if not os.environ.get("MAPBOX_ACCESS_TOKEN") and _env_file.exists():
    with open(_env_file, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, _, value = line.partition("=")
                key, value = key.strip(), value.strip().strip('"').strip("'")
                if key == "MAPBOX_ACCESS_TOKEN" and value:
                    os.environ["MAPBOX_ACCESS_TOKEN"] = value
                    break

from flask import Flask, request, jsonify, send_from_directory, render_template

app = Flask(__name__, static_folder="static", template_folder="templates")

# Socket.IO (for SITL telemetry streaming)
try:
    from flask_socketio import SocketIO
except ImportError:
    SocketIO = None

socketio = SocketIO(app, cors_allowed_origins="*") if SocketIO is not None else None


class _NumpySafeEncoder(json.JSONEncoder):
    """Encode numpy float32/int64 etc. as native Python types for JSON."""

    def default(self, obj):
        try:
            import numpy as np
            if isinstance(obj, np.generic):
                return obj.item()
            if isinstance(obj, np.ndarray):
                return obj.tolist()
        except (ImportError, AttributeError):
            pass
        return super().default(obj)


# Flask < 2.2
app.json_encoder = _NumpySafeEncoder
# Flask 2.2+ uses JSONProvider; ensure jsonify uses our encoder
try:
    from flask.json.provider import DefaultJSONProvider
    class _NumpyJSONProvider(DefaultJSONProvider):
        def dumps(self, obj, **kwargs):
            kwargs.setdefault("cls", _NumpySafeEncoder)
            return json.dumps(obj, **kwargs)
    app.json = _NumpyJSONProvider(app)
except ImportError:
    pass

# Optional: allow CORS for local dev
try:
    from flask_cors import CORS
    CORS(app)
except ImportError:
    pass

# MAVLink SITL telemetry bridge (optional)
_mav_bridge = None
_mav_bridge_started = False
if socketio is not None:
    try:
        from mavlink_bridge import MAVLinkBridge
        _mav_bridge = MAVLinkBridge(socketio)

        @socketio.on("connect")
        def _on_connect():
            global _mav_bridge_started
            if _mav_bridge is not None and not _mav_bridge_started:
                _mav_bridge.start()
                _mav_bridge_started = True
    except Exception:
        _mav_bridge = None

import cv2

from reference_cache import cache_key as _cache_key, region_dir as _cache_region_dir, region_ready as _cache_region_ready


MAX_REF_PX = 1200
CACHE_ROOT = _script_dir / ".cache" / "regions"
CACHE_ROOT.mkdir(parents=True, exist_ok=True)


def _png_b64(image_bgr) -> str:
    ok, png = cv2.imencode(".png", image_bgr)
    if not ok:
        return ""
    return base64.b64encode(png.tobytes()).decode("ascii")


def _to_json_float(x):
    """Convert numpy float32/64 to native float for JSON serialization; NaN -> None."""
    if x is None:
        return None
    try:
        v = float(x)
        return v if v == v else None  # NaN check
    except (TypeError, ValueError):
        return None


def _create_region_from_image(
    image_rgb,
    center_lat: float,
    center_lon: float,
    resolution_m: float,
    out_dir: Path,
) -> Path:
    """Write image + metadata.json in Proxigo region format. Downscales to max MAX_REF_PX for speed."""
    out_dir.mkdir(parents=True, exist_ok=True)
    h, w = image_rgb.shape[:2]
    scale = min(1.0, MAX_REF_PX / max(w, h))
    if scale < 1.0:
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        image_rgb = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)
        resolution_m = resolution_m / scale  # each pixel now covers more ground
        h, w = new_h, new_w
    img_path = out_dir / "satellite.png"
    if image_rgb.shape[2] == 3:
        cv2.imwrite(str(img_path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    else:
        cv2.imwrite(str(img_path), image_rgb)
    metadata = {
        "region_id": "sim_region",
        "name": "Simulation region",
        "center": {"latitude": center_lat, "longitude": center_lon},
        "imagery": {
            "resolution_m": resolution_m,
            "width_pixels": w,
            "height_pixels": h,
            "file": "satellite.png",
        },
    }
    with open(out_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)
    return out_dir


@app.route("/")
def index():
    mapbox_token = os.environ.get("MAPBOX_ACCESS_TOKEN", "")
    resp = app.make_response(render_template("index.html", mapbox_token=mapbox_token))
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    return resp



@app.route("/api/tile/<int:z>/<int:x>/<int:y>.png")
def tile_proxy(z, x, y):
    """Proxy map tiles same-origin so Cesium can use them (avoids CORS)."""
    import urllib.request
    url = f"https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "VPS-Sim-Web/1.0 (Cesium)"})
        with urllib.request.urlopen(req, timeout=10) as r:
            data = r.read()
            from flask import Response
            return Response(data, mimetype="image/png", headers={"Cache-Control": "public, max-age=3600"})
    except Exception:
        from flask import Response
        return Response(b"", status=404, mimetype="image/png")

@app.route("/3d")
def index_3d():
    cesium_token = os.environ.get("CESIUM_ION_ACCESS_TOKEN", "")
    resp = app.make_response(render_template("3d.html", cesium_ion_token=cesium_token))
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    return resp


@app.route("/api/simulate", methods=["POST"])
def simulate():
    try:
        data = request.get_json() or {}
        north = float(data.get("north", 0))
        south = float(data.get("south", 0))
        east = float(data.get("east", 0))
        west = float(data.get("west", 0))
        altitudes = data.get("altitudes", [50.0])
        yaw_angles = data.get("yaw_angles", [0.0])
        num_positions = min(max(1, int(data.get("num_positions", 1))), 20)
        seed = int(data.get("seed", 42))
        zoom = int(data.get("zoom", 18))
        if north <= south or east <= west:
            return jsonify({"error": "Invalid bbox: north>south, east>west"}), 400
        # Optional estimator overrides (for UI tuning)
        config_override = data.get("config_override") or {}
        nfeatures_orb = min(500, max(50, int(config_override.get("nfeatures_orb", 180))))
        min_matches = min(20, max(1, int(config_override.get("min_matches", 3))))
        ratio_threshold = min(0.99, max(0.5, float(config_override.get("ratio_threshold", 0.9))))
        n_clusters = min(10, max(1, int(config_override.get("n_clusters", 3))))
        # Cache reference region on disk for reproducible/fast reruns
        cache_id = _cache_key(north, south, east, west, zoom=zoom, max_ref_px=MAX_REF_PX)
        cached_region = _cache_region_dir(CACHE_ROOT, cache_id)
        if not _cache_region_ready(cached_region):
            try:
                image_rgb, center_lat, center_lon, resolution_m = fetch_imagery_bbox(north, south, east, west, zoom=zoom)
            except Exception as fetch_err:
                return jsonify({"error": f"Tile fetch failed: {fetch_err}"}), 503
            _create_region_from_image(image_rgb, center_lat, center_lon, resolution_m, cached_region)

        from vps_device.reference_loader import load_proxigo_region
        from vps_device.simulation import run_simulation
        from vps_device.config import VPSDeviceConfig
        ref = load_proxigo_region(cached_region)
        # Web: balance speed and success rate; relax K-means; same-scale + template fallback
        web_config = VPSDeviceConfig(
            width_px=320,
            height_px=180,
            nfeatures_orb=nfeatures_orb,
            min_matches=min_matches,
            ratio_threshold=ratio_threshold,
            max_dist_from_last_m=200.0,
            max_dist_from_cluster_median_m=50.0,
            n_clusters=n_clusters,
            simulation_same_scale=True,
        )
        results = run_simulation(
            ref,
            config=web_config,
            altitudes_m=altitudes,
            yaw_angles_deg=yaw_angles,
            num_random_positions=num_positions,
            seed=seed,
            return_debug_images=True,
        )
        out = [
            {
                "altitude_m": _to_json_float(r.altitude_m),
                "yaw_deg": _to_json_float(r.yaw_deg),
                "truth_lat": _to_json_float(r.truth_lat),
                "truth_lon": _to_json_float(r.truth_lon),
                "est_lat": _to_json_float(r.est_lat),
                "est_lon": _to_json_float(r.est_lon),
                "success": bool(r.success),
                "error_m": _to_json_float(r.error_m) if r.success else None,
                "confidence": _to_json_float(r.confidence),
                "n_matches": int(r.n_matches) if r.n_matches is not None else None,
                "debug_image_b64": getattr(r, "debug_image_b64", None),
            }
            for r in results
        ]
        success_count = sum(1 for r in results if r.success)
        avg_error = sum(r.error_m for r in results if r.success) / success_count if success_count else None
        # Best-effort git hash for reproducibility metadata
        git_hash = None
        try:
            import subprocess
            git_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=str(REPO_ROOT)).decode("utf-8").strip()
        except Exception:
            git_hash = None

        return jsonify({
            "results": out,
            "summary": {
                "total": len(results),
                "success_count": success_count,
                "avg_error_m": round(float(avg_error), 2) if avg_error is not None and avg_error == avg_error else None,
                "center_lat": _to_json_float(center_lat),
                "center_lon": _to_json_float(center_lon),
            },
            "meta": {
                "bbox": {"north": north, "south": south, "east": east, "west": west},
                "zoom": zoom,
                "seed": seed,
                "cache_id": cache_id,
                "max_ref_px": MAX_REF_PX,
                "config_override": {
                    "nfeatures_orb": nfeatures_orb,
                    "min_matches": min_matches,
                    "ratio_threshold": ratio_threshold,
                    "n_clusters": n_clusters,
                },
                "git_hash": git_hash,
            },
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/simulate_trajectory", methods=["POST"])
def simulate_trajectory():
    """
    Run a short VO-style trajectory simulation over the selected region.

    Request JSON:
      - north/south/east/west: bbox floats (required)
      - duration_s: total duration in seconds (default 5)
      - step_hz: simulation rate in Hz (default 5)
      - altitude_m: altitude for all frames (default 50)
      - speed_east_m_s, speed_north_m_s, yaw_rate_deg_s: optional motion parameters
      - seed: RNG seed for reproducible VO noise (default 42)
      - config_override: { nfeatures_orb, min_matches, ratio_threshold, n_clusters }
    """
    try:
        data = request.get_json() or {}
        north = float(data.get("north", 0))
        south = float(data.get("south", 0))
        east = float(data.get("east", 0))
        west = float(data.get("west", 0))
        if north <= south or east <= west:
            return jsonify({"error": "Invalid bbox: north>south, east>west"}), 400

        duration_s = float(data.get("duration_s", 5.0))
        step_hz = float(data.get("step_hz", 5.0))
        altitude_m = float(data.get("altitude_m", 50.0))
        seed = int(data.get("seed", 42))
        zoom = int(data.get("zoom", 18))
        speed_east_m_s = float(data.get("speed_east_m_s", 4.0))
        speed_north_m_s = float(data.get("speed_north_m_s", 0.0))
        yaw_rate_deg_s = float(data.get("yaw_rate_deg_s", 0.0))

        config_override = data.get("config_override") or {}
        nfeatures_orb = min(500, max(50, int(config_override.get("nfeatures_orb", 180))))
        min_matches = min(20, max(1, int(config_override.get("min_matches", 3))))
        ratio_threshold = min(0.99, max(0.5, float(config_override.get("ratio_threshold", 0.9))))
        n_clusters = min(10, max(1, int(config_override.get("n_clusters", 3))))

        # Cache reference region on disk as in /api/simulate
        cache_id = _cache_key(north, south, east, west, zoom=zoom, max_ref_px=MAX_REF_PX)
        cached_region = _cache_region_dir(CACHE_ROOT, cache_id)
        if not _cache_region_ready(cached_region):
            try:
                image_rgb, center_lat, center_lon, resolution_m = fetch_imagery_bbox(north, south, east, west, zoom=zoom)
            except Exception as fetch_err:
                return jsonify({"error": f"Tile fetch failed: {fetch_err}"}), 503
            _create_region_from_image(image_rgb, center_lat, center_lon, resolution_m, cached_region)
        else:
            # When cached, we still need center_lat/lon for metadata; load from metadata via loader.
            from vps_device.reference_loader import load_proxigo_region as _lp
            _tmp_ref = _lp(cached_region)
            center_lat = float(_tmp_ref.center_lat)
            center_lon = float(_tmp_ref.center_lon)

        from vps_device.reference_loader import load_proxigo_region
        from vps_device.simulation import generate_trajectory, run_trajectory_simulation
        from vps_device.config import VPSDeviceConfig

        ref = load_proxigo_region(cached_region)
        # Web configuration similar to /api/simulate
        web_config = VPSDeviceConfig(
            width_px=320,
            height_px=180,
            nfeatures_orb=nfeatures_orb,
            min_matches=min_matches,
            ratio_threshold=ratio_threshold,
            max_dist_from_last_m=200.0,
            max_dist_from_cluster_median_m=50.0,
            n_clusters=n_clusters,
            simulation_same_scale=True,
        )

        # Build trajectory starting near region center
        traj = generate_trajectory(
            ref,
            start_lat=center_lat,
            start_lon=center_lon,
            altitude_m=altitude_m,
            duration_s=duration_s,
            step_hz=step_hz,
            north_speed_m_s=speed_north_m_s,
            east_speed_m_s=speed_east_m_s,
            yaw_rate_deg_s=yaw_rate_deg_s,
            bbox=(north, south, east, west),
        )

        samples = run_trajectory_simulation(
            ref,
            config=web_config,
            trajectory=traj,
            altitude_m=altitude_m,
            seed=seed,
            return_debug_images=True,
        )

        # Convert to JSON-serializable dicts
        out_samples = []
        vps_errors = []
        vo_errors = []
        for s in samples:
            out_samples.append(
                {
                    "time_s": _to_json_float(s.time_s),
                    "altitude_m": _to_json_float(s.altitude_m),
                    "yaw_deg": _to_json_float(s.yaw_deg),
                    "truth_lat": _to_json_float(s.truth_lat),
                    "truth_lon": _to_json_float(s.truth_lon),
                    "vps_lat": _to_json_float(s.vps_lat),
                    "vps_lon": _to_json_float(s.vps_lon),
                    "vo_lat": _to_json_float(s.vo_lat),
                    "vo_lon": _to_json_float(s.vo_lon),
                    "error_vps_m": _to_json_float(s.error_vps_m),
                    "error_vo_m": _to_json_float(s.error_vo_m),
                    "confidence": _to_json_float(s.confidence),
                    "n_matches": int(s.n_matches),
                    "debug_image_b64": s.debug_image_b64,
                }
            )
            if s.error_vps_m == s.error_vps_m:
                vps_errors.append(s.error_vps_m)
            if s.error_vo_m == s.error_vo_m:
                vo_errors.append(s.error_vo_m)

        def _summary_stats(errs: List[float]) -> Tuple[Optional[float], Optional[float]]:
            if not errs:
                return None, None
            avg = sum(errs) / len(errs)
            return float(avg), float(max(errs))

        avg_vps, max_vps = _summary_stats(vps_errors)
        avg_vo, max_vo = _summary_stats(vo_errors)

        return jsonify(
            {
                "samples": out_samples,
                "summary": {
                    "total": len(samples),
                    "avg_error_vps_m": _to_json_float(avg_vps),
                    "max_error_vps_m": _to_json_float(max_vps),
                    "avg_error_vo_m": _to_json_float(avg_vo),
                    "max_error_vo_m": _to_json_float(max_vo),
                },
                "meta": {
                    "bbox": {"north": north, "south": south, "east": east, "west": west},
                    "zoom": zoom,
                    "seed": seed,
                    "cache_id": cache_id,
                    "max_ref_px": MAX_REF_PX,
                    "config_override": {
                        "nfeatures_orb": nfeatures_orb,
                        "min_matches": min_matches,
                        "ratio_threshold": ratio_threshold,
                        "n_clusters": n_clusters,
                    },
                },
            }
        )
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/localize_frame", methods=["POST"])
def localize_frame():
    """
    Localize an uploaded frame against a reference built from a bbox.
    Form-data:
      - frame: image file (jpeg/png)
      - north/south/east/west: bbox floats
      - altitude_m: float (optional, default 50)
      - seed_lat/seed_lon: optional pose seed for continuity
      - config_override: JSON string { nfeatures_orb, min_matches, ratio_threshold, n_clusters }
    """
    try:
        if "frame" not in request.files:
            return jsonify({"error": "Missing form-data file field: frame"}), 400
        f = request.files["frame"]
        raw = f.read()
        if not raw:
            return jsonify({"error": "Empty frame upload"}), 400

        north = float(request.form.get("north", "0"))
        south = float(request.form.get("south", "0"))
        east = float(request.form.get("east", "0"))
        west = float(request.form.get("west", "0"))
        if north <= south or east <= west:
            return jsonify({"error": "Invalid bbox: north>south, east>west"}), 400

        altitude_m = float(request.form.get("altitude_m", "50"))
        seed_lat = request.form.get("seed_lat", None)
        seed_lon = request.form.get("seed_lon", None)
        seed_lat_f = float(seed_lat) if seed_lat is not None else None
        seed_lon_f = float(seed_lon) if seed_lon is not None else None

        cfg_raw = request.form.get("config_override", "") or "{}"
        try:
            cfg_obj = json.loads(cfg_raw) if isinstance(cfg_raw, str) else (cfg_raw or {})
        except Exception:
            cfg_obj = {}

        nfeatures_orb = min(500, max(50, int(cfg_obj.get("nfeatures_orb", 180))))
        min_matches = min(50, max(1, int(cfg_obj.get("min_matches", 3))))
        ratio_threshold = min(0.99, max(0.5, float(cfg_obj.get("ratio_threshold", 0.9))))
        n_clusters = min(10, max(1, int(cfg_obj.get("n_clusters", 3))))

        # Decode image (OpenCV expects BGR)
        import numpy as np
        img_arr = np.frombuffer(raw, dtype=np.uint8)
        frame_bgr = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        if frame_bgr is None:
            return jsonify({"error": "Could not decode frame (expected jpeg/png)"}), 400

        try:
            image_rgb, center_lat, center_lon, resolution_m = fetch_imagery_bbox(north, south, east, west)
        except Exception as fetch_err:
            return jsonify({"error": f"Tile fetch failed: {fetch_err}"}), 503

        with tempfile.TemporaryDirectory(prefix="vps_localize_") as tmp:
            region_dir = Path(tmp) / "region"
            _create_region_from_image(image_rgb, center_lat, center_lon, resolution_m, region_dir)

            from vps_device.reference_loader import load_proxigo_region
            from vps_device.estimator import VPSEstimator
            from vps_device.config import VPSDeviceConfig

            ref = load_proxigo_region(region_dir)

            h, w = frame_bgr.shape[:2]
            cfg = VPSDeviceConfig(
                width_px=int(w),
                height_px=int(h),
                nfeatures_orb=nfeatures_orb,
                min_matches=min_matches,
                ratio_threshold=ratio_threshold,
                n_clusters=n_clusters,
                max_dist_from_last_m=200.0,
                max_dist_from_cluster_median_m=50.0,
            )
            est = VPSEstimator(ref, config=cfg)
            if seed_lat_f is not None and seed_lon_f is not None:
                est.set_last_position(seed_lat_f, seed_lon_f)

            result = est.estimate(frame_bgr, altitude_m=altitude_m)
            debug = est.get_debug_image(frame_bgr)
            debug_b64 = _png_b64(debug) if debug is not None else None

            return jsonify({
                "success": bool(result.success),
                "est_lat": _to_json_float(result.lat) if result.success else None,
                "est_lon": _to_json_float(result.lon) if result.success else None,
                "confidence": _to_json_float(result.confidence),
                "n_matches": int(result.n_matches),
                "debug_image_b64": debug_b64,
            })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def main():
    token = os.environ.get("MAPBOX_ACCESS_TOKEN", "")
    has_mapbox = token.startswith("pk.") and len(token) >= 30
    print("VPS Simulation Web App")
    if has_mapbox:
        print("Mapbox token found -> globe view (Death Valley) will be used.")
    else:
        print("No MAPBOX_ACCESS_TOKEN (or invalid) -> using Esri satellite (flat).")
        print("  To get the globe: set MAPBOX_ACCESS_TOKEN or add it to scripts/vps_sim_web/.env")
    print("Open http://127.0.0.1:5000 in your browser.")
    print("Draw a rectangle on the map, then click 'Download & Run Simulation'.")
    if socketio is not None:
        socketio.run(app, host="0.0.0.0", port=5000, debug=False)
    else:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

# Ensure CESIUM_ION_ACCESS_TOKEN is loaded from .env even without python-dotenv
if not os.environ.get("CESIUM_ION_ACCESS_TOKEN") and _env_file.exists():
    with open(_env_file, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, _, value = line.partition("=")
                key, value = key.strip(), value.strip().strip('"').strip("'")
                if key == "CESIUM_ION_ACCESS_TOKEN" and value:
                    os.environ["CESIUM_ION_ACCESS_TOKEN"] = value
                    break
