import hashlib
import json
from pathlib import Path


def cache_key(
    north: float,
    south: float,
    east: float,
    west: float,
    zoom: int,
    max_ref_px: int,
    provider: str = "esri_world_imagery",
) -> str:
    """
    Stable cache key for a bbox + fetch parameters.
    We round coordinates to limit key explosion from UI jitter.
    """
    payload = {
        "north": round(float(north), 7),
        "south": round(float(south), 7),
        "east": round(float(east), 7),
        "west": round(float(west), 7),
        "zoom": int(zoom),
        "max_ref_px": int(max_ref_px),
        "provider": provider,
    }
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha1(raw).hexdigest()[:16]


def region_dir(cache_root: Path, key: str) -> Path:
    return cache_root / key


def region_ready(p: Path) -> bool:
    return (p / "metadata.json").exists() and (p / "satellite.png").exists()

