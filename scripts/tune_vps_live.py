#!/usr/bin/env python3
"""
Parameter sweep for vps_live.py on a fixed image+GT dataset.

Focus: camera-only positioning (no IMU) accuracy + drift behavior.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PY = sys.executable
VPS_LIVE = ROOT / "scripts" / "vps_live.py"

SOURCE_DIR = ROOT / "test_data" / "anyvis_qz" / "images_nadir"
SOURCE_CSV = ROOT / "test_data" / "anyvis_qz" / "ground_truth_nadir.csv"
REFERENCE = ROOT / "test_data" / "anyvis_qz"
OUT_DIR = ROOT / "test_data" / "anyvis_qz" / "tuning_runs"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def compute_metrics(csv_path: Path) -> dict:
    errors = []
    map_success = 0
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("map_lat"):
                map_success += 1
            if row.get("error_m"):
                errors.append(float(row["error_m"]))
    if not errors:
        return {
            "mean": 1e9,
            "median": 1e9,
            "p80": 1e9,
            "lt10": 0.0,
            "lt50": 0.0,
            "lt100": 0.0,
            "last": 1e9,
            "map_success": map_success,
            "n": 0,
        }
    errors_sorted = sorted(errors)
    p80 = errors_sorted[min(len(errors_sorted) - 1, int(0.8 * (len(errors_sorted) - 1)))]
    n = len(errors)
    return {
        "mean": statistics.mean(errors),
        "median": statistics.median(errors),
        "p80": p80,
        "lt10": sum(e < 10 for e in errors) / n,
        "lt50": sum(e < 50 for e in errors) / n,
        "lt100": sum(e < 100 for e in errors) / n,
        "last": errors[-1],
        "map_success": map_success,
        "n": n,
    }


def _cfg_key(cfg: dict) -> str:
    fields = [
        "flow",
        "ratio",
        "map_every",
        "min_inliers",
        "map_noise",
        "max_dist",
        "cal_frames",
        "nfeatures",
        "beblid",
    ]
    payload = {k: cfg[k] for k in fields}
    return json.dumps(payload, sort_keys=True)


def run_one(
    i: int,
    cfg: dict,
    args: argparse.Namespace,
    label: str = "run",
    start_frame_override: int | None = None,
    max_frames_override: int | None = None,
) -> tuple[dict, dict]:
    out_csv = OUT_DIR / f"{label}_{i:03d}.csv"
    cmd = [
        PY,
        str(VPS_LIVE),
        "--source-dir",
        str(SOURCE_DIR),
        "--source-csv",
        str(SOURCE_CSV),
        "--reference",
        str(REFERENCE),
        "--headless",
        "--output-csv",
        str(out_csv),
        "--fov-h",
        "71.5",
        "--fov-d",
        "84",
        "--width",
        "4000",
        "--height",
        "3000",
        "--nfeatures",
        str(cfg["nfeatures"]),
        "--ratio",
        str(cfg["ratio"]),
        "--max-dist",
        str(cfg["max_dist"]),
        "--map-match-every",
        str(cfg["map_every"]),
        "--min-map-inliers",
        str(cfg["min_inliers"]),
        "--map-noise",
        str(cfg["map_noise"]),
        "--calibration-frames",
        str(cfg["cal_frames"]),
        "--matching-flow",
        cfg["flow"],
    ]
    start_frame = args.start_frame if start_frame_override is None else start_frame_override
    max_frames = args.max_frames if max_frames_override is None else max_frames_override
    if start_frame > 0:
        cmd += ["--start-frame", str(start_frame)]
    if args.frame_stride > 1:
        cmd += ["--frame-stride", str(args.frame_stride)]
    if max_frames is not None and max_frames > 0:
        cmd += ["--max-frames", str(max_frames)]
    if cfg["beblid"]:
        cmd.append("--beblid")
    print(f"[{i:03d}] {cfg}")
    cp = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    ok = cp.returncode == 0
    if not ok:
        return cfg, {"ok": False, "err": cp.stderr[-500:] if cp.stderr else cp.stdout[-500:]}
    m = compute_metrics(out_csv)
    m["ok"] = True
    return cfg, m


def baseline_metrics() -> dict:
    baseline_csv = ROOT / "test_data" / "anyvis_qz" / "verify_homography.csv"
    if not baseline_csv.exists():
        return {}
    return compute_metrics(baseline_csv)


def quick_configs() -> list[dict]:
    # Fast loop around the strongest known neighborhood.
    return [
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 12, "map_noise": 30.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.9, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 7, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 20.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
    ]


def full_configs() -> list[dict]:
    return [
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 20.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 40.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 3, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 7, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.9,  "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.8,  "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 12, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 18, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 100, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 160, "cal_frames": 5, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 8, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 3, "nfeatures": 2000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 3000, "beblid": False},
        {"flow": "homography", "ratio": 0.9,  "map_every": 5, "min_inliers": 15, "map_noise": 20.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 3000, "beblid": False},
        {"flow": "homography", "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 20.0, "max_dist": 100, "cal_frames": 5, "nfeatures": 3000, "beblid": False},
        {"flow": "cluster",    "ratio": 0.85, "map_every": 5, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 2000, "beblid": True},
        {"flow": "cluster",    "ratio": 0.85, "map_every": 7, "min_inliers": 15, "map_noise": 30.0, "max_dist": 120, "cal_frames": 5, "nfeatures": 3000, "beblid": True},
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune vps_live.py with fast or full profiles.")
    parser.add_argument("--profile", choices=["quick", "full", "auto"], default="quick",
                        help="quick: small config set, fast iterations. full: broader search.")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="Skip raw frames before this index.")
    parser.add_argument("--frame-stride", type=int, default=1,
                        help="Evaluate every Nth frame.")
    parser.add_argument("--max-frames", type=int, default=24,
                        help="Processed frames cap per run (default 24 for fast tuning).")
    parser.add_argument("--no-limit", action="store_true",
                        help="Disable max-frames and run full sequence.")
    parser.add_argument("--auto-window-size", type=int, default=24,
                        help="Window size per quick pass in auto mode (default 24).")
    parser.add_argument("--auto-windows", type=int, default=2,
                        help="Number of quick windows in auto mode (default 2).")
    parser.add_argument("--auto-confirm-topk", type=int, default=2,
                        help="How many quick winners to full-confirm in auto mode (default 2).")
    return parser.parse_args()


def score_tuple(m: dict) -> tuple:
    return (
        m["p80"],
        m["mean"],
        m["median"],
        -m["map_success"],
        -m["lt50"],
    )


def write_summary(scored: list[tuple], out_summary: Path) -> None:
    with open(out_summary, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "rank",
                "flow",
                "ratio",
                "map_every",
                "min_inliers",
                "map_noise",
                "max_dist",
                "beblid",
                "mean",
                "median",
                "p80",
                "lt10",
                "lt50",
                "lt100",
                "last",
                "map_success",
                "n",
            ]
        )
        for rank, (_, c, m) in enumerate(scored, start=1):
            writer.writerow(
                [
                    rank,
                    c["flow"],
                    c["ratio"],
                    c["map_every"],
                    c["min_inliers"],
                    c["map_noise"],
                    c["max_dist"],
                    c["beblid"],
                    round(m["mean"], 3),
                    round(m["median"], 3),
                    round(m["p80"], 3),
                    round(m["lt10"], 5),
                    round(m["lt50"], 5),
                    round(m["lt100"], 5),
                    round(m["last"], 3),
                    m["map_success"],
                    m["n"],
                ]
            )


def evaluate_configs(
    configs: list[dict],
    args: argparse.Namespace,
    label: str,
    start_frame_override: int | None = None,
    max_frames_override: int | None = None,
) -> tuple[list[tuple], list[tuple]]:
    results = []
    for i, cfg in enumerate(configs, start=1):
        c, m = run_one(
            i,
            cfg,
            args,
            label=label,
            start_frame_override=start_frame_override,
            max_frames_override=max_frames_override,
        )
        results.append((c, m))

    scored = []
    for c, m in results:
        if not m.get("ok"):
            continue
        scored.append((score_tuple(m), c, m))
    scored.sort(key=lambda x: x[0])
    return scored, results


def print_baseline() -> None:
    base = baseline_metrics()
    if not base:
        return
    print("Baseline (verify_homography.csv):")
    print(
        {
            "mean": round(base["mean"], 3),
            "median": round(base["median"], 3),
            "p80": round(base["p80"], 3),
            "lt10": round(base["lt10"], 5),
            "lt50": round(base["lt50"], 5),
            "lt100": round(base["lt100"], 5),
            "map_success": base["map_success"],
            "n": base["n"],
        }
    )


def run_standard(args: argparse.Namespace) -> int:
    configs = quick_configs() if args.profile == "quick" else full_configs()
    scored, results = evaluate_configs(configs, args, label="run")
    out_summary = OUT_DIR / "summary.csv"
    write_summary(scored, out_summary)
    print(f"\nCompleted {len(results)} runs. Summary: {out_summary}")
    if scored:
        _, best_c, best_m = scored[0]
        print("Best config:")
        print(best_c)
        print(best_m)
    print_baseline()
    return 0


def run_auto(args: argparse.Namespace) -> int:
    configs = quick_configs()
    window_size = args.max_frames if args.max_frames is not None else args.auto_window_size
    if window_size is None:
        window_size = args.auto_window_size

    starts = [args.start_frame + i * window_size for i in range(args.auto_windows)]
    aggregate: dict[str, dict] = {}

    for w_idx, start in enumerate(starts, start=1):
        label = f"auto_w{w_idx}"
        print(f"\n--- Auto quick window {w_idx}/{len(starts)}: start={start}, frames={window_size} ---")
        scored, _ = evaluate_configs(
            configs,
            args,
            label=label,
            start_frame_override=start,
            max_frames_override=window_size,
        )
        write_summary(scored, OUT_DIR / f"summary_{label}.csv")
        for _, cfg, m in scored:
            key = _cfg_key(cfg)
            if key not in aggregate:
                aggregate[key] = {"cfg": cfg, "metrics": []}
            aggregate[key]["metrics"].append(m)

    aggregate_scored = []
    for payload in aggregate.values():
        cfg = payload["cfg"]
        ms = payload["metrics"]
        merged = {
            "mean": statistics.mean([m["mean"] for m in ms]),
            "median": statistics.mean([m["median"] for m in ms]),
            "p80": statistics.mean([m["p80"] for m in ms]),
            "lt10": statistics.mean([m["lt10"] for m in ms]),
            "lt50": statistics.mean([m["lt50"] for m in ms]),
            "lt100": statistics.mean([m["lt100"] for m in ms]),
            "last": statistics.mean([m["last"] for m in ms]),
            "map_success": sum(m["map_success"] for m in ms),
            "n": sum(m["n"] for m in ms),
        }
        aggregate_scored.append((score_tuple(merged), cfg, merged))
    aggregate_scored.sort(key=lambda x: x[0])
    write_summary(aggregate_scored, OUT_DIR / "summary_auto_quick_aggregate.csv")

    topk = max(1, min(args.auto_confirm_topk, len(aggregate_scored)))
    finalists = [aggregate_scored[i][1] for i in range(topk)]
    print(f"\nAuto quick finalists (top {topk}):")
    for cfg in finalists:
        print(cfg)

    full_results = []
    print("\n--- Auto full confirmation (entire sequence) ---")
    for i, cfg in enumerate(finalists, start=1):
        c, m = run_one(
            i,
            cfg,
            args,
            label="auto_full",
            start_frame_override=0,
            max_frames_override=0,
        )
        if m.get("ok"):
            full_results.append((score_tuple(m), c, m))
    full_results.sort(key=lambda x: x[0])
    write_summary(full_results, OUT_DIR / "summary_auto_full.csv")
    write_summary(full_results, OUT_DIR / "summary.csv")

    print(f"\nCompleted auto mode. Summary: {OUT_DIR / 'summary_auto_full.csv'}")
    if full_results:
        _, best_c, best_m = full_results[0]
        print("Best config after full confirmation:")
        print(best_c)
        print(best_m)
    print_baseline()
    return 0


def main() -> int:
    args = parse_args()
    if args.no_limit:
        args.max_frames = None
    if args.start_frame < 0 or args.frame_stride < 1:
        raise SystemExit("start-frame must be >= 0 and frame-stride >= 1")
    if args.auto_windows < 1 or args.auto_window_size < 1 or args.auto_confirm_topk < 1:
        raise SystemExit("auto-windows, auto-window-size, and auto-confirm-topk must be >= 1")

    if args.profile == "auto":
        return run_auto(args)
    return run_standard(args)


if __name__ == "__main__":
    raise SystemExit(main())

