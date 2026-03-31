#!/usr/bin/env python3
import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1.0 - a)))
    return r * c


def _read_gt(gt_csv: Path) -> Dict[str, Tuple[float, float]]:
    out: Dict[str, Tuple[float, float]] = {}
    with gt_csv.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            name = str(row["filename"]).strip()
            out[name] = (float(row["lat"]), float(row["lon"]))
    return out


def _read_sorted_names(source_dir: Path) -> List[str]:
    names = [p.name for p in source_dir.iterdir() if p.is_file()]
    names.sort()
    return names


def _read_simple_rows(simple_raw_csv: Path) -> List[dict]:
    rows: List[dict] = []
    with simple_raw_csv.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def _build_simple_with_error(
    simple_rows: List[dict],
    sorted_names: List[str],
    gt_by_name: Dict[str, Tuple[float, float]],
    out_csv: Path,
) -> List[dict]:
    out_rows: List[dict] = []
    for row in simple_rows:
        idx = int(row["frame"])
        if idx < 0 or idx >= len(sorted_names):
            continue
        name = sorted_names[idx]
        gt = gt_by_name.get(name)
        if gt is None:
            continue

        lat = float(row["lat"])
        lon = float(row["lon"])
        success = str(row.get("success", "")).lower() == "true"
        err = (
            _haversine_m(lat, lon, gt[0], gt[1])
            if success and abs(lat) > 0.0 and abs(lon) > 0.0
            else float("inf")
        )

        out_rows.append(
            {
                "frame": idx,
                "filename": name,
                "pred_lat": lat,
                "pred_lon": lon,
                "truth_lat": gt[0],
                "truth_lon": gt[1],
                "confidence": float(row.get("confidence", 0.0) or 0.0),
                "n_matches": int(row.get("n_matches", 0) or 0),
                "success": int(success),
                "error_m": err,
                "map_stage": "accepted" if success else "match_fail",
            }
        )

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "frame",
                "filename",
                "pred_lat",
                "pred_lon",
                "truth_lat",
                "truth_lon",
                "confidence",
                "n_matches",
                "success",
                "error_m",
                "map_stage",
            ],
        )
        w.writeheader()
        w.writerows(out_rows)
    return out_rows


def _read_current_rows(current_csv: Path) -> List[dict]:
    out: List[dict] = []
    with current_csv.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            row["error_m"] = float(row["error_m"])
            row["accepted"] = int(row.get("accepted", 0) or 0)
            out.append(row)
    return out


def _percentile(sorted_vals: List[float], q: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    q = min(max(q, 0.0), 1.0)
    idx = (len(sorted_vals) - 1) * q
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    frac = idx - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def _safe_mean(vals: List[float]) -> float:
    return sum(vals) / len(vals) if vals else float("nan")


def _safe_median(vals: List[float]) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    n = len(s)
    mid = n // 2
    if n % 2:
        return s[mid]
    return (s[mid - 1] + s[mid]) / 2.0


def _summarize_rows(rows: List[dict], names: List[str]) -> dict:
    filt = [r for r in rows if r.get("filename") in set(names)]
    errs = [float(r["error_m"]) for r in filt if math.isfinite(float(r["error_m"]))]
    errs_sorted = sorted(errs)

    h10 = sum(1 for e in errs if e < 10.0)
    h20 = sum(1 for e in errs if e < 20.0)
    accepted = sum(int(r.get("accepted", r.get("success", 0)) or 0) for r in filt)
    stage_counter = Counter(str(r.get("map_stage", "unknown")) for r in filt)

    return {
        "count": len(filt),
        "mean_m": _safe_mean(errs),
        "median_m": _safe_median(errs),
        "p80_m": _percentile(errs_sorted, 0.8),
        "hit_10": h10,
        "hit_20": h20,
        "hit_10_rate": (h10 / len(filt)) if filt else 0.0,
        "hit_20_rate": (h20 / len(filt)) if filt else 0.0,
        "accepted": accepted,
        "accepted_rate": (accepted / len(filt)) if filt else 0.0,
        "stages": dict(stage_counter),
    }


def _fmt(v: float) -> str:
    if isinstance(v, float) and math.isnan(v):
        return "nan"
    return f"{v:.2f}"


def build_report(
    split_json: Path,
    simple_summary: Dict[str, dict],
    current_summary: Dict[str, dict],
    out_md: Path,
) -> None:
    split_data = json.loads(split_json.read_text(encoding="utf-8"))
    counts = split_data.get("counts", {})
    total = int(counts.get("total", 0))
    lines: List[str] = []
    lines.append("# VPS 80% Feasibility Split Report")
    lines.append("")
    lines.append("## Scope")
    lines.append(
        f"- Dataset slice: `{split_data.get('region')}/{split_data.get('place')}`, total `{total}` frames (`train={counts.get('train')}`, `val={counts.get('val')}`, `test={counts.get('test')}`)."
    )
    lines.append("- Thresholds evaluated: `<10m` (strict), `<20m` (relaxed).")
    lines.append("- Mean/median/p80 are computed on finite errors; hit-rates and accepted-update ratios use all frames in each split.")
    lines.append("")
    lines.append("## Split Metrics")
    lines.append("")
    lines.append("| Baseline | Split | Mean (m) | Median (m) | P80 (m) | <10m | <20m | Accepted updates |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|")

    for label, summary in [("Simple legacy", simple_summary), ("Current best", current_summary)]:
        for split in ("train", "val", "test"):
            s = summary[split]
            lines.append(
                f"| {label} | {split} | {_fmt(s['mean_m'])} | {_fmt(s['median_m'])} | {_fmt(s['p80_m'])} | "
                f"{s['hit_10']}/{s['count']} ({100.0 * s['hit_10_rate']:.1f}%) | "
                f"{s['hit_20']}/{s['count']} ({100.0 * s['hit_20_rate']:.1f}%) | "
                f"{s['accepted']}/{s['count']} ({100.0 * s['accepted_rate']:.1f}%) |"
            )

    lines.append("")
    lines.append("## Stage Histograms (all splits combined)")
    lines.append("")
    for label, summary in [("Simple legacy", simple_summary), ("Current best", current_summary)]:
        comb = Counter()
        for split in ("train", "val", "test"):
            comb.update(summary[split]["stages"])
        chunks = ", ".join(f"`{k}={v}`" for k, v in sorted(comb.items()))
        lines.append(f"- {label}: {chunks if chunks else 'none'}")

    cur_test = current_summary["test"]
    lines.append("")
    lines.append("## Go/No-Go Against 80% Target")
    lines.append("")
    if cur_test["hit_10_rate"] >= 0.80:
        lines.append("- Decision: **GO** for `<10m @ 80%` on test split.")
    elif cur_test["hit_20_rate"] >= 0.80:
        lines.append("- Decision: **PARTIAL GO** for `<20m @ 80%`, but **NO-GO** for `<10m @ 80%`.")
    else:
        lines.append("- Decision: **NO-GO** for both `<10m @ 80%` and `<20m @ 80%` on test split.")

    lines.append(
        "- Primary indicator: accepted absolute updates and tail error (`p80`) remain the main bottleneck for strict 80% feasibility."
    )
    lines.append("")

    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    p = argparse.ArgumentParser(description="Build split-wise feasibility report for VPS baselines.")
    p.add_argument("--source-dir", required=True)
    p.add_argument("--gt-csv", required=True)
    p.add_argument("--split-json", required=True)
    p.add_argument("--simple-raw-csv", required=True)
    p.add_argument("--current-csv", required=True)
    p.add_argument("--simple-out-csv", required=True)
    p.add_argument("--report-md", required=True)
    args = p.parse_args()

    source_dir = Path(args.source_dir).resolve()
    gt_csv = Path(args.gt_csv).resolve()
    split_json = Path(args.split_json).resolve()
    simple_raw_csv = Path(args.simple_raw_csv).resolve()
    current_csv = Path(args.current_csv).resolve()
    simple_out_csv = Path(args.simple_out_csv).resolve()
    report_md = Path(args.report_md).resolve()

    gt_by_name = _read_gt(gt_csv)
    sorted_names = _read_sorted_names(source_dir)
    simple_rows = _read_simple_rows(simple_raw_csv)
    simple_eval = _build_simple_with_error(simple_rows, sorted_names, gt_by_name, simple_out_csv)
    current_eval = _read_current_rows(current_csv)

    split_data = json.loads(split_json.read_text(encoding="utf-8"))
    splits = split_data["splits"]

    simple_summary = {}
    current_summary = {}
    for split in ("train", "val", "test"):
        names = [str(x) for x in splits[split]]
        simple_summary[split] = _summarize_rows(simple_eval, names)
        current_summary[split] = _summarize_rows(current_eval, names)

    build_report(split_json, simple_summary, current_summary, report_md)
    print(f"Wrote: {simple_out_csv}")
    print(f"Wrote: {report_md}")


if __name__ == "__main__":
    main()
