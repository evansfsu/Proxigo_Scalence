#!/usr/bin/env python3
"""Generate comparison charts for ground-truth benchmark CSV runs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _read_errors(path: Path) -> list[float]:
    vals: list[float] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            v = row.get("error_m")
            if not v:
                continue
            try:
                vals.append(float(v))
            except ValueError:
                continue
    return vals


def _stats(errs: list[float]) -> dict[str, float]:
    a = np.asarray(errs, dtype=np.float64)
    return {
        "frames": int(a.size),
        "mean_error_m": float(np.mean(a)),
        "rms_error_m": float(np.sqrt(np.mean(a * a))),
        "max_error_m": float(np.max(a)),
        "p50_error_m": float(np.percentile(a, 50)),
        "p90_error_m": float(np.percentile(a, 90)),
        "p95_error_m": float(np.percentile(a, 95)),
    }


def _plot_bar(summary: dict[str, dict], out: Path) -> None:
    names = list(summary.keys())
    rms = [summary[n]["rms_error_m"] for n in names]
    p95 = [summary[n]["p95_error_m"] for n in names]
    x = np.arange(len(names))
    w = 0.35
    fig, ax = plt.subplots(figsize=(8.8, 5.2), dpi=160)
    ax.bar(x - w / 2, rms, width=w, label="RMS")
    ax.bar(x + w / 2, p95, width=w, label="P95")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=10)
    ax.set_ylabel("Error (m)")
    ax.set_title("Ground-Truth Benchmark Comparison: RMS vs P95")
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def _plot_cdf(runs: dict[str, list[float]], out: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 5.2), dpi=160)
    for name, errs in runs.items():
        x = np.sort(np.asarray(errs, dtype=np.float64))
        y = np.arange(1, len(x) + 1) / len(x)
        ax.plot(x, y, lw=2, label=name)
    ax.set_title("Ground-Truth Benchmark Error CDF")
    ax.set_xlabel("Horizontal error (m)")
    ax.set_ylabel("CDF")
    ax.set_ylim(0, 1)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def main() -> int:
    p = argparse.ArgumentParser(description="Generate GT benchmark comparison media.")
    p.add_argument("--real-eval-dir", default="portfolio_assets/real_eval")
    p.add_argument("--out-dir", default="portfolio_assets/real_eval")
    args = p.parse_args()

    real_eval = Path(args.real_eval_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    run_files = {
        "seq07_baseline_30f": real_eval / "final_uavvisloc_seq07_baseline_results.csv",
        "seq03_long_100f": real_eval / "final_uavvisloc_seq03_long_results.csv",
    }

    run_errs: dict[str, list[float]] = {}
    summary: dict[str, dict] = {}
    for name, path in run_files.items():
        if not path.is_file():
            continue
        errs = _read_errors(path)
        if not errs:
            continue
        run_errs[name] = errs
        summary[name] = _stats(errs)

    if len(summary) < 1:
        raise SystemExit("No benchmark result files found.")

    _plot_bar(summary, out_dir / "gt_benchmark_compare_rms_p95.png")
    _plot_cdf(run_errs, out_dir / "gt_benchmark_compare_cdf.png")
    (out_dir / "gt_benchmark_compare_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Wrote GT benchmark comparison outputs to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
