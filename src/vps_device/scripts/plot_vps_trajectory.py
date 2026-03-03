#!/usr/bin/env python3
"""
Plot VPS trajectory from the CSV produced by run_vps_standalone.py --output.
Shows estimated (lat, lon) path and optional confidence over frame index.
"""

import argparse
import csv
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend by default
except ImportError:
    print("Install matplotlib: pip install matplotlib", file=sys.stderr)
    sys.exit(1)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot VPS trajectory from CSV (frame, lat, lon, confidence, n_matches, success)."
    )
    parser.add_argument("csv_file", type=str, help="Path to CSV from run_vps_standalone.py --output")
    parser.add_argument("--out", "-o", type=str, help="Save figure to this path (e.g. trajectory.png)")
    parser.add_argument("--show", action="store_true", help="Show interactive plot window")
    parser.add_argument("--no-confidence", action="store_true", help="Do not plot confidence subplot")
    args = parser.parse_args()

    path = Path(args.csv_file)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        return 1

    frames = []
    lats = []
    lons = []
    confidences = []
    successes = []

    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            frames.append(int(row[0]))
            lats.append(float(row[1]))
            lons.append(float(row[2]))
            confidences.append(float(row[3]) if len(row) > 3 else 0.0)
            successes.append(row[5].strip().lower() == "true" if len(row) > 5 else True)

    if not frames:
        print("No rows in CSV", file=sys.stderr)
        return 1

    n_plots = 2 if not args.no_confidence and confidences else 1
    fig, axes = plt.subplots(n_plots, 1, figsize=(8, 4 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]

    # Trajectory (lat, lon)
    ax = axes[0]
    ax.plot(lons, lats, "b-", alpha=0.7, label="Estimated path")
    ax.scatter(lons[0], lats[0], c="green", s=80, label="Start", zorder=5)
    ax.scatter(lons[-1], lats[-1], c="red", s=80, label="End", zorder=5)
    # Mark failed estimates
    fail_lons = [lons[i] for i in range(len(lons)) if not successes[i]]
    fail_lats = [lats[i] for i in range(len(lats)) if not successes[i]]
    if fail_lons:
        ax.scatter(fail_lons, fail_lats, c="orange", s=20, alpha=0.8, label="No match")
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("VPS estimated trajectory")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    if n_plots > 1 and confidences:
        ax2 = axes[1]
        ax2.plot(frames, confidences, "g-", alpha=0.8)
        ax2.set_xlabel("Frame")
        ax2.set_ylabel("Confidence")
        ax2.set_title("Match confidence")
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(0, 1.05)

    plt.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f"Saved: {args.out}")
    if args.show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
