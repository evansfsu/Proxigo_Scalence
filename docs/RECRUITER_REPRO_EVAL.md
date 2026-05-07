# Recruiter Reproducible VPS Evaluation

This document defines a ground-truth-backed replay that can be rerun by recruiters with a single command, plus deterministic media generation for report figures.

## Benchmark Dataset Path

- Dataset family: `UAV-VisLoc` under `test_data/uav_visloc`.
- Sequence selection process:
  - Baseline scan run artifacts: `portfolio_assets/real_eval/scan_ranking_summary.json`
  - Winner in current repo snapshot: sequence `07` (lowest RMS among scanned sequences).
- Final selected configuration:
  - `--nfeatures 2000`
  - `--map-match-every 5`
  - `--min-map-inliers 15`
  - `--map-noise 30`
  - `--topk-map-update 1`

## Reproduce Exact Benchmark Run

From repository root:

```bash
python scripts/vps_live.py \
  --source-dir test_data/uav_visloc/07/drone \
  --source-csv test_data/uav_visloc/07/07.csv \
  --reference test_data/uav_visloc/07 \
  --altitude 689 \
  --headless \
  --max-frames 120 \
  --nfeatures 2000 \
  --map-match-every 5 \
  --min-map-inliers 15 \
  --map-noise 30 \
  --topk-map-update 1 \
  --output-csv portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_results.csv
```

Expected terminal summary (approximately):
- `Processed 30 frames`
- `Mean error vs ground truth: ~85.8 m`

## Generate Final Report Media (Deterministic)

```bash
python scripts/generate_real_eval_media.py \
  --input-csv portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_results.csv \
  --output-dir portfolio_assets/real_eval \
  --prefix final_uavvisloc_seq07_baseline \
  --title "UAV-VisLoc Seq07 GT Replay" \
  --config-tag "seq07_baseline_nfeatures2000_mapEvery5_inliers15"
```

Expected generated files:
- `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_metrics.json`
- `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_error_timeline.png`
- `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_traj_gt_vs_fused.png`
- `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_summary_board.png`

## What This Validates

- Absolute horizontal error against per-frame ground truth in the selected sequence.
- Reproducibility of metrics and plots from a fixed command and fixed dataset files.

## Known Limits (Important for Conservative Claims)

- This is a replay benchmark, not a live flight trial.
- Results are sequence-specific and do not imply universal field accuracy.
- In current runs, accepted map matches at `>=15` inliers are rare; the system relies heavily on VO/fusion continuity.
- Use this as evidence of benchmark methodology and current baseline performance, not final deployment-grade accuracy.

## Report Evidence Bundle

Use these artifacts directly in the report:
- Final command: this document, `Reproduce Exact Benchmark Run`.
- Metrics JSON: `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_metrics.json`.
- Figures:
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_error_timeline.png`
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_traj_gt_vs_fused.png`
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_summary_board.png`
