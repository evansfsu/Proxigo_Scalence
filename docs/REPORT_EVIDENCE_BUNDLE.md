# Report Evidence Bundle (Current Snapshot)

Use this bundle as the source set while writing/refining the report narrative.

## Benchmark Command

See `docs/RECRUITER_REPRO_EVAL.md` for the exact replay command and media generation command.

## Core Evidence Files

- Replay telemetry CSV:
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_results.csv`
- Metrics JSON:
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_metrics.json`
- Final benchmark figures:
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_error_timeline.png`
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_traj_gt_vs_fused.png`
  - `portfolio_assets/real_eval/final_uavvisloc_seq07_baseline_summary_board.png`

## Supporting Selection Artifacts

- Sequence baseline scan ranking:
  - `portfolio_assets/real_eval/scan_ranking_summary.json`
- Sequence-07 tuning ranking:
  - `portfolio_assets/real_eval/seq07_tuning_summary.json`

## Suggested Report Paragraph Seed

"Primary quantitative evidence in this draft comes from a deterministic replay benchmark on UAV-VisLoc sequence 07, with per-frame ground-truth error computed from estimated vs labeled latitude/longitude. The current selected baseline configuration yields approximately 85.8 m mean error and 99.8 m RMS error over 30 frames. This should be interpreted as a reproducible baseline for methodology and trend tracking rather than a final field-validated deployment claim."
