# VPS Module Roadmap (Concise)

## Purpose

Track the practical next steps to move this project from camera-only benchmarking to reliable field deployment.

## Current Baseline

- Camera-first VPS with multiple runners (`vps_live.py`, `vps_avl_benchmark.py`, `vps_vnav_like.py`)
- Map matching + VO + EKF fusion paths implemented
- ROS2/PX4 integration scaffolding present
- Software-only IMU/VIO simulation path available in the fusion stack

## Near-Term Priorities (Execution Order)

1. **Stabilize absolute updates**
   - Improve retrieval + local match quality on low-altitude slices
   - Increase accepted map updates without introducing large jumps

2. **Revisit VIO + Satellite fusion design**
   - Use the approach in `docs/VIO_SATELLITE_DESIGN.md` as a reference
   - Implement a minimal integration path first (predict from VO/IMU, update from satellite)

3. **Orin Nano deployment hardening**
   - Re-run camera calibration and latency checks on-device
   - Validate runtime stability and throughput under sustained operation

4. **ROS2/PX4 validation loop**
   - Publish fused pose through MAVROS-compatible topics/messages
   - Verify end-to-end timing and frame conventions in SITL before flight

## Exit Criteria

- Repeatable benchmark performance on selected sub-200m datasets
- Stable fused pose output over long sequences (no NaN states, bounded drift)
- Orin runtime validated with camera feed and logging enabled
- ROS2/PX4 integration validated in SITL and ready for field tests

## Out of Scope (for now)

- Productization timelines
- Manufacturing/certification planning
- Competitive landscape analysis

