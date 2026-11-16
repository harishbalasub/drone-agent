# Measured verification record

Date: 2026-11-11. These are individual integration checks, not statistical success-rate estimates.

## Controller calibration (actual Gazebo physics)

Outer-loop gain search on short near-home flights; full delivery/gate tasks held out. Payload feedforward is manually configured. No neural/VLA model was trained.

| Gain | Duration | Objective (lower is better) |
|---|---|---|
| 0.5 | 20.70 s | 28.120 |
| 0.8 | 13.44 s | 18.424 |
| 1.1 | 11.97 s | 12.249 |

Selected gain: **1.1**. The default world configuration uses this gain.

## Mission evaluations

| Episode | Transport | Verified views | Outcome | Duration | Expected behavior |
|---|---|---|---|---|---|
| clear_gate_trained | gazebo_native | 2 | MISSION_COMPLETED | 75.0 s | PASS |
| blocked_gate_abort | gazebo_native | 2 | MISSION_ABORTED_SAFE_RETURN_COMPLETED | 53.7 s | PASS |
| blocked_gate_bypass_three_views | gazebo_native | 3 | MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS | 74.5 s | PASS |
| final_native_regression | gazebo_native | 2 | MISSION_COMPLETED | 74.9 s | PASS |
| ros2_clear_gate | ros2 | 2 | MISSION_COMPLETED | 75.3 s | PASS |

Native evaluations use Gazebo rotor dynamics, physical attachment/release and an independent world-pose subscriber to check final home/package positions. ROS evaluation traverses real DDS and `ros_gz_bridge`; its report uses executor sensor evidence. Both use **ideal logical-camera perception** and simulated motion-capture pose, not learned image localization.

A blocked-gate abort passes the **recovery requirement**; the original delivery mission is aborted. The package returns still attached. Bypass success explicitly waives traversal and preserves delivery/home requirements.

`clear_gate_baseline` preserves an earlier diagnostic failure: gate status was unknown because the cameras were not oriented to survey both posts. The controller aborted and returned, and the evaluator correctly marked that clear-gate mission as failed. The implemented survey maneuver resolves that issue. This failure is not included as a successful evaluation.

## Rendered sensor and browser checks

- Real Gazebo RGB frames received from front, down and external cameras at 256×256; sample PNGs in `sensor_preview/`.
- Downward float32 depth image received; initial center-depth measurement approximately 0.340 m.
- Browser default mission launched through the actual ROS gateway with rendering enabled; status reached `MISSION_COMPLETED`, with two inspection observations, placement and home evidence.
- The live external 3D scene and both camera previews rendered in the browser; screenshot in `sensor_preview/app_completed.jpg`.
- External save button wrote the session and pipeline artifacts to `session_logs/ros2_ui_verification/20261111T233556Z_2b73db51`.
- The browser run is additional to the CLI evaluations above; its session was exported through the real save endpoint.

ROS dependencies and Gazebo rendering resources were restored into isolated local overlays without installing or upgrading system packages. The project-local setup helper was then run successfully, and `doctor.sh` passed. No DGX Spark execution was performed.

## Regression checks

18 new unit/contract/HTTP/UI tests passed, including missing viewpoints, ordering, unsupported constraints, stale odometry, gate unknown/blocked/clear distinction and payload loss. Original four Python server/export checks and the original viewpoint/scenario JS suites passed. SHA256 checks confirmed the original HTML and server were unchanged.

The rollout logs include UTC timestamps and local monotonic receipt times. Check `session.json` for event-by-event evidence, interrupts and revisions, and `report.json` for machine-readable outcomes.
