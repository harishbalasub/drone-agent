# DGX Spark CUDA pilot

Completed 30 epochs on NVIDIA GB10 using PyTorch 2.9.1+cu130 and torchvision
0.24.1. Selected epoch 12 by validation mask IoU. Same synthetic pilot
and scene-level splits as Ubuntu: 56 training, 12 validation, 12 test frames.

Held-out test: mask IoU 0.8211; median XY error 1.12 cm; p95 XY error 2.89 cm;
8/8 visible packages localized; 0/4 absent frames falsely accepted. Median GPU
inference plus depth geometry latency: 2.91 ms (not end-to-end ROS latency).

Small synthetic pilot only; four negatives cannot establish operational reliability.
Mission execution still uses semantic sensing. No drone-control policy was trained.

Remote project: `~/projects/drone-agent-rgbd`; NVIDIA Sync SSH alias: `spark-harish`.
Existing runtime: `~/projects/graphjepa-driving/runtime/bin/python` (reused without
installation or changes). Training used CUDA, batch size 2, 2 threads, 0 workers.
PyTorch emitted a capability warning about GB10 sm_121; CUDA training and evaluation
completed successfully. Evaluate runtime compatibility again before larger jobs.

`best.pt`, `training.json`, `validation.json`, and `test.json` are copied locally.
Spark is the user's preferred training host for subsequent runs.
