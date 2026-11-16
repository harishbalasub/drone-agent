# Ubuntu CPU RGB-D pilot

Completed 30 epochs using 56 training frames from 14 scene seeds. Checkpoint
`best.pt` is epoch 10, selected using the 12 validation frames (three different
scene seeds), not the test set. Later epochs overfit the small training set.

| Held-out test measurement | Result |
| --- | --- |
| Frames / unseen scene seeds | 12 / 3 |
| Package mask IoU | 0.7683 |
| Median XY position error | 1.34 cm |
| 95th percentile XY error | 2.90 cm |
| Visible targets localized | 8 / 8 |
| False accepts on absent frames | 0 / 4 |
| Median CPU inference + geometry latency | 29.2 ms |

This is a small synthetic pilot, not deployment validation. Geometry assumes an
upright package of known 0.24 m height, registered depth, and known camera pose.
Four negative frames cannot establish a low operational false-accept rate. No
flight policy was trained, and the mission runtime still uses its semantic sensor.

- `training.json`: all 30 epochs and training configuration.
- `validation.json`, `test.json`: learned localization evaluations.
- `best.pt`: trained two-class LR-ASPP MobileNetV3 checkpoint.
- `run_metadata.json`: exact command, dataset hash, local device and DGX discovery.
- `environment.txt`: installed Python packages.

Ubuntu's NVIDIA driver was unavailable, so CPU training used two threads and
batch size 2. NVIDIA Sync's saved `spark-harish` SSH connection was discovered
and verified against the Spark's NVIDIA GB10. No remote training was performed.
All 25 project regression tests passed after these changes.
