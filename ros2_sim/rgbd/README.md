# RGB-D package localization: data and training pipeline

This extension prepares rendered data, an RGB segmentation training entry point,
and calibrated depth geometry. Optional `--rgbd` mode now replaces package
semantic observations in the mission runtime with the Spark-trained model.
Broader robustness evaluation remains pending; other scene sensing stays ideal.

## Dataset we need

Initial task: segment the **red package**. Package metric location comes from its
registered depth image, camera calibration and camera pose. Gate/obstacle perception
is a separate task. The initial geometric estimator assumes an upright package of
known 0.24 m height on a horizontal support. It is not a general 6D pose estimator.

Proposed first full dataset: **10,000 RGB-D pairs from 500 randomized scene seeds**,
20 views per seed, 70/15/15 train/validation/test by scene seed. This is a starting
budget, not a claim that a particular sample count guarantees accuracy. Increase
it based on validation learning curves and failure analysis. Approximately 20%
of scenes should omit the package. Include red non-package distractors, occlusion,
off-center views, lighting and floor variation. Prototype sampler currently varies
positions/yaw, camera pose, lighting, floor color and a few distractor primitives;
it does not yet cover realistic textures, motion blur or a real sensor noise model.

Every sample stores:

- RGB PNG and raw float32 depth NPY in meters.
- Pixel mask from Gazebo's segmentation camera (training label only).
- Exact common RGB/depth/mask simulation timestamp.
- Matching pinhole calibration and world-from-Gazebo-camera transform.
- Package center ground truth, visibility, scene seed and split (labels only).

No user-supplied image dataset is needed for the initial simulation experiment.
Before real flight, collect camera-specific real RGB-D data and perform a separate
sim-to-real evaluation. Synthetic scene performance is not evidence of real-world
performance.

## Collect

From the project root, using the existing local runtime:

```bash
source ros2_sim/.runtime/env.sh
/usr/bin/python3 ros2_sim/rgbd/collect.py \
  --output ros2_sim/datasets/rgbd_v1 --episodes 500 --views 20
```

The collector creates separate static-camera Gazebo scenes. It does not fly the
drone or train a flight policy. Each view keeps exact timestamp-matched RGB,
depth and segmentation data; unmatched captures fail rather than pairing arbitrary
latest frames. It starts one Gazebo process at a time. Completed episodes can be
skipped when resuming; incomplete episode folders must be inspected/moved before
retrying that seed. It never overwrites them silently.

## Ubuntu CPU pilot

Reproduce the first local run (a new output directory is required for each run):

```bash
TORCH_HOME=ros2_sim/.runtime/torch_cache \
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 \
ros2_sim/.runtime/rgbd_venv/bin/python ros2_sim/rgbd/train_segmentation.py \
  --dataset ros2_sim/datasets/rgbd_pilot_v2 \
  --output ros2_sim/results/rgbd_ubuntu_cpu_v1 \
  --epochs 30 --batch-size 2 --device cpu --workers 0 --threads 2
```

The pilot contains 80 frames from 20 scenes (56/12/12 by scene split). It is a
small training/evaluation experiment, not evidence of deployment readiness.
Dependencies are recorded in `requirements-cpu.txt` and the run's environment log.
Do not install these x86 CPU wheels onto the ARM64 Spark.

## Training on DGX Spark

NVIDIA Sync's saved `spark-harish` SSH profile was discovered and successfully
verified on 2026-11-11 (hostname `spark-5d76`, NVIDIA GB10 GPU). The profile lives
in `~/.config/NVIDIA/Sync/config/ssh_config`; its private key stays outside this
project. Connect using:

```bash
ssh -F "$HOME/.config/NVIDIA/Sync/config/ssh_config" spark-harish
```

Ubuntu's NVIDIA driver is currently unavailable (`nvidia-smi` cannot communicate
with it). The initial pilot training therefore uses explicit `--device cpu` in
`ros2_sim/.runtime/rgbd_venv`, with two CPU threads, batch size 2, and no loader
workers. This environment is separate from system Python. A 30-epoch DGX CUDA pilot has now completed using an existing runtime; no remote
installation was needed. Spark is the preferred host for subsequent training.

Use a Spark-compatible NVIDIA/PyTorch environment, verify CUDA and matching
`torch`/`torchvision` versions, and install NumPy/Pillow. The training script uses
an approximately 3.2M-parameter LR-ASPP MobileNetV3 model, with a pretrained RGB
backbone and a new background/package segmentation head. Model initialization may
download official torchvision weights. No labels or data are uploaded by the code.

After copying this module and the dataset to Spark:

```bash
python3 ros2_sim/rgbd/train_segmentation.py \
  --dataset ros2_sim/datasets/rgbd_v1 \
  --output ros2_sim/results/rgbd_training_v1 --device cuda --epochs 30
python3 ros2_sim/rgbd/evaluate_localization.py \
  --dataset ros2_sim/datasets/rgbd_v1 \
  --checkpoint ros2_sim/results/rgbd_training_v1/best.pt \
  --split test --device cuda --output ros2_sim/results/rgbd_test.json
```

Training uses RGB/masks only, with weighted cross entropy + Dice, AdamW, brightness
augmentation, and validation IoU checkpoint selection. Neither metric ground-truth
position nor depth is input to the neural network. Depth is used **after predicted
segmentation** to recover metric position. Test episodes are excluded from training
and checkpoint selection. The output training report records the actual history.

An optional `--oracle-mask` evaluation is a geometry diagnostic using perfect masks.
It must never be reported as learned localization performance.

## Completed pilot result

Run: [`rgbd_ubuntu_cpu_v1`](../results/rgbd_ubuntu_cpu_v1/README.md).
Completed 30 Ubuntu CPU epochs, selecting epoch 10 by validation IoU (0.7078).
Held-out test: 12 frames from three unseen scene seeds; mask IoU 0.7683,
median XY error 1.34 cm, p95 XY error 2.90 cm, 8/8 visible targets accepted,
0/4 absent frames falsely accepted. Median CPU localization latency was 29.2 ms
(including mask inference, depth loading and geometry; not an end-to-end ROS latency).
Four negatives cannot establish a 1% false-accept rate. Broader scenes, sensor noise,
occlusion and a mission-level test with ideal package sensing disabled are still needed.
The model uses RGB to segment and depth/calibration to locate; no flight policy
or grasp controller was trained by this experiment.

## How localization works

Predicted mask → valid depth pixels → pinhole unprojection → optical-to-Gazebo FLU
conversion → world transform → visible top-plane estimate → known-height correction
to package center. Invalid depth, insufficient pixels or missing top surface return
unknown. These checks do not establish calibrated confidence; uncertainty must be
calibrated on held-out residuals before enabling autonomous grasp acceptance.

Initial engineering targets (not achieved results): median XY error ≤3 cm, 95th
percentile ≤6 cm, visible-target recall ≥95%, false accepts on negatives ≤1%.
Report mask IoU, position errors, rejected/missed cases, false positives and latency.
Then rerun the full mission with ideal **package** observations disabled, measuring
pickup/delivery success and recovery. Retaining the old semantic package sensor as
an automatic fallback would invalidate that evaluation.

References: [torchvision model](https://docs.pytorch.org/vision/stable/models/generated/torchvision.models.segmentation.lraspp_mobilenet_v3_large.html),
[Gazebo segmentation camera](https://gazebosim.org/api/sensors/8/segmentationcamera_igngazebo.html).

## DGX Spark completed run

See [Spark results](../results/rgbd_spark_cuda_v1/README.md). The GPU run completed
30 epochs; test median XY error was 1.12 cm, p95 2.89 cm, mask IoU 0.8211,
and median inference-plus-geometry latency 2.91 ms. The 12-frame test set is too
small for deployment acceptance. Checkpoint and reports are stored locally and on Spark.

## Use the learned package estimate for pickup

From the project root:

```bash
bash ros2_sim/scripts/launch.sh 1 --render-headless --rgbd
```

Open `http://localhost:8100`, choose the ROS2/Gazebo backend, compile the mission,
and run it. The gateway summary reports `learned_rgbd_package; logical_camera_scene`.
Use scenario `2` in the command for the blocked-gate world. RGB-D mode requires
rendering; `--headless` is rejected. Leaving off `--rgbd` retains the ideal baseline.
The original v2 HTML app is unchanged.

`live_bridge.py` consumes exact timestamp-matched down-camera RGB/depth, uses the
nearest drone odometry pose within 50 ms, and applies the full quaternion camera
extrinsic. It sends RGB/depth/calibration to a persistent local CPU model worker
using the Spark-trained checkpoint. It publishes `/drone/perception/package` as
JSON in `gz.msgs.StringMsg`, bridged to ROS2 `std_msgs/String`. Estimates include
simulation timestamp, source, checkpoint, pose skew, latency and geometric quality.
No package world pose or logical-camera package observation is an inference input.

When enabled, both native and ROS2 transports discard ideal `red_package`
observations. Missing learned observations age out and cause normal mission recovery;
there is no automatic fallback to ideal package sensing. Other objects, including
gate posts/blockers, still use ideal semantics. Camera pose remains simulated mocap.

Pickup aligns from a view at least 0.95 m above ground, logs a fresh learned estimate,
locks it for the short final descent, checks odometry alignment, commands the physical
attachment joint, and verifies attached feedback during a 0.3 m test lift. This is
an attachment/odometry verification, not a visual claim about a partly occluded held
package. Placement is checked again with learned RGB-D after release and ascent.

The static-rig dataset omitted the drone's own fingers/legs. The live worker therefore
excludes depth ≤0.35 m and rejects package-center heights outside ±0.08 m of the
supported 0.12 m ground / 0.62 m platform centers. These are explicit world-specific
geometry assumptions, not calibrated confidence or general object localization.
Packages at arbitrary heights and realistic grasp mechanics need further work.

Verified end-to-end native mission: `../results/rgbd_pickup_v2/report.json` and
`session.json`: three inspections, learned pickup, gate traversal, delivery and
return home, no collision, independent evaluator confirms final package placement.
The earlier `rgbd_pickup_v1` is an interrupted diagnostic run exposing self geometry;
it is not a successful evaluation.

ROS2 transport verification also passed the same three-view mission in 80.6 s:
`../results/rgbd_pickup_ros2/report.json`. This second report uses ROS sensor
evidence; independent world-pose checks are provided by the native run above.
