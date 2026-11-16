# Learned RGB-D pickup: native Gazebo evaluation

Passed Scenario 1 with three distinct inspections, physical attachment + test lift,
gate traversal, release on blue platform and return home, without collision.
Independent world-pose evaluator (never an Executor input) verified placement and
home position. Duration: 80.0 seconds. Package final pose: (7.565, 4.155, 0.620) m.

The package pose consumed by the mission comes exclusively from the Spark-trained
RGB segmenter and timestamp-matched depth/camera pose. Ideal package observations
are discarded by the transport. Gate/other scene sensing remains ideal, and camera
pose uses simulated mocap. Grasp verification uses attachment feedback plus drone
odometry test lift; it does not claim a visual held-package measurement.

See `session.json` for `GRASP_LOCALIZATION`, `GRASP_EVIDENCE`, `PLACEMENT_EVIDENCE`,
all three `INSPECTION_EVIDENCE` events and the complete action/status trace.
The first interrupted run (`../rgbd_pickup_v1`) exposed self-geometry detections.
Near-camera depth exclusion and supported-height checks were added before this run.
These world-specific checks do not establish generalization to arbitrary objects.
