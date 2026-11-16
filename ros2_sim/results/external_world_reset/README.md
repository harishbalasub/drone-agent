# External world reset verification

The reported package-on-platform startup came from an old LAN gateway retaining a completed November 11 mission. A separate fresh localhost gateway had been running Scene 1. The old completed session was preserved under `session_logs/before_scene_reset_fix` before retiring those idle stacks.

A fresh Scene 1 with ROS2, rendered cameras and learned RGB-D was launched on the existing LAN address, isolated in ROS domain 74 / Gazebo partition `drone_lan_v3`.

Real `/api/sim/reset` verification observed:

- Drone: `[0.85, 0.65, 0.430027899880128]` (expected `[0.85, 0.65, 0.43]`).
- Red package: `[7.1, 1.3, 0.12]`, at pickup rather than blue platform `[7.6, 4.15, 0.5]`.
- Reset evidence source: `gazebo_reset_pose_verification`.
- Reset did not mark package localized/found for the mission.

The actual UI Reset World button was exercised in External Gazebo / ROS2 mode. It completed, returned to IDLE and remained enabled. The screenshot shows the empty blue platform, package at pickup, drone at start and enabled reset button. All three cameras recovered. Learned RGB-D observations continued after simulation time restarted (observed message age 0.15 seconds); `valid=false` at home is expected because the package is outside the downward view.

The gateway verifies real poses after reset, not merely successful HTTP delivery or reset-service acknowledgement. Newly created transport nodes retry bounded service discovery. Camera evidence is not used as a substitute for pose verification. Tests also cover original-plan retention, active-mission rejection and blocking execution after a failed reset.
