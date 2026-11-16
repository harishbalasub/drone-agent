# Drone Agent v3: ROS 2 / Gazebo execution

This is a separate extension of the HTML mission app. `accountable_drone_agent_prototype_v2.html`, `prototype_server.py`, and the original domain remain unchanged.

## What runs

- **New app:** `accountable_drone_agent_ros2_v3.html`; choose **Browser simulation** or **External Gazebo / ROS 2**.
- **Gazebo Harmonic (Sim 8)**: self-contained quadrotor, rotor thrust, gravity, collision geometry, 60 g movable red package, attachment gripper, gate, obstacle, blue delivery platform, home marker. No Fuel asset download.
- **Scenario 1:** clear gate. **Scenario 2:** an obstruction fills the gate.
- **ROS 2 Humble**: odometry, IMU, contact, RGB/depth camera streams and gripper commands/state through `ros_gz_bridge`; ideal semantic-camera observations through `semantic_bridge.py`.
- **Executor:** search, distinct inspections, navigation, aligned attachment, test lift, gate survey/crossing, settled delivery, return home. Position feedback drives the Gazebo multicopter velocity controller.
- **Recovery:** structured interrupt, explicit mission revision, abort/return by default; user-authorized bypass; optional human decision with a 30-second timeout.
- **Evidence:** an action returning does not alone establish success. Grasp needs attachment + observed lift; delivery needs detachment + settled platform observation; home needs measured position. Holding is an intermediate milestone, so successful delivery does not require still holding the package.

This is an **AirVLA-inspired embodiment**, not an exact Starling model or an AirVLA reproduction. Mass, inertia, dimensions, sensors and gains are engineering approximations. The initial gripper is a fixed-joint attachment with alignment checks; finger articulation/friction/slip are not modeled. PX4 SITL is **not integrated in v3**: Gazebo's native multicopter controller is the flight backend. The app's PDDL export remains an audit artifact, not the source of a solver-produced trajectory.

## Ubuntu 22.04 installation

Target: Ubuntu 22.04 x86-64, ROS 2 Humble, **Gazebo Harmonic**. Do not substitute Gazebo Classic's `gazebo` executable or Humble's default Fortress bridge. A GTX 1080 can be used for Gazebo graphics; actual rendered performance depends on drivers and scene load. DGX Spark/ARM64 deployment has not been tested by this project.

Use the official ROS and Gazebo apt repositories:

- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- https://gazebosim.org/docs/harmonic/install_ubuntu/
- https://gazebosim.org/docs/harmonic/ros_installation/

After configuring those repositories, install:

```bash
sudo apt update
sudo apt install ros-humble-ros-base gz-harmonic ros-humble-ros-gzharmonic \
  python3-gz-transport13 python3-gz-msgs10 \
  ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
```

The `ros-humble-ros-gzharmonic` packages conflict with the default `ros-humble-ros-gz*` Fortress packages. Review the package manager's proposed changes, particularly if another project uses Fortress. The `launch.sh` workflow does not install packages or modify your OS.

Alternatively, on this project's Ubuntu machine, install the missing ROS dependencies into an **isolated project-local overlay** without sudo or changing system packages:

```bash
/usr/bin/python3 ros2_sim/scripts/setup_local_runtime.py
```

This helper requires the official Gazebo apt source to already be configured. It downloads signed ROS/OSRF packages, verifies repository signatures through apt, and extracts them under ignored `ros2_sim/.runtime/`. The launcher automatically sources its environment. Rendering shader resources are also included. The helper is tested for Ubuntu 22.04 x86-64, not Spark.

Check dependencies from the project root:

```bash
bash ros2_sim/scripts/doctor.sh
```

Use `/usr/bin/python3` (Ubuntu's Python 3.10), not a Conda or bundled app Python. If apt reports an expired ROS signing key or 404 URLs, repair the official ROS repository configuration and refresh its index before installation. Do not disable signature verification.

## Launch and use the app

From the project root:

```bash
# Scenario 1, 3D GUI and rendered sensors
bash ros2_sim/scripts/launch.sh 1
```

Open **http://127.0.0.1:8100/**. Select **External Gazebo / ROS 2**, check the connection, generate the default mission spec, validate, build its plan and run it. Leave the API key blank for the existing local compiler, or use the existing LLM proxy workflow.

```bash
# Scenario 2, blocked gate
bash ros2_sim/scripts/launch.sh 2

# No GUI or rendered RGB/depth; physics + ideal semantic cameras still operate
bash ros2_sim/scripts/launch.sh 1 --headless

# No GUI, but render RGB/depth using EGL
bash ros2_sim/scripts/launch.sh 1 --render-headless
```

Stop the launcher before changing scenario. Restart Gazebo **and the gateway for every fresh episode**; the package has physically moved and mission evidence must not carry over. Browser page refresh alone does not reset the external world. The HTML scenario selector is disabled for external execution; the launch argument selects the authoritative world. Dragging on the browser reference map cannot move Gazebo objects.

The launcher isolates its Gazebo partition (`drone_agent`) and DDS domain (`42`), starts Gazebo, the ROS bridge, semantic adapter, and gateway, and stops child processes on exit. Keep `ROS_DOMAIN_ID` and `GZ_PARTITION` consistent when launching components separately. The gateway binds to loopback; use SSH forwarding for another machine rather than opening an unauthenticated command port:

```bash
ssh -L 8100:127.0.0.1:8100 your-simulator-host
```

The final home condition is **hovering at home at 1.6 m**, not landing. A feedback hold continues after completion while the gateway runs. `Stop and hold` cancels mission progression. LAND is rejected until a touchdown verifier is commissioned. This software is simulation-only; it has no hardware flight authorization or PX4 failsafe integration.

## Default mission and recovery

> Find the red package, inspect it from two viewpoints, pick it up, fly through the gate, place it on the blue platform, and return home.

1. Take off, search near the package and obtain a fresh semantic observation.
2. Inspect from two measured positions; require both minimum physical and angular separation.
3. Navigate overhead, descend, check grasp alignment, attach, test-lift and observe actual package elevation.
4. Carry the package to the gate approach, turn to survey the opening and require fresh gate observations. Unknown is not clear.
5. Cross between the posts, verify opposite-side positions, approach the platform, release and observe settling.
6. Return around the obstacle and verify home position.

Scenario 2 produces `GATE_BLOCKED`, then `REPLAN_INTERRUPT`. With no conditional clause, the replacement spec contains only `RECOVERY_HOME`, and the drone returns holding the package. Outcome: `MISSION_ABORTED_SAFE_RETURN_COMPLETED`, **not original mission success**.

Add **“If the gate is blocked, bypass it.”** to authorize a replacement route around the gate, then delivery and home. Outcome: `MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS`; the traversal goal is explicitly waived. Add **“If the gate is blocked, ask me before bypassing.”** to use the human panel. Timeout selects abort/return. Only gate blockage permits bypass; a sensor failure is not treated as permission to waive traversal. One recovery is attempted; further failure halts progression and records incomplete status.

## Sensors and ground truth

The runnable baseline uses **Gazebo logical cameras**, which directly report identified objects in their viewing frustum. These are ideal semantic/oracle sensors, not RGB object detection, not occlusion-robust perception, and not an AirVLA model. `semantic_bridge.py` transforms camera-relative observations into `map` coordinates, then publishes a versioned JSON payload via ROS. This adapter is needed because the Humble Harmonic binary bridge does not provide the newer `LogicalCameraImage` ROS mapping.

The drone pose is Gazebo odometry used as **simulated motion capture**, analogous to an external pose source. It is not visual-inertial odometry or SLAM. World-pose truth is used separately by the native evaluator to check final package/home positions, never as the executor's object lookup.

Rendered worlds also publish three 256×256 RGB cameras at 5 Hz and a downward float32 depth image. Depth is an extension to the AirVLA-inspired RGB setup. The app offers a live external 3D scene view or telemetry map, previews forward/downward images and reports a center-depth statistic. The baseline grasp localizer still uses the ideal semantic sensor; The separate [RGB-D training pipeline](rgbd/README.md) now supports rendered dataset collection and supervised package segmentation with calibrated depth localization; acceptance and integration into mission execution are separate steps. Headless non-rendering worlds intentionally produce no RGB/depth frames.

## Interfaces

Browser → gateway (same origin, port 8100):

| Method/path | Content |
|---|---|
| `GET /api/sim/state` | connection health, pose/sensor freshness, facts, current run/session |
| `POST /api/sim/run` | `mission_spec`, `robot_plan`, optional original `pddl` and `validation` |
| `POST /api/sim/stop` | `{}`; cancel progression and hold |
| `POST /api/sim/decision` | `{"action":"BYPASS_GATE"}` or `ABORT_AND_RETURN_HOME` |
| `POST /api/sim/save` | `{"directory":"session_logs/ros2"}` |
| `GET /api/sim/camera/front` | latest received RGB frame as PNG; 404 if absent |

ROS topics:

| Topic | Type / role |
|---|---|
| `/drone/cmd_vel` | `geometry_msgs/Twist`, body-frame velocity/yaw-rate command |
| `/drone/odometry` | `nav_msgs/Odometry`, map-frame pose, simulated motion capture |
| `/drone/imu` | `sensor_msgs/Imu` |
| `/drone/contacts` | `ros_gz_interfaces/Contacts`, body contacts |
| `/drone/camera/{front,down,external}/image` | `sensor_msgs/Image`, RGB |
| `/drone/camera/down/depth_image` | `sensor_msgs/Image`, `32FC1` meters |
| `/drone/gripper/{attach,detach}` | `std_msgs/Empty` commands |
| `/drone/gripper/state` | `std_msgs/String`, attached/detached acknowledgement |
| `/drone/perception/{front,down}` | `std_msgs/String`, JSON semantic observations in map frame |
| `/drone/mission/status` | `std_msgs/String`, JSON observations, action results, revisions and mission outcome |
| `/drone/mission/interrupt` | `std_msgs/String`, JSON failure + observed state + completed goals |
| `/clock` | ROS simulated clock |

Commands and results are coordinated by the gateway; this version does not define custom ROS action messages. Event timestamps are UTC; local monotonic receipt times enforce freshness. JSON events have `schema_version`, `message_type`, `run_id`, `mission_revision`, monotonic `sequence` per session, `time` and event-specific payload. An `ACTION_RESULT` contains the skill, result, verified facts and received sensor snapshot. No API keys are included in simulator requests or logs.

Save external logs with either save button while external mode is selected. The existing safe export writes the original spec/validation/PDDL/plan and each recovery revision to a project subdirectory, with interrupts, human decisions, trace and metrics. Recovery PDDL is explicitly marked not solver-generated; the revised spec and robot plan are authoritative for this controller.

## Training and evaluation

**Training here means waypoint-controller calibration, not neural VLA training.** `train.py` searches outer-loop gains 0.5, 0.8 and 1.1 using real Gazebo physics on short near-home takeoff/navigation trajectories. It selects the lowest score (elapsed time + weighted terminal position errors). Full package delivery and blocked-gate missions are held out. Payload vertical feedforward is a configured engineering value, not learned by this search.

```bash
/usr/bin/python3 ros2_sim/train.py --output ros2_sim/results/my_calibration
/usr/bin/python3 ros2_sim/evaluate.py --scenario 1 \
  --config ros2_sim/results/my_calibration/trained_config.json \
  --output ros2_sim/results/my_clear_gate
/usr/bin/python3 ros2_sim/evaluate.py --scenario 2 \
  --config ros2_sim/results/my_calibration/trained_config.json \
  --output ros2_sim/results/my_blocked_gate
/usr/bin/python3 ros2_sim/evaluate.py --scenario 2 --policy bypass --views 3 \
  --config ros2_sim/results/my_calibration/trained_config.json \
  --output ros2_sim/results/my_bypass
```

If using the local overlay, source `ros2_sim/.runtime/env.sh` before the training/evaluation commands. These default commands launch isolated **native Gazebo transport** episodes, useful for physics/controller testing without DDS. They do not pretend to test ROS. To evaluate ROS, launch a fresh simulator through `launch.sh`, keep the same ROS/Gazebo domains, and run:

```bash
source /opt/ros/humble/setup.bash
# If using the local overlay:
# source ros2_sim/.runtime/env.sh
export ROS_DOMAIN_ID=42 GZ_PARTITION=drone_agent
/usr/bin/python3 ros2_sim/evaluate.py --transport ros2 --scenario 1 \
  --output ros2_sim/results/my_ros_episode
```

Do not start a browser mission concurrently with an evaluation: one executor must own velocity commands. ROS evaluation connects to an already running world; pass the actual scenario selected at launch. Each run stores `session.json`, `report.json` and, for native episodes, a Gazebo log. Failed runs exit nonzero. The independent native evaluator checks final world poses; the ROS report checks executor evidence and clearly identifies that scope. Results are individual deterministic-scene checks, not statistical success rates or robustness guarantees.

For later neural training, record synchronized camera images, pose, gripper state and action commands into ROS bags (install `ros-humble-rosbag2`), collect varied successful demonstrations, and split held-out worlds/object poses before training. Existing event logs alone are not an image-action dataset sufficient to reproduce AirVLA. No pretrained VLA checkpoint is bundled or trained in this release.

## Tests

```bash
/usr/bin/python3 -m unittest discover -s ros2_sim/tests -v
/usr/bin/python3 -m unittest discover -s tests -p 'test_server.py' -v
/usr/bin/python3 tests/test_viewpoints.py
/usr/bin/python3 tests/test_scenarios.py
```

The JS regression tests require `gjs`. Unit/HTTP/UI tests use explicitly labeled stubs; the reports in `results/` come from actual Gazebo/ROS runs. `generate_worlds.py` regenerates all four checked-in SDF files. See `results/SUMMARY.md` for the measured verification record.

## Learned RGB-D package pickup

Run `bash ros2_sim/scripts/launch.sh 1 --render-headless --rgbd` to use the
Spark-trained RGB segmenter plus depth geometry for package search/inspection,
pickup alignment and placement verification. The gateway explicitly blocks ideal
package observations in this mode; gate/obstacle sensing remains ideal. See
[runtime details and assumptions](rgbd/README.md#use-the-learned-package-estimate-for-pickup).
The native three-view mission passed independent physical placement/home checks in
[the evaluation report](results/rgbd_pickup_v2/report.json).

## Runtime panes and end-of-mission behavior

The Runtime / Skill Contract pane follows gateway ACTION_STARTED/ACTION_RESULT
events and current telemetry (pose freshness, geofence, recorded collisions,
package estimate freshness and attachment feedback). Unknown checks are not green
passes. The battery display reads **Unavailable** in ROS2 mode: these worlds have
no battery plugin or sensor, so the browser's 92% slider is not flight telemetry.
Battery reserve is not verified by the ROS2 controller.

RETURN_HOME flies to `(0.85, 0.65, 1.6)` m and holds there. It does not land.
LAND and landed goals are rejected until touchdown sensing/control is commissioned.
Restart an already-running gateway and refresh the browser after installing changes.

## Behavior-tree robot plans and gate alternates

The final Robot Plan tab and saved session versions now contain JSON
`behavior_tree_v1` trees (Sequence, Fallback, Condition, Action). The PDDL plan
remains the nominal symbolic route; the tree adds declared conditional execution.
Browser and ROS2/native executors run memory sequences: completed actions are not
repeated on each telemetry poll. Legacy flat requests remain accepted and are
compiled into trees by the gateway.

Example mission suffix: **If gate is not clear, goto blue platform directly.**
The compiler also recognizes “go to the blue platform directly,” “bypass,” and
“skip the gate.” It stores a USER-authorized BYPASS_GATE contingency. At runtime:

- Fresh gate-clear evidence: traverse the gate.
- Fresh blocked evidence plus explicit authorization: waive only the traversal
  goal, route around the blocker to the platform, place the package, return home.
- Unknown gate state, a collision or another skill exception: normal safe recovery;
  these are not interpreted as permission to skip the gate.
- No authorized alternate: abort and return home, or ask the human if specified.

The gateway checks every submitted branch against the validated nominal actions
and mission contingencies; arbitrary client-supplied alternate skills are rejected.
Session logs include BT_NODE status transitions, BT_ALTERNATE_SELECTED with waived
goal IDs, action evidence and the behavior-tree artifact. Authorized bypass reports
MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS, never an accomplished traversal.

See [complete request example](config/mission_gate_alternate_bt.json). Restart the
launcher and refresh/recompile the mission to load this format in an existing app.
This implementation commissions the gate-blockage alternate; it is not a general
purpose behavior-tree editor or an interface to BehaviorTree.CPP.

Browser gate approach is a safe staging point before the aperture, so NAVIGATE
(gate_loc) does not require gate-clear evidence. Only TRAVERSE_GATE requires it.
This lets the behavior-tree fallback evaluate blockage before attempting traversal.
The preauthorized browser bypass validates its route/energy before committing the
goal waiver, then delivers before returning home. Rejected recovery candidates
restore the previous mission and plan. Browser regression tests execute the full
blocked-gate mission and verify release, delivery-before-return, and no replan.

Gate conditional compilation now accepts “skip it,” “skip the gate traversal,”
and “go [directly] to the blue platform” without requiring a trailing “directly.”
Unrecognized gate condition/action clauses produce NEEDS_CLARIFICATION instead of
silently selecting abort. The mission pane and trace display the compiled gate
policy before execution. Browser build bt-gate-policy-v3 includes this correction.

## Scenario 3: two colored platforms

Select **Scenario 3 — blue and red platforms** in Browser Simulation. The gate is
clear; blue platform remains at (760, 105), red platform is at (760, 255) in the
reference canvas. Missions can specify either color as the delivery destination.

For Gazebo/ROS2, launch `bash ros2_sim/scripts/launch.sh 3 --render-headless`
(add `--rgbd` to opt into learned package perception). Platforms are at
(7.6, 4.15) and (7.6, 2.65) m with equal 0.5 m support height. The new red pad
is a new visual distractor, not part of the original small RGB-D training pilot;
robust learned perception in this scene requires separate evaluation.
The blue browser platform and legend now use an actual blue fill in all scenes.

## Clarify validation items

V6 assumptions and V7 unresolved items are listed individually in a **Clarify
mission** panel below Validate Mission Spec. Confirm selected planning assumptions
and revalidate, or supply corrections/answers and regenerate the mission. Confirmed
premises are retained in user_confirmed_assumptions with USER provenance and a
timestamp; they do not modify sensor facts. Unresolved ambiguities cannot be
confirmed away. Every change invalidates the old PDDL/behavior tree, records an
auditable mission snapshot, and requires validation before planning/execution.

### LLM behavior-tree planning (November 13, 2026)

With an API key, mission compilation and behavior-tree generation are separate LLM requests. The compiler receives the full mission including conditional clauses. The planner receives the validated spec, worksheet, available platforms and skill contract; it proposes the node topology and alternate actions. Local checks validate both gate states, inspection coverage, ordering, supported arguments and authorized delivery changes. Browser and gateway execute the accepted tree without replacing it with a local template. PDDL remains a deterministic translation. Runtime fault recovery remains local policy.

Without a key, the explicitly enabled local demo uses scripted compilation and planning. API failures do not silently select demo planning. Refresh the v3 page and regenerate the spec and plan to use the new flow. Restart the gateway for Python executor changes. Tests include mocked provider proposals with a non-template topology, blue delivery with a clear gate and red delivery when blocked; these are not live provider or physical Gazebo validation.

PDDL export is optional. After validating the Mission Spec, select **Generate Behavior Tree** directly. **Generate PDDL (optional)** creates a separate export only when requested. Validation, tree generation, alternate selection and recovery do not invoke the PDDL compiler. An export failure does not invalidate the Behavior Tree.

### Mission conversation

The v3 page now includes a conversation alongside the scene and technical panels:

- Review assumptions introduced by the compiler, including proposed inspection separation. Accept individual assumptions or edit the angle; edits invalidate the previous tree and revalidate the spec.
- Validation cards show failures and energy/reserve estimates. External battery is explicitly unavailable unless supplied by telemetry; browser estimates do not certify external endurance.
- Submit a new mission or an incremental edit. With a provider key, an LLM maps the turn to a proposed complete mission; **Apply and validate** is required before mutation. Without a key, chat can accept a complete replacement mission through the explicit local demo path.
- Milestone cards report successful search, inspections, pickup, gate traversal, placement, return and final outcome. Camera thumbnails supplement controller evidence; they do not establish success. Browser views are schematic renderings, not ROS camera images.
- For ROS2, the gateway captures the latest preceding forward/down frames at each event, with frame age and estimated UTC capture time. Frames older than 1.5 seconds are labeled stale and omitted. Pillow provides bounded JPEG thumbnails; if unavailable, the card reports that rather than substituting a live image. Restart the gateway to enable capture.
- The chat mirrors existing human bypass/abort decisions and controller timeouts. Arbitrary mission changes during execution remain blocked; dynamic LLM replanning is not introduced by this interface.
- Cards are tied to mission state; stale decisions cannot update a revised mission. Chat and image audit records are included in saved session events. No keys are added to chat context or saved records.

Validation includes mocked structured chat responses, stale decisions, assumption confirmation, angle changes, insufficient battery, completed milestones, world reset/plan reuse, and stale/missing camera evidence. Live LLM-provider and physical Gazebo runs require separate verification.

### Gateway port conflicts

The launcher checks its HTTP address before starting Gazebo or the bridges. If an older gateway owns the address, stop its launcher with Ctrl+C before restarting. The launcher never automatically kills an existing listener. The default address is `127.0.0.1:8100`; `DRONE_BIND_HOST` and `DRONE_PORT` configure the address.

For example, `DRONE_PORT=8101 bash ros2_sim/scripts/launch.sh 1 --render-headless --rgbd` serves the app on port 8101. Parallel simulations also require different `ROS_DOMAIN_ID` and `GZ_PARTITION` values; changing the HTTP port alone does not isolate robot topics. Gateway SIGINT/SIGTERM now closes the HTTP server, and launcher cleanup has a bounded shutdown period for rendering children.
