ROSOrin red-cube experiment

These are experimental ROS 2 scripts for the specific robot at 192.168.1.177,
not a finished general robot API or persistent executor. No LLM runs in them.
Run in the robot's ROS environment. Source files live here; test copies were
transferred to /tmp on the robot.

observe_red_cube.py: read-only HSV red segmentation, matched RGB/depth,
calibration/TF reporting and forward lidar measurement. Color/shape filtering
finds red candidates; it is not a trained semantic cube detector.
approach_red_cube.py: dry-run by default; --execute enables a short straight
approach. Target loss, alignment, sensor freshness, odometry, front clearance,
travel and elapsed-time bounds guard movement. Stale frames cause zero velocity.
plan_red_grasp.py: IK planning by default; --execute performs open/approach/
descend/close/lift. IK feasibility is not collision checking or grasp verification.

Hardware-specific assumptions: Aurora align_mode=true (checked live), mono16
millimeter depth, ROSOrin lidar mounting yaw=pi (checked using TF), vendor hand-
to-camera matrix, gripper pulses 150=open and 560=close. Recalibrate before
reusing on other hardware. These scripts are not an emergency-stop system.

Observed trial: red target initially ~0.47m ahead of base. Approach completed
with ~0.152m odometry travel on the final run, after earlier short attempts
stopped on sensor checks. Pickup motion ran, but the post-grasp camera appeared
to show the cube on the floor. Physical retention is NOT confirmed. Do not
advertise this experiment as a successful autonomous pick.

Next work: verified grasp calibration/feedback, persistent sensor executor,
command arbitration, cancellation, and integration with the desktop chatbot.

Follow-up: user observed the fingers closing just above the cube. Added an
explicit --grasp-height-offset-m option, bounded to [-0.015, 0] metres and
defaulting to zero. A proposed -0.01 m correction has NOT been executed or
validated. After moving the empty arm to init and then camera_up, the cube
was absent from the camera view; planning rejected the missing target.
Base was not commanded to move during this follow-up. Reacquire the target
and check calibration before attempting another grasp.

Hardware diagnosis: /controller_manager/servo_states reports cached COMMANDS,
not measured positions. Kinematics using these states can be wrong when a
joint fails to move. Before reboot, actual shoulder position stayed at 413
despite a 630 target; do not assume a height offset fixes the missed grasp.
The vendor get_state voltage query crashes the hardware driver (missing
bus_servo_read_voltage); torque query also references a missing method.
Use get_position only. The driver was recovered temporarily, then the user
rebooted the robot. After reboot, measured IDs 1,2,3,4,5,10 were
501,723,44,150,500,502, close to saved init 500,725,50,150,500,500.
External Logitech camera /dev/video2 confirmed a raised forward/downward
gripper pose. No motion was commanded during the post-reboot check.
The alternate grasp-controller home (630,151,70 for joints 2-4) should not
replace the saved init simply because earlier commands failed physically.

Post-reboot retry: added get_position-only checks before execution and after
each movement (20 pulse tolerance; closure exempt because an object may stop
the fingers). Zero height offset used. Above and descent passed: descent
targets [490,118,427,333,504], measured [492,115,426,331,504]. Lift aborted:
shoulder target 234, actual 202. External camera confirmed cube still on
floor, gripper empty. No further motion sent. Pickup remains unsuccessful;
target calibration and shoulder tracking require investigation. The check
aborts subsequent commands; it is not an active servo stop or stall watchdog.

Requested follow-up: 2cm farther forward, slightly lower. Added bounded
--forward-offset-m [0,.02]; proposed arm-target adjustment is +.02m X and
-.01m grasp Z, with base stationary. NOT executed: subsequent actual shoulder
reading remained 202 versus last target 234. Other actual joints 1,3,4,5:
492,443,203,504. Resolve tracking discrepancy before applying this adjustment.

After calibration-tool closure, normal bringup was restarted; actual init
positions 2-4 settled at 722,49,148, and external camera confirmed init.
Latest requested retry: dry plans with +.02m forward and -.01m grasp height
returned NO grasp IK solution, at both preferred pitches 80 and 45 degrees.
Target arm XYZ was [.28156,-.01124,.00651], grasp Z -.00349.
No base or arm motion was sent for this retry. Added bounded --pitch-deg
30..80 planning option and slowed future arm movements to 5 seconds.
Do not execute an empty grasp solution or relax joint bounds to force it.

Latest execution: --step-2cm approach completed at odometry travel .020125m
(3cm maximum travel, .02m/s, sensor guards). Fresh adjusted arm target
[.270814,-.010243,.006258], grasp Z -.003742, became reachable.
Pickup used +.02m X / -.01m Z and five-second arm movements. Above and
descent checks passed (descent actual IDs1..5 [493,97,387,416,504]). Lift
failed again: shoulder target228, actual137. Desktop webcam confirmed empty
gripper and cube on floor. No further attempts or home commands sent.
Arm remains extended near cube; base stopped. Investigate recurring shoulder
tracking failure before more physical retries; grasp calibration also unresolved.
