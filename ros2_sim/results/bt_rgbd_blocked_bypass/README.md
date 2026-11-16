# Behavior tree: blocked-gate delivery with learned RGB-D

Scenario 2 passed: three inspections, RGB-D-aligned pickup, one authorized gate
bypass, package placed on blue platform, then return home. No traversal action
was executed; only the gate goal was waived. No replan interrupt was needed for
this predeclared alternate. No collisions; independent world-pose evaluator
verified package placement and home. Duration: 77.44 seconds.

`session.json` contains the complete behavior tree in versions[0].robot_plan,
BT_NODE transitions, BT_ALTERNATE_SELECTED, waiver IDs and physical action evidence.
`report.json` records MISSION_COMPLETED_WITH_AUTHORIZED_BYPASS and passed=true.
Other scene sensing (including gate blockage) remains ideal; package sensing is
learned RGB-D. Unit tests separately cover clear gates, unknown state, skill failure,
unauthorized branches, and exact user wording cross-compiled in JS and Python.
