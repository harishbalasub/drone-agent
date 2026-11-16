#!/usr/bin/env bash
set -u
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
status=0
check() { if "$@"; then return; else status=1; fi; }
printf 'Architecture: '; uname -m
printf 'Gazebo Sim: '; check gz sim --versions
if [[ -f /opt/ros/humble/setup.bash ]]; then set +u; source /opt/ros/humble/setup.bash; set -u; fi
if [[ -f "$sim_root/.runtime/env.sh" ]]; then source "$sim_root/.runtime/env.sh"; fi
printf 'ROS CLI: '; check command -v ros2
check /usr/bin/python3 -c 'import rclpy; from sensor_msgs.msg import Image; from nav_msgs.msg import Odometry; from ros_gz_interfaces.msg import Contacts; from gz.transport13 import Node; print("ROS messages and Gazebo transport: OK")'
if command -v ros2 >/dev/null; then check ros2 pkg prefix ros_gz_bridge; fi
exit "$status"
