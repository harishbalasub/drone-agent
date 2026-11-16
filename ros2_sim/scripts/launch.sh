#!/usr/bin/env bash
set -euo pipefail
sim_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scenario="${1:-1}"
if [[ "$scenario" != 1 && "$scenario" != 2 && "$scenario" != 3 ]]; then echo 'Usage: launch.sh [1|2|3] [--headless|--render-headless]'; exit 2; fi
if [[ -n "${2:-}" && "${2:-}" != --headless && "${2:-}" != --render-headless ]]; then echo 'Unknown rendering option'; exit 2; fi
if [[ -n "${3:-}" && "${3:-}" != --rgbd ]]; then echo 'Unknown perception option; use --rgbd'; exit 2; fi
if [[ "${3:-}" == --rgbd ]]; then
  [[ -x "$sim_root/.runtime/rgbd_venv/bin/python" && -f "$sim_root/results/rgbd_spark_cuda_v1/best.pt" ]] || { echo 'RGB-D runtime/checkpoint missing; see rgbd/README.md'; exit 1; }
  if [[ "${2:-}" == --headless ]]; then echo 'RGB-D requires rendered cameras; use --render-headless'; exit 2; fi
  export DRONE_PACKAGE_PERCEPTION=rgbd
fi
export DRONE_SCENARIO="$scenario"
suffix=''; gz_args=(-r)
if [[ "${2:-}" == --headless ]]; then suffix=_headless; gz_args=(-s -r); fi
if [[ "${2:-}" == --render-headless ]]; then gz_args=(-s -r --headless-rendering); fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then echo 'Install ROS 2 Humble first; see ros2_sim/README.md'; exit 1; fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ -f "$sim_root/.runtime/env.sh" ]]; then source "$sim_root/.runtime/env.sh"; fi
command -v ros2 >/dev/null || { echo 'ROS 2 CLI is missing; see README dependencies.'; exit 1; }
ros2 pkg prefix ros_gz_bridge >/dev/null
/usr/bin/python3 -c 'import rclpy; import ros_gz_interfaces.msg'
# Each invocation has an isolated DDS and Gazebo domain; all child processes inherit it.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${GZ_PARTITION:-drone_agent}"
children=()
cleanup() { trap - EXIT INT TERM; for pid in "${children[@]}"; do kill -- "-$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT INT TERM
setsid gz sim "${gz_args[@]}" "$sim_root/worlds/scenario_${scenario}${suffix}.sdf" & children+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=$sim_root/config/bridge.yaml" & children+=("$!")
setsid /usr/bin/python3 "$sim_root/semantic_bridge.py" & children+=("$!")
if [[ "${DRONE_PACKAGE_PERCEPTION:-}" == rgbd ]]; then
  setsid /usr/bin/python3 "$sim_root/rgbd/live_bridge.py" & children+=("$!")
fi
setsid /usr/bin/python3 "$sim_root/server.py" --transport ros2 & children+=("$!")
wait -n
