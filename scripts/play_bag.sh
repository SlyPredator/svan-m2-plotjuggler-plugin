#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

BAG_PATH="${1:-${repo_root}/xtr_rosbags/testbag4}"
RATE="${2:-1.0}"
shift 2 2>/dev/null || shift $# 2>/dev/null || true

if [[ ! -d "${BAG_PATH}" && ! -f "${BAG_PATH}" ]]; then
  echo "Error: Bag not found at ${BAG_PATH}"
  exit 1
fi

player_bin="${repo_root}/build/m2_bag_player"
if [[ -x "${player_bin}" ]]; then
  export LD_LIBRARY_PATH="${repo_root}/.deps/plotjuggler/lib:${repo_root}/third_party/m2_sdk/third_party/install/lib:${repo_root}/build:${LD_LIBRARY_PATH:-}"
  exec "${player_bin}" "${BAG_PATH}" --rate "${RATE}" "$@"
fi

echo "=================================================="
echo " Playing Svan M2 Bag via ROS 2 Fallback"
echo " Bag:  ${BAG_PATH}"
echo " Rate: ${RATE}x"
echo "=================================================="

# Source ROS 2 and M2 message overlay (temporarily disable nounset for ROS scripts)
set +u
source /opt/ros/jazzy/setup.bash
if [[ -n "${M2_WS:-}" && -f "${M2_WS}/install/setup.bash" ]]; then
  source "${M2_WS}/install/setup.bash"
elif [[ -f "${repo_root}/../m2_ws/install/setup.bash" ]]; then
  source "${repo_root}/../m2_ws/install/setup.bash"
fi
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 bag play "${BAG_PATH}" --rate "${RATE}" --disable-keyboard-controls --topics /m2_metal/hw/sensor_data /m2_metal/hw/joint_command /joystick_data /m2_metal/hw/odom "$@"
