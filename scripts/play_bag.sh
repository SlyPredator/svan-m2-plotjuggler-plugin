#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${1:-/home/robotics/navneeth/pj-plugin/m2-pj/xtr_rosbags/testbag4}"
RATE="${2:-1.0}"

if [[ ! -d "${BAG_PATH}" && ! -f "${BAG_PATH}" ]]; then
  echo "Error: Bag not found at ${BAG_PATH}"
  exit 1
fi

echo "=================================================="
echo " Playing Svan M2 Bag over CycloneDDS"
echo " Bag:  ${BAG_PATH}"
echo " Rate: ${RATE}x"
echo "=================================================="

# Source ROS 2 and M2 message overlay (temporarily disable nounset for ROS scripts)
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "/home/robotics/m2_ws/install/setup.bash" ]]; then
  source /home/robotics/m2_ws/install/setup.bash
fi
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 bag play "${BAG_PATH}" --rate "${RATE}" --disable-keyboard-controls --topics /m2_metal/hw/sensor_data /m2_metal/hw/joint_command /joystick_data /m2_metal/hw/odom "${@:3}"
