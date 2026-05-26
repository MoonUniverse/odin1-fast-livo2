#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="/tmp/odin_livo_control"
LOG_FILE="${LOG_DIR}/desktop_launcher.log"

mkdir -p "${LOG_DIR}" /tmp/ros-log

{
  echo "==== $(date '+%Y-%m-%d %H:%M:%S') Odin FAST-LIVO2 GUI launcher ===="

  cd "${WORKSPACE}" || {
    echo "Workspace not found: ${WORKSPACE}"
    exit 1
  }

  if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "Missing ROS setup: /opt/ros/humble/setup.bash"
    exit 1
  fi

  if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo "Missing workspace setup: ${WORKSPACE}/install/setup.bash"
    echo "Build odin_livo_control before launching from the desktop."
    exit 1
  fi

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"

  export ODIN_LIVO_WORKSPACE="${WORKSPACE}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-33}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros-log}"

  exec ros2 run odin_livo_control odin_livo_gui
} >> "${LOG_FILE}" 2>&1
