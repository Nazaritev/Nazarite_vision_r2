#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/stm32_serial_launcher}"
SUDO_PASSWORD="${SUDO_PASSWORD:-1}"
mkdir -p "${LOG_DIR}"

PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck disable=SC1090
    source "${setup_file}"
  fi
}

start_cmd() {
  local name="$1"
  shift
  echo "[serial-launcher] starting ${name}: $*"
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  sleep 1
}

sudo_cmd() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' "$@"
  fi
}

setup_can0() {
  echo "[serial-launcher] configuring can0 before arm_moveit demo"
  sudo_cmd ip link set can0 down 2>/dev/null || true
  sudo_cmd ip link set can0 type can bitrate 1000000
  sudo_cmd ip link set can0 up
  ip link show can0

  if command -v candump >/dev/null 2>&1; then
    echo "[serial-launcher] starting candump can0"
    candump can0 >"${LOG_DIR}/candump_can0.log" 2>&1 &
    PIDS+=("$!")
    sleep 1
  else
    echo "[serial-launcher] candump not found, skip can0 dump"
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/dgut/Nazarite/install/setup.bash
source_if_exists /home/dgut/arm_control/install/setup.bash
source_if_exists /home/dgut/r2_send_ws/install/setup.bash

start_cmd livox_mid360 ros2 launch livox_ros_driver2 msg_MID360_launch.py
setup_can0
start_cmd arm_moveit_demo ros2 launch arm_moveit_config_grand_new23 demo.launch.py
start_cmd rc_mapping ros2 launch rc_bringup mapping.launch.py
start_cmd r1_mode1_challenge_red_test ros2 run stm32_driver r1_mode1_challenge_red_test
start_cmd r2_global_matrix_node_challenge ros2 run r2_planner r2_global_matrix_node_challenge

echo "[serial-launcher] all processes started. logs: ${LOG_DIR}"
wait
