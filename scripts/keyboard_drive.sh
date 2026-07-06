#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CAN_PORT="${CAN_PORT:-/dev/ttyUSB0}"
PUBLISH_TF="${PUBLISH_TF:-true}"

PIDS=()

cleanup() {
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
}

trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

require_file() {
  [[ -f "$1" ]] || {
    echo "Missing file: $1" >&2
    exit 2
  }
}

wait_ros_master() {
  local timeout="${1:-15}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done
  echo "Timed out waiting for ROS master" >&2
  return 1
}

start_ros_master() {
  if rostopic list >/dev/null 2>&1; then
    return 0
  fi

  echo "==> starting ROS master"
  roscore >/tmp/robot_ws_keyboard_roscore.log 2>&1 &
  PIDS+=("$!")
  wait_ros_master 15
}

make_writable() {
  local dev="$1"
  [[ -e "$dev" ]] || {
    echo "Device does not exist: $dev" >&2
    exit 3
  }
  if [[ ! -w "$dev" ]]; then
    echo "==> sudo chmod 666 $dev"
    sudo chmod 666 "$dev"
  fi
}

node_running() {
  local node="$1"
  rosnode list 2>/dev/null | grep -qx "$node"
}

wait_topic() {
  local topic="$1"
  local timeout="${2:-20.0}"
  rosrun robot_diagnostics wait_for_topics.py _topics:="$topic" _timeout:="$timeout"
}

wait_cmd_subscriber() {
  local timeout="${1:-20}"
  local deadline=$((SECONDS + timeout))
  while (( SECONDS < deadline )); do
    if rostopic info /cmd_vel 2>/dev/null | grep -q '/m2_driver'; then
      return 0
    fi
    sleep 0.2
  done
  echo "Timed out waiting for /m2_driver to subscribe to /cmd_vel" >&2
  return 1
}

require_file "$ROS_SETUP"
require_file "$ROBOT_WS/devel/setup.bash"

source "$ROS_SETUP"
source "$ROBOT_WS/devel/setup.bash"

start_ros_master
make_writable "$CAN_PORT"

if node_running /canbus_driver && node_running /m2_driver; then
  echo "==> CAN chassis driver already running"
else
  echo "==> starting CAN chassis driver on $CAN_PORT"
  roslaunch robot_bringup can.launch port_name:="$CAN_PORT" publish_tf:="$PUBLISH_TF" &
  PIDS+=("$!")
fi

wait_topic "/canbus_msg" 30.0
wait_cmd_subscriber 30

echo "==> starting keyboard teleop"
echo "    Keep this terminal focused. Press Ctrl-C to quit."
rosrun robot_bringup autolabor_keyboard_teleop.py
