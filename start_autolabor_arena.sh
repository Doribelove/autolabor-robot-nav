#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="/home/robot"
AUTOLABOR_WS="$BASE_DIR/autolabor_ws"
LIVOX_WS="$AUTOLABOR_WS/src/livox/Mid_livox_ros_driver2"
ARENA_WS="$BASE_DIR/arena_ws"
CAN_PORT_CACHE="$BASE_DIR/.ros/autolabor_can_port"
ROS_SETUP="/opt/ros/noetic/setup.bash"
START_DELAY="${START_DELAY:-3}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_file() {
  [[ -f "$1" ]] || die "Missing file: $1"
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

is_ttyusb_0_or_1() {
  [[ "$1" == "/dev/ttyUSB0" || "$1" == "/dev/ttyUSB1" ]]
}

find_can_port() {
  if [[ -n "${CAN_PORT:-}" ]]; then
    [[ -e "$CAN_PORT" ]] || die "CAN_PORT does not exist: $CAN_PORT"
    echo "$CAN_PORT"
    return
  fi

  local devs=()
  local dev
  for dev in /dev/ttyUSB0 /dev/ttyUSB1; do
    [[ -e "$dev" ]] && devs+=("$dev")
  done

  if [[ -d /dev/serial/by-id ]]; then
    local links=()
    local link target
    for link in /dev/serial/by-id/*; do
      [[ -e "$link" ]] || continue
      target="$(readlink -f "$link")"
      is_ttyusb_0_or_1 "$target" && links+=("$link")
    done

    if (( ${#links[@]} == 1 )); then
      echo "${links[0]}"
      return
    fi

    if [[ -f "$CAN_PORT_CACHE" ]]; then
      local cached
      cached="$(<"$CAN_PORT_CACHE")"
      [[ -e "$cached" ]] && echo "$cached" && return
    fi
  fi

  if (( ${#devs[@]} == 1 )); then
    echo "${devs[0]}"
    return
  fi

  if (( ${#devs[@]} == 0 )); then
    die "No /dev/ttyUSB0 or /dev/ttyUSB1 found. Connect the chassis USB-CAN adapter first."
  fi

  die "Found multiple USB serial ports: ${devs[*]}. Run with CAN_PORT=/dev/ttyUSB0 or CAN_PORT=/dev/ttyUSB1."
}

start_terminal() {
  local title="$1"
  local body="$2"

  gnome-terminal --title="$title" -- bash -ic "
$body
code=\$?
echo
echo '$title exited with code '\$code
exec bash
"
}

require_file "$ROS_SETUP"
set +u
source "$ROS_SETUP"
set -u

require_cmd gnome-terminal
require_cmd roslaunch
require_file "$AUTOLABOR_WS/devel/setup.bash"
require_file "$LIVOX_WS/devel/setup.sh"
require_file "$LIVOX_WS/devel/setup.bash"
require_file "$ARENA_WS/devel/setup.bash"

CAN_PORT="$(find_can_port)"
mkdir -p "$(dirname "$CAN_PORT_CACHE")"
printf '%s\n' "$CAN_PORT" > "$CAN_PORT_CACHE"

echo "Using CAN port: $CAN_PORT"
echo "Updating permission with sudo chmod 666 $CAN_PORT"
sudo chmod 666 "$CAN_PORT"

printf -v CAN_PORT_Q '%q' "$CAN_PORT"

start_terminal "01 chassis canbus" \
  "source /opt/ros/noetic/setup.bash &&
source $AUTOLABOR_WS/devel/setup.bash &&
roslaunch autolabor_canbus_driver drive_only.launch port_name:=$CAN_PORT_Q"

sleep "$START_DELAY"

start_terminal "02 livox mid360 msg" \
  "source /opt/ros/noetic/setup.bash &&
source $LIVOX_WS/devel/setup.sh &&
roslaunch livox_ros_driver2 msg_MID360.launch"

sleep "$START_DELAY"

start_terminal "03 fast lio mapping" \
  "source /opt/ros/noetic/setup.bash &&
source $LIVOX_WS/devel/setup.bash &&
roslaunch fast_lio mapping_mid360.launch"

sleep "$START_DELAY"

start_terminal "04 livox scan" \
  "workon rosnav &&
source /opt/ros/noetic/setup.bash &&
source $LIVOX_WS/devel/setup.sh &&
roslaunch livox_ros_driver2 scan.launch"

sleep "$START_DELAY"

start_terminal "05 arena nav" \
  "workon rosnav &&
cd $ARENA_WS &&
source /opt/ros/noetic/setup.bash &&
source devel/setup.bash &&
roslaunch arena_bringup real_nav_nomap.launch"

echo "All launch commands have been opened."
