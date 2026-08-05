#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS_SETUP="${WS_SETUP:-$ROBOT_WS/devel/setup.bash}"
PRIVATE_SETUP="${PRIVATE_SETUP:-$ROBOT_WS/.deps/setup.bash}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup: $ROS_SETUP" >&2
  exit 2
fi
if [[ ! -f "$WS_SETUP" ]]; then
  echo "Missing workspace setup: $WS_SETUP" >&2
  echo "Build the workspace before starting the operator GUI." >&2
  exit 2
fi

source "$ROS_SETUP"
source "$WS_SETUP"
if [[ -f "$PRIVATE_SETUP" ]]; then
  source "$PRIVATE_SETUP"
fi

# roslaunch starts a ROS master when none is running.  The GUI itself treats
# every robot-side node as optional and remains usable as a status dashboard.
exec roslaunch autolabor_operator_gui operator_gui.launch "$@"
