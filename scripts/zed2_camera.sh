#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
# The same-host ARM wrapper remains installed in the original workspace.
# Its environment is confined to this camera process.
ZED_DRIVER_SETUP="${ZED_DRIVER_SETUP:-$HOME/robot_ws/.deps/setup.bash}"
source "$ZED_DRIVER_SETUP"
exec roslaunch "$ROBOT_WS/src/scripts/robot_bringup/launch/zed2_camera.launch" "$@"
