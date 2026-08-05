#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
PRIVATE_SETUP="${PRIVATE_SETUP:-$ROBOT_WS/.deps/setup.bash}"
WORKSPACE_SETUP="$ROBOT_WS/devel/setup.bash"

if [[ -f "$PRIVATE_SETUP" ]]; then
  source "$PRIVATE_SETUP"
elif [[ -f "$WORKSPACE_SETUP" ]]; then
  source "$WORKSPACE_SETUP"
else
  printf 'ROS workspace is not built: %s\n' "$WORKSPACE_SETUP" >&2
  printf 'Run %s/scripts/build_workspace.sh first.\n' "$ROBOT_WS" >&2
  exit 1
fi

exec roslaunch robot_bringup zed2_camera.launch "$@"
