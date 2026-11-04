#!/usr/bin/env bash
# Opens the same remote task page used by the complete navigation console.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
exec rosrun autolabor_operator_gui autolabor_operator_gui_node \
  __name:=platform_task_ui _enable_rviz:=false _camera_driver:=zed \
  --platform-task-panel "$@"
