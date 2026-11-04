#!/usr/bin/env bash
# Optional Qt window only. Opening/closing it does not arm recovery or stop navigation.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
exec rosrun autolabor_operator_gui autolabor_operator_gui_node \
  __name:=camera_direct_recovery_ui _enable_rviz:=false _camera_driver:=zed \
  --camera-recovery-panel "$@"
