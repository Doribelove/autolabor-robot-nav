#!/usr/bin/env bash
set -euo pipefail
candidate=/home/slam/robot_j6m_ws_optimized_20260905
if [[ "${ROBOT_OPTIMIZED_SANDBOX:-}" != 1 ]]; then
  exec "$candidate/scripts/optimized.sh" run bash "$0" "$@"
fi
source "$candidate/scripts/setup_env.sh"
export ROS_MASTER_URI=http://127.0.0.1:11571 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ "${1:-}" != --stop ]]; then
  "$candidate/scripts/zed_camera_check.sh" --wait 0
fi
exec nice -n 10 python3 "$candidate/experiments/j6m_bpu_20260905/tools/qt_preview_session.py" "$@"
