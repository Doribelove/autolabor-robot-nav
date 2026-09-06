#!/usr/bin/env bash
# Optional companion to an ALREADY running candidate navigation stack.
# Starts neither camera nor ROS master. No arguments = continuous with navigation.
# Leaving both overview/BPU pages or minimizing Qt removes image demand.
set -euo pipefail
candidate=/home/slam/robot_j6m_ws_optimized_20260905
if [[ "${ROBOT_OPTIMIZED_SANDBOX:-}" != 1 ]]; then
  exec "$candidate/scripts/optimized.sh" run bash "$0" "$@"
fi
source "$candidate/scripts/setup_env.sh"
if [[ "${1:-}" != --stop ]] && ! "$candidate/scripts/verify_master_owner.sh"; then
  echo 'BPU bridge not started: an already-running, safely verified candidate navigation master is required.' >&2
  exit 3
fi
export ROS_MASTER_URI=http://192.168.10.100:11311 ROS_IP=192.168.10.50
unset ROS_HOSTNAME
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if (( $# == 0 )); then
  set -- --continuous
fi
exec nice -n 10 /home/slam/robot_ws/.venv/fod_yolo/bin/python3 \
  "$candidate/experiments/j6m_bpu_20260905/tools/qt_bpu_bridge.py" --display-with-navigation "$@"
