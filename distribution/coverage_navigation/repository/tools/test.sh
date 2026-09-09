#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "$repo/devel/setup.bash"
set -u
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11495 ROS_IP=127.0.0.1
cd "$repo"
catkin_make run_tests -j"${BUILD_JOBS:-2}" -l"${BUILD_JOBS:-2}"
catkin_test_results --all build/test_results
python3 tools/test_summary.py build/test_results
python3 -m unittest discover \
  -s src/platform/autolabor_dual_host/test -p test_move_base_pause_bridge.py -v
