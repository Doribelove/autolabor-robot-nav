#!/usr/bin/env bash
set -euo pipefail
bev_test_ws=$(cd -- "$(dirname -- "$0")/.." && pwd)
mkdir -p "$bev_test_ws/runtime/bev_cleaning_test/tmp" "$bev_test_ws/validation/bev_cleaning_20261027"
exec bwrap --die-with-parent --unshare-net --unshare-pid --ro-bind / / \
  --bind "$bev_test_ws" "$bev_test_ws" \
  --bind "$bev_test_ws/runtime/bev_cleaning_test/tmp" /tmp \
  --dev /dev --proc /proc --chdir "$bev_test_ws" \
  --setenv ROBOT_OPTIMIZED_SANDBOX 1 bash -c '
    set -e
    source scripts/setup_env.sh
    export ROS_PACKAGE_PATH="$PWD/src:/home/slam/robot_j6m_ws_navigation_20260907/src:$ROS_PACKAGE_PATH"
    export PYTHONPATH="$PWD/src/application/autolabor_fod_control/src:$PWD/devel/lib/python3/dist-packages:/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/python3/dist-packages:$PYTHONPATH"
    export ROS_MASTER_URI=http://127.0.0.1:11589 ROS_IP=127.0.0.1
    export ROS_HOME="$PWD/runtime/bev_cleaning_test/ros-home" ROS_LOG_DIR="$PWD/validation/bev_cleaning_20261027/ros_logs"
    unset ROS_HOSTNAME
    python3 -s src/application/autolabor_fod_control/test/test_bev_cleaning.py -v
    rostest --results-base-dir="$PWD/validation/bev_cleaning_20261027" autolabor_fod_control bev_cleaning.test
  '
