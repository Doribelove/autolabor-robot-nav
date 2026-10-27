#!/usr/bin/env bash
# Build/test this sibling worktree without robot network or writable old trees.
set -euo pipefail
nav_tuning_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
nav_tuning_base="${NAV_TUNING_BASE_WS:-$(dirname "$nav_tuning_ws")/robot_j6m_ws_navigation_20260907}"
[[ -r "$nav_tuning_base/devel/setup.bash" ]] || { echo 'Missing read-only base dependency build' >&2; exit 2; }
case "${1:-build}" in build|test) nav_tuning_mode="${1:-build}";; *) echo 'Usage: validate_navigation_tuning.sh [build|test]' >&2; exit 2;; esac
mkdir -p "$nav_tuning_ws/runtime/isolated/tmp"
exec bwrap --die-with-parent --unshare-net --unshare-pid \
  --ro-bind / / --bind "$nav_tuning_ws" "$nav_tuning_ws" \
  --bind "$nav_tuning_ws/runtime/isolated/tmp" /tmp \
  --dev /dev --proc /proc --chdir "$nav_tuning_ws" \
  --setenv NAV_TUNING_BASE_WS "$nav_tuning_base" \
  --setenv NAV_TUNING_MODE "$nav_tuning_mode" \
  --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
  bash -c '
    set -e
    source scripts/setup_env.sh
    export CMAKE_PREFIX_PATH="$PWD/devel:$NAV_TUNING_BASE_WS/devel:$CMAKE_PREFIX_PATH"
    export PYTHONPATH="$PWD/devel/lib/python3/dist-packages:$NAV_TUNING_BASE_WS/devel/lib/python3/dist-packages:$PYTHONPATH"
    export ROS_HOME="$PWD/runtime/isolated/ros-home" ROS_LOG_DIR="$PWD/validation/navigation_tuning_20261023/ros_logs"
    export XDG_CONFIG_HOME="$PWD/runtime/isolated/config" XDG_CACHE_HOME="$PWD/runtime/isolated/cache"
    mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME"
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://127.0.0.1:11523 ROS_IP=127.0.0.1
    if [[ "$NAV_TUNING_MODE" == build ]]; then
      exec catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="autolabor_canbus_driver;robot_bringup;autolabor_coverage;autolabor_operator_gui;autolabor_fod_msgs;autolabor_fod_control;autolabor_fod_vision;teb_local_planner" -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON -j2 -l2
    fi
    export QT_QPA_PLATFORM=offscreen
    python3 -B scripts/test_conbine_preview_relay.py
    python3 -B -m unittest distribution.j6m_bpu_navigation.surround.tests.test_operator_protocol
    catkin_make run_tests_autolabor_coverage run_tests_autolabor_operator_gui run_tests_teb_local_planner -j2 -l2
    catkin_test_results build/test_results
  '
