#!/usr/bin/env bash
# Process-local, read-only dependency reuse; all generated files stay in B.
OPERATOR_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
[[ "$OPERATOR_WS" == /home/slam/j6m-dev-b-surround ]] || { echo 'Unexpected B workspace' >&2; return 2; }
export OPERATOR_RUNTIME="$OPERATOR_WS/runtime/surround_operator"
export PYTHONDONTWRITEBYTECODE=1
set +u
source /home/slam/robot_j6m_ws_navigation_20260907/devel/setup.bash
operator_deps=/home/slam/robot_ws/.deps/sysroot
export CMAKE_PREFIX_PATH="$operator_deps/opt/ros/noetic:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$operator_deps/opt/ros/noetic/lib:$operator_deps/usr/lib/aarch64-linux-gnu:$operator_deps/usr/lib/aarch64-linux-gnu/openblas-pthread:$operator_deps/usr/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_LIBRARY_PATH="$operator_deps/usr/lib/aarch64-linux-gnu:$operator_deps/opt/ros/noetic/lib:${CMAKE_LIBRARY_PATH:-}"
export CPATH="$operator_deps/opt/ros/noetic/include:$operator_deps/usr/include:${CPATH:-}"
export PYTHONPATH="$operator_deps/opt/ros/noetic/lib/python3/dist-packages:$operator_deps/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$OPERATOR_WS/src:$operator_deps/opt/ros/noetic/share:/opt/ros/noetic/share"
if [[ -r "$OPERATOR_RUNTIME/devel/setup.bash" ]]; then
  source "$OPERATOR_RUNTIME/devel/setup.bash" --extend
fi
export ROS_HOME="$OPERATOR_RUNTIME/ros-home"
export ROS_LOG_DIR="$OPERATOR_RUNTIME/log"
export NVIDIA_FOD_BACKEND=j6m_surround_bpu
unset ROS_HOSTNAME
unset operator_deps
set -u
