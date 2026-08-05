#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
PRIVATE_SETUP="${PRIVATE_SETUP:-$ROBOT_WS/.deps/setup.bash}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
BUILD_JOBS="${BUILD_JOBS:-6}"

if [[ -f "$PRIVATE_SETUP" ]]; then
  source "$PRIVATE_SETUP"
else
  source "$ROS_SETUP"
fi

cd "$ROBOT_WS"
exec catkin_make \
  -DCATKIN_WHITELIST_PACKAGES= \
  -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DMVS_ROOT="$ROBOT_WS/.deps/mvs" \
  -DLIVOX_LIDAR_SDK_LIBRARY="$ROBOT_WS/.deps/livox-sdk2/lib/liblivox_lidar_sdk_static.a" \
  "-j$BUILD_JOBS" \
  "-l$BUILD_JOBS" \
  "$@"
