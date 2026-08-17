#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUAL_HOST_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"
source "$SCRIPT_DIR/setup_env.sh"

BUILD_JOBS="${BUILD_JOBS:-4}"
mkdir -p "$DUAL_HOST_WS/runtime/fast_lio/Log" "$DUAL_HOST_WS/runtime/fast_lio/PCD"

cd "$DUAL_HOST_WS"
exec catkin_make \
  -DCATKIN_WHITELIST_PACKAGES= \
  -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_ENABLE_TESTING=ON \
  -DFAST_LIO_RUNTIME_DIR="$DUAL_HOST_WS/runtime/fast_lio/" \
  -DLIVOX_LIDAR_SDK_LIBRARY="$BASE_DEPS/livox-sdk2/lib/liblivox_lidar_sdk_static.a" \
  "-j$BUILD_JOBS" \
  "-l$BUILD_JOBS" \
  "$@"
