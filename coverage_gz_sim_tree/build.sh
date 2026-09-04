#!/usr/bin/env bash
set -euo pipefail

tree_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
navigation_sysroot="/home/slam/robot_ws/.deps/sysroot/opt/ros/noetic"

exec catkin_make \
  -C "${tree_root}/ws" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/home/slam/robot_j6m_ws/devel;${navigation_sysroot};/opt/ros/noetic" \
  -DCMAKE_LIBRARY_PATH="${navigation_sysroot}/lib" \
  -DCATKIN_WHITELIST_PACKAGES="autolabor_coverage;teb_local_planner;coverage_gz_sim" \
  "$@"
