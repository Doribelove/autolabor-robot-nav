#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/noetic/setup.bash
set -u
unset ROS_HOSTNAME
cd "$repo"
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -DAUTOLABOR_COVERAGE_WORKSPACE_CONTRACT_TESTS=OFF \
  -j"${BUILD_JOBS:-2}" -l"${BUILD_JOBS:-2}"
