#!/usr/bin/env bash
set -euo pipefail

# The development workspace may keep ROS navigation binaries in its private
# sysroot.  catkin's cached test environment retains the libraries but can
# omit that sysroot from ROS_PACKAGE_PATH, so restore it for this test process.
runner_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${runner_dir}/../../../.." && pwd)"
ros_distro="${ROS_DISTRO:-noetic}"
dependency_root="${BASE_DEPS:-${workspace_root}/.deps}"
private_ros_prefix="${dependency_root}/sysroot/opt/ros/${ros_distro}"

if [[ -d "${private_ros_prefix}/share" ]]; then
  export ROS_PACKAGE_PATH="${private_ros_prefix}/share:${ROS_PACKAGE_PATH:-}"
fi
export CMAKE_PREFIX_PATH="${private_ros_prefix}:${workspace_root}/devel:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${private_ros_prefix}/lib:${workspace_root}/devel/lib:${LD_LIBRARY_PATH:-}"
export PATH="${private_ros_prefix}/bin:${PATH}"

for move_base_binary in \
  "${private_ros_prefix}/lib/move_base/move_base" \
  "/opt/ros/${ros_distro}/lib/move_base/move_base"
do
  if [[ -x "${move_base_binary}" ]]; then
    exec "${move_base_binary}" "$@"
  fi
done

echo "test_move_base_runner: move_base executable not found" >&2
exit 127
