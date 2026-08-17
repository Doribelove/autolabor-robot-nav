#!/usr/bin/env bash

DUAL_HOST_WS="${DUAL_HOST_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BASE_ROBOT_WS="${BASE_ROBOT_WS:-$(cd "$DUAL_HOST_WS/.." && pwd)/robot_ws}"
BASE_DEPS="${BASE_DEPS:-$BASE_ROBOT_WS/.deps}"

# Catkin's generated setup files probe variables that may not exist in a clean
# systemd service environment. Temporarily suspend nounset only while sourcing
# those generated files, then restore the caller's shell policy.
dual_host_restore_nounset=false
if [[ "$-" == *u* ]]; then
  dual_host_restore_nounset=true
  set +u
fi
source /opt/ros/noetic/setup.bash
if [[ -r "$DUAL_HOST_WS/devel/setup.bash" ]]; then
  source "$DUAL_HOST_WS/devel/setup.bash"
fi
if [[ "$dual_host_restore_nounset" == true ]]; then
  set -u
fi
unset dual_host_restore_nounset

dual_host_prepend() {
  local variable_name="$1" value="$2" current_value="${!1:-}"
  [[ -d "$value" ]] || return 0
  [[ ":$current_value:" == *":$value:"* ]] && return 0
  if [[ -n "$current_value" ]]; then
    export "$variable_name=$value:$current_value"
  else
    export "$variable_name=$value"
  fi
}

DEPENDENCY_ROS="$BASE_DEPS/sysroot/opt/ros/noetic"
DEPENDENCY_SYSROOT="$BASE_DEPS/sysroot"
LIVOX_PREFIX="$BASE_DEPS/livox-sdk2"

dual_host_prepend PATH "$DEPENDENCY_ROS/bin"
dual_host_prepend CMAKE_PREFIX_PATH "$DEPENDENCY_ROS"
dual_host_prepend CMAKE_PREFIX_PATH "$LIVOX_PREFIX"
dual_host_prepend ROS_PACKAGE_PATH "$DEPENDENCY_ROS/share"
dual_host_prepend LD_LIBRARY_PATH "$DEPENDENCY_ROS/lib"
dual_host_prepend LD_LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu"
dual_host_prepend LD_LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu/openblas-pthread"
dual_host_prepend LD_LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib"
dual_host_prepend LD_LIBRARY_PATH "$LIVOX_PREFIX/lib"
dual_host_prepend LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu"
dual_host_prepend LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu/openblas-pthread"
dual_host_prepend LIBRARY_PATH "$DEPENDENCY_ROS/lib"
dual_host_prepend LIBRARY_PATH "$LIVOX_PREFIX/lib"
dual_host_prepend CMAKE_LIBRARY_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu"
dual_host_prepend CMAKE_LIBRARY_PATH "$DEPENDENCY_ROS/lib"
dual_host_prepend CMAKE_LIBRARY_PATH "$LIVOX_PREFIX/lib"
dual_host_prepend CMAKE_INCLUDE_PATH "$DEPENDENCY_SYSROOT/usr/include"
dual_host_prepend CMAKE_INCLUDE_PATH "$DEPENDENCY_ROS/include"
dual_host_prepend CMAKE_INCLUDE_PATH "$LIVOX_PREFIX/include"
dual_host_prepend CPATH "$DEPENDENCY_SYSROOT/usr/include"
dual_host_prepend CPATH "$DEPENDENCY_ROS/include"
dual_host_prepend CPATH "$LIVOX_PREFIX/include"
dual_host_prepend PYTHONPATH "$DEPENDENCY_ROS/lib/python3/dist-packages"
dual_host_prepend PYTHONPATH "$DEPENDENCY_SYSROOT/usr/lib/python3/dist-packages"
dual_host_prepend PKG_CONFIG_PATH "$DEPENDENCY_ROS/lib/pkgconfig"
dual_host_prepend PKG_CONFIG_PATH "$DEPENDENCY_SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig"
dual_host_prepend PKG_CONFIG_PATH "$LIVOX_PREFIX/lib/pkgconfig"

export DUAL_HOST_WS BASE_ROBOT_WS BASE_DEPS
export FOD_YOLO_ENV="${FOD_YOLO_ENV:-$BASE_ROBOT_WS/.venv/fod_yolo}"
export FOD_YOLO_PYTHON="${FOD_YOLO_PYTHON:-$FOD_YOLO_ENV/bin/python3}"

unset DEPENDENCY_ROS DEPENDENCY_SYSROOT LIVOX_PREFIX
unset -f dual_host_prepend
