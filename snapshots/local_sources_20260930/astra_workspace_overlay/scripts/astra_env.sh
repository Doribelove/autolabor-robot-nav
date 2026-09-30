#!/usr/bin/env bash
# Source only the independent Astra workspace and the frozen Python environment.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source /path/to/workspace/scripts/astra_env.sh" >&2
  exit 2
fi

_astra_activate() {
  local astra_ws
  astra_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
  if [[ ! -f "$astra_ws/devel/setup.bash" || ! -f "$astra_ws/.venv/bin/activate" ]]; then
    echo "Build this independent workspace first: $astra_ws/scripts/astra_build.sh" >&2
    return 2
  fi
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
  unset PKG_CONFIG_PATH PYTHONPATH LD_LIBRARY_PATH
  unset GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
  unset GAZEBO_INSTALL_PREFIX OGRE_RESOURCE_PATH
  unset VIRTUAL_ENV ROS_HOSTNAME ROS_IP
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  source /opt/ros/noetic/setup.bash || return
  source /usr/share/gazebo/setup.sh || return
  source "$astra_ws/devel/setup.bash" || return
  source "$astra_ws/.venv/bin/activate" || return
  export THESIS_WS="$astra_ws"
  export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PIP_REQUIRE_VIRTUALENV=true
  export ROS_IP=127.0.0.1
  export ROS_MASTER_URI="http://127.0.0.1:${ASTRA_ROS_PORT:-14700}"
  export GAZEBO_MASTER_URI="http://127.0.0.1:${ASTRA_GAZEBO_PORT:-14701}"
  export GAZEBO_MODEL_DATABASE_URI=""
  export GAZEBO_PLUGIN_PATH="$astra_ws/devel/lib:/opt/ros/noetic/lib:$GAZEBO_PLUGIN_PATH"
  export ROS_HOME="${ASTRA_ROS_HOME:-$astra_ws/.astra_runtime/ros_home}"
  export ROS_LOG_DIR="${ASTRA_ROS_LOG_DIR:-$astra_ws/.astra_runtime/roslog}"
  mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" || return
  export OMP_NUM_THREADS="${ASTRA_CPU_THREADS:-1}"
  export MKL_NUM_THREADS="${ASTRA_CPU_THREADS:-1}"
}
_astra_activate
_astra_status=$?
unset -f _astra_activate
return "$_astra_status"
