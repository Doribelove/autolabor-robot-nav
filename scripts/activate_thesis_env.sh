#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this script instead of executing it:" >&2
  echo "  source /home/robot/robot_ws_base_rl/scripts/activate_thesis_env.sh" >&2
  exit 2
fi

THESIS_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

unset CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset PKG_CONFIG_PATH
unset PYTHONPATH

source /opt/ros/noetic/setup.bash
source "$THESIS_WS/devel/setup.bash"
source "$THESIS_WS/.venv/bin/activate"

export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PIP_REQUIRE_VIRTUALENV=true
export THESIS_WS

echo "Thesis environment active: $THESIS_WS"
echo "Python: $(command -v python)"
