#!/bin/bash
# No shell setup is sourced before the clean isolated standard-library bootstrap.
set -eu
exec /usr/bin/env -i HOME=/home/robot USER=robot LOGNAME=robot LANG=C.UTF-8 PATH=/usr/bin:/bin \
  /usr/bin/python3 -I -S /home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/workspace/scripts/astra_gazebo/navigation_n06_bootstrap_r05.py "$@"
