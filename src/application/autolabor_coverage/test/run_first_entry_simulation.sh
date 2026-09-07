#!/usr/bin/env bash
set -euo pipefail
ws=/home/slam/robot_j6m_ws_navigation_20260907
if [[ ${ROBOT_OPTIMIZED_SANDBOX:-0} != 1 ]]; then
  exec "$ws/scripts/optimized.sh" run bash "$0" "$@"
fi
# Network and PID namespaces contain only this simulated ROS graph. No device
# drivers, remote SSH, physical ROS master, or hardware /dev nodes are present.
exec bwrap --unshare-net --unshare-pid --die-with-parent \
  --ro-bind / / --bind "$ws" "$ws" --tmpfs /tmp --proc /proc --dev /dev \
  --chdir "$ws" -- bash -c '
    source scripts/setup_env.sh
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://127.0.0.1:11491 ROS_IP=127.0.0.1
    exec python3 -u src/application/autolabor_coverage/test/first_entry_simulation.py "$@"
  ' simulation "$@"
