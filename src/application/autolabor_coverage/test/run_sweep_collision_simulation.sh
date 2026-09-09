#!/usr/bin/env bash
set -euo pipefail
ws="$(cd "$(dirname "$0")/../../../.." && pwd)"
if [[ ${ROBOT_OPTIMIZED_SANDBOX:-0} != 1 ]]; then
  exec "$ws/scripts/optimized.sh" run bash "$0" "$@"
fi
# Service-only planner tests: no hardware, reachable real ROS graph, or goals.
exec bwrap --unshare-net --unshare-pid --die-with-parent \
  --ro-bind / / --bind "$ws" "$ws" --tmpfs /tmp --proc /proc --dev /dev \
  --chdir "$ws" -- bash -c '
    source scripts/setup_env.sh
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://127.0.0.1:11493 ROS_IP=127.0.0.1
    exec python3 -u src/application/autolabor_coverage/test/sweep_collision_simulation.py "$@"
  ' simulation "$@"
