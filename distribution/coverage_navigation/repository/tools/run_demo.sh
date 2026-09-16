#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
kind="${1:-sweep}"
if (( $# )); then shift; fi
case "$kind" in
  sweep) port=11493; script=sweep_collision_simulation.py ;;
  preview) port=11491; script=first_entry_simulation.py ;;
  yaw-free) port=11494; script=yaw_free_goal_simulation.py ;;
  *) echo 'Usage: bash tools/run_demo.sh sweep|preview|yaw-free [test options]' >&2; exit 2 ;;
esac
# These fixtures assert loopback is the only interface. A --network none Docker
# container already satisfies this; a workstation needs its own network/PID ns.
if ! python3 -c 'import socket,sys; sys.exit([n for _,n in socket.if_nameindex()] != ["lo"])'; then
  command -v bwrap >/dev/null || { echo 'Use Docker --network none, or install bubblewrap.' >&2; exit 2; }
  exec bwrap --unshare-net --unshare-pid --die-with-parent \
    --ro-bind / / --bind "$repo" "$repo" --tmpfs /tmp --proc /proc --dev /dev \
    --chdir "$repo" -- bash "$repo/tools/run_demo.sh" "$kind" "$@"
fi
set +u
source "$repo/devel/setup.bash"
set -u
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://127.0.0.1:$port" ROS_IP=127.0.0.1
cd "$repo"
exec python3 -u "src/application/autolabor_coverage/test/$script" "$@"
