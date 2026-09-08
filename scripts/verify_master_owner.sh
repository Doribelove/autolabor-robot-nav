#!/usr/bin/env bash
# Read-only check. Exit 0 only for this candidate's J6M master; --allow-absent
# also accepts no listener, but never an unidentifiable or foreign process.
set -euo pipefail
allow_absent=false
case "${1:-}" in
  '') ;;
  --allow-absent) allow_absent=true ;;
  *) exit 2 ;;
esac
ssh -o BatchMode=yes -o ConnectTimeout=4 root@192.168.10.100 \
  bash -s -- "$allow_absent" <<'REMOTE'
set -euo pipefail
listeners="$(ss -H -ltnp 'sport = :11311')"
if [[ -z "$listeners" ]]; then
  [[ "$1" == true ]]
  exit $?
fi
pids="$(printf '%s\n' "$listeners" | sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' | sort -u)"
[[ -n "$pids" ]] || { echo 'Cannot identify J6M ROS master owner.' >&2; exit 3; }
for pid in $pids; do
  root="$(readlink "/proc/$pid/root")"
  [[ "$root" == /map/robot_j6m_navigation_20260907/rootfs ]] || {
    echo "ROS master PID $pid is outside the candidate; refusing shared-graph changes." >&2
    exit 3
  }
done
REMOTE
