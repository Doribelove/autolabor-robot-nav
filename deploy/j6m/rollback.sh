#!/usr/bin/env bash
set -euo pipefail

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/robot_j6m_optimized_20260905}"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"
[[ "$RUNTIME_BASE" == /map/robot_j6m_optimized_20260905 &&
   "$ROOTFS" == /map/robot_j6m_optimized_20260905/rootfs ]] || exit 2
BASE="$ROOTFS/opt/autolabor/dual_host"

[[ "$(id -u)" == 0 ]] || { echo "rollback.sh must run as root on J6M." >&2; exit 2; }
[[ ! -f "$RUNTIME_BASE/dual_host/run/j6m_stack.pid" ]] || {
  echo "Stop the J6M stack before switching a release." >&2
  exit 3
}

if [[ "${1:-}" == "--list" ]]; then
  find "$BASE/releases" -mindepth 2 -maxdepth 2 -type d -name install -printf '%h\n' |
    sed 's#.*/##' | sort
  exit 0
fi

release="${1:-}"
[[ "$release" =~ ^[0-9]{8}_[0-9]{6}$ ]] || {
  echo "Usage: $0 --list | YYYYMMDD_HHMMSS" >&2
  exit 2
}
[[ -r "$BASE/releases/$release/install/setup.bash" ]] || {
  echo "Unknown release: $release" >&2
  exit 4
}
# The link is consumed inside chroot: a host-side $ROOTFS prefix is invalid.
# Replace the link atomically, retaining the previously installed release.
next_link="$BASE/current.next.$$"
trap '[[ ! -L "$next_link" ]] || unlink "$next_link"' EXIT
ln -s "/opt/autolabor/dual_host/releases/$release/install" "$next_link"
mv -Tf -- "$next_link" "$BASE/current"
echo "J6M dual-host overlay switched to release $release."
