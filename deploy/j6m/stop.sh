#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUAL_HOST_STOP_GRACE_SEC="${J6M_STOP_GRACE_SEC:-25}"
DUAL_HOST_STOP_TERM_SEC="${J6M_STOP_TERM_SEC:-5}"
DUAL_HOST_STOP_KILL_SEC="${J6M_STOP_KILL_SEC:-2}"
source "$SCRIPT_DIR/process_control.sh"

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/autolabor_runtime}"
PID_FILE="$RUNTIME_BASE/dual_host/run/j6m_stack.pid"
LAUNCHER_PID_FILE="$RUNTIME_BASE/dual_host/run/j6m_launcher.pid"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"
LAUNCHER_PATTERN="(^|[[:space:]])${RUNTIME_BASE}/dual_host/bin/start\\.sh([[:space:]]|$)"

[[ "$(id -u)" == 0 ]] || { echo "stop.sh must run as root on J6M." >&2; exit 2; }

if [[ ! -f "$PID_FILE" && ! -f "$LAUNCHER_PID_FILE" ]]; then
  echo "J6M dual-host stack is not running."
  "$RUNTIME_BASE/bin/unmount_chroot.sh" >/dev/null 2>&1 || true
  exit 0
fi

status=0
dual_host_stop_pid_file "$LAUNCHER_PID_FILE" "J6M dual-host launcher" \
  "$LAUNCHER_PATTERN" || status=$?
dual_host_stop_pid_file "$PID_FILE" "J6M dual-host stack" \
  '(^|/)j6m_stack\.sh([[:space:]]|$)' || status=$?

if [[ -f "$PID_FILE" || -f "$LAUNCHER_PID_FILE" ]]; then
  echo "J6M PID record remains after shutdown; leaving the chroot mounted for inspection." >&2
  exit 1
fi

for unmount_attempt in $(seq 1 25); do
  "$RUNTIME_BASE/bin/unmount_chroot.sh" >/dev/null 2>&1 || true
  ! mountpoint -q "$ROOTFS/proc" && break
  sleep 0.2
done
if mountpoint -q "$ROOTFS/proc"; then
  echo "J6M stack exited, but the chroot is still mounted after bounded retries." >&2
  status=1
fi
if (( status == 0 )); then
  echo "J6M dual-host stack stopped; only its verified process tree was signalled."
fi
exit "$status"
