#!/usr/bin/env bash
set -euo pipefail

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/autolabor_runtime}"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"

[[ "$(id -u)" == 0 ]] || { echo "unmount_chroot.sh must run as root." >&2; exit 2; }
pid_files=(
  "$RUNTIME_BASE/run/navigation.pid"
  "$RUNTIME_BASE/dual_host/run/j6m_stack.pid"
)
for pid_file in "${pid_files[@]}"; do
  [[ -f "$pid_file" ]] || continue
  pid="$(<"$pid_file")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    echo "A chroot workload is still running (PID $pid); stop it before unmounting." >&2
    exit 3
  fi
done

targets=(
  "$ROOTFS/etc/resolv.conf"
  "$ROOTFS/etc/hosts"
  "$ROOTFS/var/log/autolabor"
  "$ROOTFS/var/lib/autolabor/ros-home"
  "$ROOTFS/var/lib/autolabor/fast_lio"
  "$ROOTFS/var/lib/autolabor/maps"
  "$ROOTFS/var/lib/autolabor/config"
  "$ROOTFS/tmp"
  "$ROOTFS/run"
  "$ROOTFS/sys"
  "$ROOTFS/proc"
  "$ROOTFS/dev"
)
for target in "${targets[@]}"; do
  if mountpoint -q "$target"; then
    umount -R "$target"
  fi
done

echo "J6M chroot mounts have been removed."
