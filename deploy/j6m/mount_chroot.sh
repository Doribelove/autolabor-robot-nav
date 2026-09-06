#!/usr/bin/env bash
set -euo pipefail

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/robot_j6m_optimized_20260905}"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"

[[ "$RUNTIME_BASE" == /map/robot_j6m_optimized_20260905 &&
   "$ROOTFS" == /map/robot_j6m_optimized_20260905/rootfs ]] || exit 2
"$RUNTIME_BASE/bin/prepare_isolated_rootfs.sh" >/dev/null

[[ "$(id -u)" == 0 ]] || { echo "mount_chroot.sh must run as root." >&2; exit 2; }
[[ -x "$ROOTFS/bin/bash" ]] || { echo "Invalid J6M rootfs: $ROOTFS" >&2; exit 2; }

mkdir -p \
  "$ROOTFS/dev" "$ROOTFS/dev/pts" "$ROOTFS/proc" "$ROOTFS/sys" \
  "$ROOTFS/run" "$ROOTFS/tmp" "$ROOTFS/var/lib/autolabor/config" \
  "$ROOTFS/var/lib/autolabor/maps" "$ROOTFS/var/lib/autolabor/fast_lio" \
  "$ROOTFS/var/lib/autolabor/ros-home" "$ROOTFS/var/log/autolabor" \
  "$ROOTFS/etc" "$RUNTIME_BASE/config" "$RUNTIME_BASE/maps" \
  "$RUNTIME_BASE/fast_lio" "$RUNTIME_BASE/ros-home" \
  "$RUNTIME_BASE/logs" "$RUNTIME_BASE/run" "$RUNTIME_BASE/tmp"
mkdir -p "$RUNTIME_BASE/fast_lio/Log" "$RUNTIME_BASE/fast_lio/PCD"

rbind_once() {
  local source_path="$1" target_path="$2"
  if ! mountpoint -q "$target_path"; then
    mount --rbind "$source_path" "$target_path"
    mount --make-rslave "$target_path"
  fi
}

bind_once() {
  local source_path="$1" target_path="$2"
  if ! mountpoint -q "$target_path"; then
    mount --bind "$source_path" "$target_path"
  fi
}

rbind_once /dev "$ROOTFS/dev"
rbind_once /proc "$ROOTFS/proc"
rbind_once /sys "$ROOTFS/sys"
bind_once "$RUNTIME_BASE/run" "$ROOTFS/run"
bind_once "$RUNTIME_BASE/tmp" "$ROOTFS/tmp"
bind_once "$RUNTIME_BASE/config" "$ROOTFS/var/lib/autolabor/config"
bind_once "$RUNTIME_BASE/maps" "$ROOTFS/var/lib/autolabor/maps"
bind_once "$RUNTIME_BASE/fast_lio" "$ROOTFS/var/lib/autolabor/fast_lio"
bind_once "$RUNTIME_BASE/ros-home" "$ROOTFS/var/lib/autolabor/ros-home"
bind_once "$RUNTIME_BASE/logs" "$ROOTFS/var/log/autolabor"

for host_file in hosts resolv.conf; do
  if [[ -f "/etc/$host_file" ]]; then
    # Own copies, never bind writable host configuration into the candidate.
    if ! mountpoint -q "$ROOTFS/etc/$host_file"; then
      cp --remove-destination --dereference "/etc/$host_file" "$ROOTFS/etc/$host_file"
    fi
  fi
done

echo "J6M chroot mounts are ready under $ROOTFS"
