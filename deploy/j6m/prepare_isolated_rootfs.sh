#!/usr/bin/env bash
# Create only the candidate runtime. Share the existing Ubuntu userland read-only.
set -euo pipefail
base_root=/map/autolabor_runtime/rootfs
candidate_base=/map/robot_j6m_navigation_20260907
candidate_root="$candidate_base/rootfs"
[[ "$(id -u)" == 0 ]] || exit 2
[[ -x "$base_root/usr/bin/bash" ]] || exit 2
[[ ! -L "$candidate_base" && ! -L "$candidate_root" ]] || exit 2
mkdir -p "$candidate_root" "$candidate_base"/{bin,dual_host/bin,dual_host/config,dual_host/run}
if [[ ! -d "$candidate_root/etc" ]]; then
  cp -a -- "$base_root/etc" "$candidate_root/etc"
fi
mkdir -p "$candidate_root"/{usr,opt/ros,opt/autolabor/ros,opt/autolabor/dual_host,root,dev,proc,sys,run,tmp,var/log,var/lib}
for entry in bin sbin lib; do
  if [[ ! -e "$candidate_root/$entry" && ! -L "$candidate_root/$entry" ]]; then
    ln -s "usr/$entry" "$candidate_root/$entry"
  fi
done
for entry in usr opt/ros opt/autolabor/ros; do
  [[ ! -L "$candidate_root/$entry" ]] || exit 2
  if ! mountpoint -q "$candidate_root/$entry"; then
    mount --bind "$base_root/$entry" "$candidate_root/$entry"
    mount -o remount,bind,ro "$candidate_root/$entry"
  fi
  findmnt -n -o OPTIONS --target "$candidate_root/$entry" | tr ',' '\n' | grep -qx ro || {
    echo "Candidate dependency mount is not read-only: $entry" >&2; exit 3;
  }
done
chroot "$candidate_root" /bin/bash -c 'test -r /opt/ros/noetic/setup.bash; test -r /opt/autolabor/ros/install/setup.bash'
echo "Isolated runtime ready: $candidate_base (Ubuntu and base ROS are read-only mounts)."
