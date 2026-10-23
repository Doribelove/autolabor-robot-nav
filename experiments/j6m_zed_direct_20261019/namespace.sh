#!/usr/bin/env bash
set -euo pipefail
candidate=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
rootfs=/map/autolabor_runtime/rootfs
[[ $(id -u) == 0 && $(uname -m) == aarch64 && $# == 1 ]] || exit 2
[[ $(readlink /proc/self/ns/mnt) != $(readlink /proc/1/ns/mnt) ]] || exit 2
[[ $(readlink /proc/self/ns/net) != $(readlink /proc/1/ns/net) ]] || exit 2
case "$candidate" in /map/j6m-dev-b/zed-direct/releases/*) ;; *) exit 2;; esac
case "$1" in build) command=/tmp/build_inner.sh;; self-test) command=/tmp/offline_inner.sh;; *) exit 2;; esac
[[ -f "$candidate/deployment.json" && -f "$candidate/${command#/tmp/}" ]] || exit 2
mount --bind "$rootfs" "$rootfs"
mount -o remount,bind,ro "$rootfs"
mount --bind "$candidate" "$rootfs/tmp"
mount -t tmpfs -o size=16m,mode=755 tmpfs "$rootfs/dev"
mknod -m 666 "$rootfs/dev/null" c 1 3
mknod -m 666 "$rootfs/dev/zero" c 1 5
mknod -m 666 "$rootfs/dev/random" c 1 8
mknod -m 666 "$rootfs/dev/urandom" c 1 9
mkdir -m 1777 "$rootfs/dev/shm"
mount --bind /proc "$rootfs/proc"
mount -o remount,bind,ro "$rootfs/proc"
mount --bind "$candidate/hosts" "$rootfs/etc/hosts"
exec chroot "$rootfs" /usr/bin/env -i \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 PYTHONDONTWRITEBYTECODE=1 TMPDIR=/tmp/tmp \
  /bin/bash --noprofile --norc "$command"
