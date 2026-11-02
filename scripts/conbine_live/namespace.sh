#!/bin/bash
# Called only inside this session's private mount/PID namespace.
set -euo pipefail
run_dir="$1"
[[ "$run_dir" == /map/conbine_navigation/session-* ]] || exit 2
base=/map/autolabor_runtime/rootfs
navigation=/map/robot_j6m_navigation_20260907/rootfs/opt/autolabor/dual_host
driver=/map/j6m-dev-b/livox2_20261019_2zho_g16
map_dir="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["board_map_directory"])' "$run_dir/session.json")"
[[ -z "$map_dir" || "$map_dir" == /map/robot_j6m_navigation_20260907/maps/releases/* || "$map_dir" == /map/conbine_navigation/maps/* ]] || exit 2
mount --make-rprivate /
mount --bind "$base" "$base"
mount -o remount,bind,ro "$base"
mount --bind "$run_dir" "$base/tmp"
mount --bind "$navigation" "$base/opt/autolabor/dual_host"
mount -o remount,bind,ro "$base/opt/autolabor/dual_host"
bindings=("${driver}/install:livox" "${driver}/sdk:sdk")
[[ -z "$map_dir" ]] || bindings+=("${map_dir}:map_set")
for item in "${bindings[@]}"; do
  mount --bind "${item%:*}" "$base/tmp/${item##*:}"
  mount -o remount,bind,ro "$base/tmp/${item##*:}"
done
mount --bind /map/conbine_navigation/persistent "$base/tmp/persistent"
mount --bind "$run_dir/varlib" "$base/var/lib"
mount -t tmpfs -o size=16m,mode=755 tmpfs "$base/dev"
for spec in 'null 1 3' 'zero 1 5' 'random 1 8' 'urandom 1 9'; do
  read -r device major minor <<<"$spec"
  mknod -m 666 "$base/dev/$device" c "$major" "$minor"
done
mkdir -m 1777 "$base/dev/shm"
mount -t proc proc "$base/proc"
mount --bind "$run_dir/hosts" "$base/etc/hosts"
exec chroot "$base" /usr/bin/env -i \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 \
  PYTHONDONTWRITEBYTECODE=1 ROS_HOME=/tmp/persistent/ros-home ROS_LOG_DIR=/tmp/logs \
  ROS_MASTER_URI=http://192.168.10.100:11583 ROS_IP=192.168.10.100 \
  /bin/bash --noprofile --norc /tmp/run_inner.sh "${2:-run}"
