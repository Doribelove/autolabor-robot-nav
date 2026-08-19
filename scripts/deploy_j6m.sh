#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUAL_HOST_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"

target="$(dual_host_select_ssh)" || {
  echo "J6M SSH is unavailable at both configured addresses." >&2
  exit 2
}
stamp="$(date +%Y%m%d_%H%M%S)"
rootfs="$J6M_RUNTIME_BASE/rootfs"
remote_build="$rootfs/opt/autolabor/dual_host/build_ws.$stamp"
remote_install="$rootfs/opt/autolabor/dual_host/releases/$stamp/install"
navigation_runtime="${J6M_NAVIGATION_RUNTIME_SOURCE:-${BASE_ROBOT_WS:-/home/slam/robot_ws}/.deps/sysroot/opt/ros/noetic}"

navigation_runtime_paths=(
  ./lib/amcl
  ./lib/map_server
  ./lib/libamcl_map.so
  ./lib/libamcl_pf.so
  ./lib/libamcl_sensors.so
  ./lib/libmap_server_image_loader.so
  ./lib/python3/dist-packages/amcl
  ./share/amcl
  ./share/map_server
)
for runtime_path in "${navigation_runtime_paths[@]}"; do
  [[ -e "$navigation_runtime/${runtime_path#./}" ]] || {
    echo "Missing ARM64 navigation runtime: $navigation_runtime/${runtime_path#./}" >&2
    exit 2
  }
done

ssh "$target" "set -eu
  test -x '$rootfs/bin/bash'
  test -f '$rootfs/opt/autolabor/ros/install/setup.bash'
  test ! -f '$J6M_RUNTIME_BASE/dual_host/run/j6m_stack.pid'
  test ! -f '$J6M_RUNTIME_BASE/dual_host/run/j6m_launcher.pid'
  mkdir -p '$remote_build' '$J6M_RUNTIME_BASE/bin' '$J6M_RUNTIME_BASE/dual_host/bin' '$J6M_RUNTIME_BASE/dual_host/config' '$J6M_RUNTIME_BASE/dual_host/run'"

paths=(
  ./src/CMakeLists.txt
  ./src/navigation_arena/arena-rosnav-3D/arena_navigation/arena_local_planer/model_based/conventional
  ./src/perception_ldlidar/autolabor_dual_lidar
  ./src/scripts/robot_bringup
  ./src/platform/autolabor_dual_host
)
rsync -aR \
  --exclude='.git' --exclude='build' --exclude='devel' --exclude='install' \
  --exclude='log' --exclude='__pycache__' --exclude='*.pyc' \
  "${paths[@]}" "$target:$remote_build/"

rsync -a \
  "$DUAL_HOST_WS/deploy/j6m/start.sh" \
  "$DUAL_HOST_WS/deploy/j6m/stop.sh" \
  "$DUAL_HOST_WS/deploy/j6m/health_check.sh" \
  "$DUAL_HOST_WS/deploy/j6m/rollback.sh" \
  "$DUAL_HOST_WS/scripts/process_control.sh" \
  "$target:$J6M_RUNTIME_BASE/dual_host/bin/"
rsync -a \
  "$DUAL_HOST_WS/deploy/j6m/mount_chroot.sh" \
  "$DUAL_HOST_WS/deploy/j6m/unmount_chroot.sh" \
  "$target:$J6M_RUNTIME_BASE/bin/"
rsync -a "$DUAL_HOST_CONFIG" \
  "$target:$J6M_RUNTIME_BASE/dual_host/config/dual_host.env"

ssh "$target" "set -eu
  chmod 0755 '$J6M_RUNTIME_BASE/dual_host/bin/'*.sh '$J6M_RUNTIME_BASE/bin/'*.sh
  '$J6M_RUNTIME_BASE/bin/mount_chroot.sh' >/dev/null
  trap \"'$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null 2>&1 || true\" EXIT
  chroot '$rootfs' /usr/bin/env RELEASE='$stamp' /bin/bash -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /opt/autolabor/ros/install/setup.bash
    set -u
    cd /opt/autolabor/dual_host/build_ws.\"\$RELEASE\"
    catkin_make install -j2 -l2 \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_INSTALL_PREFIX=/opt/autolabor/dual_host/releases/\"\$RELEASE\"/install \
      -DCATKIN_WHITELIST_PACKAGES=conventional\\;robot_bringup\\;autolabor_dual_lidar\\;autolabor_dual_host
    test -f /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    source /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    rospack find autolabor_dual_host >/dev/null
    rospack find autolabor_dual_lidar >/dev/null
    rospack find robot_bringup >/dev/null
  '
  '$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null"

# The minimal J6M rootfs does not carry the Debian amcl/map_server packages.
# Reuse the project's pinned ARM64 Noetic sysroot and place the runtime in the
# release overlay, where setup.bash provides both ROS_PACKAGE_PATH and
# LD_LIBRARY_PATH without modifying the base rootfs.
(
  cd "$navigation_runtime"
  rsync -aR "${navigation_runtime_paths[@]}" "$target:$remote_install/"
)

ssh "$target" "set -eu
  '$J6M_RUNTIME_BASE/bin/mount_chroot.sh' >/dev/null
  trap \"'$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null 2>&1 || true\" EXIT
  chroot '$rootfs' /usr/bin/env RELEASE='$stamp' /bin/bash -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /opt/autolabor/ros/install/setup.bash
    source /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    rospack find amcl >/dev/null
    rospack find map_server >/dev/null
    ! ldd /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/lib/amcl/amcl | grep -q not.found
    ! ldd /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/lib/map_server/map_server | grep -q not.found
    roslaunch --files robot_bringup navigation_j6m.launch use_static_map:=true >/dev/null
    ln -sfn /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install /opt/autolabor/dual_host/current
  '
  '$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null"

echo "Deployed J6M dual-host release $stamp through $target."
echo "Run: ssh -t $target $J6M_RUNTIME_BASE/dual_host/bin/health_check.sh"
