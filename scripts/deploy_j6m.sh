#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUAL_HOST_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/load_config.sh"

dual_host_validate_fod_model_contract || exit 2
dual_host_validate_fod_weights || exit 2

target="$(dual_host_select_ssh)" || {
  echo "J6M SSH is unavailable at both configured addresses." >&2
  exit 2
}
stamp="$(date +%Y%m%d_%H%M%S)"
rootfs="$J6M_RUNTIME_BASE/rootfs"
remote_build="$rootfs/opt/autolabor/dual_host/build_ws.$stamp"
remote_install="$rootfs/opt/autolabor/dual_host/releases/$stamp/install"
navigation_runtime="${J6M_NAVIGATION_RUNTIME_SOURCE:-${BASE_ROBOT_WS:-/home/slam/robot_ws}/.deps/sysroot/opt/ros/noetic}"
navigation_sysroot="${navigation_runtime%/opt/ros/noetic}"

navigation_runtime_paths=(
  ./lib/map_server
  ./lib/libmap_server_image_loader.so
  ./share/map_server
)
for runtime_path in "${navigation_runtime_paths[@]}"; do
  [[ -e "$navigation_runtime/${runtime_path#./}" ]] || {
    echo "Missing ARM64 navigation runtime: $navigation_runtime/${runtime_path#./}" >&2
    exit 2
  }
done

# map_server and SDL_image need these direct/transitive libraries, but the
# deliberately small J6M rootfs does not provide them. Prefer the pinned ARM64
# sysroot and fall back to the matching ARM64 host copy where necessary.
find_navigation_library() {
  local soname="$1"
  local candidate
  for candidate in \
    "$navigation_sysroot/usr/lib/aarch64-linux-gnu/$soname" \
    "$navigation_sysroot/usr/lib/aarch64-linux-gnu/pulseaudio/$soname" \
    "$navigation_sysroot/lib/aarch64-linux-gnu/$soname" \
    "/usr/lib/aarch64-linux-gnu/$soname" \
    "/usr/lib/aarch64-linux-gnu/pulseaudio/$soname" \
    "/lib/aarch64-linux-gnu/$soname"; do
    if [[ -e "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  echo "Missing ARM64 navigation library: $soname" >&2
  return 1
}

navigation_system_libraries=(
  "$(find_navigation_library libyaml-cpp.so.0.6)"
  "$(find_navigation_library libSDL-1.2.so.0)"
  "$(find_navigation_library libSDL_image-1.2.so.0)"
  "$(find_navigation_library libasound.so.2)"
  "$(find_navigation_library libpulse-simple.so.0)"
  "$(find_navigation_library libpulse.so.0)"
  "$(find_navigation_library libcaca.so.0)"
  "$(find_navigation_library libpulsecommon-13.99.so)"
  "$(find_navigation_library libslang.so.2)"
  "$(find_navigation_library libwrap.so.0)"
  "$(find_navigation_library libsndfile.so.1)"
  "$(find_navigation_library libasyncns.so.0)"
  "$(find_navigation_library libFLAC.so.8)"
)
for system_library in "${navigation_system_libraries[@]}"; do
  file -L "$system_library" | grep -q 'ARM aarch64' || {
    echo "Navigation library is not ARM64: $system_library" >&2
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
  ./src/navigation_arena/forks/navigation/local_planner/teb
  ./src/perception_ldlidar/autolabor_dual_lidar
  ./src/localization_fastlio/FAST_LIO
  ./src/localization_fastlio/fast_lio_localization
  ./src/application/autolabor_coverage
  ./src/application/autolabor_fod_control
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
  grep -Fq 'requested_fod_motion_enabled' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
  grep -Fq 'requested_fod_model_sha256' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
  grep -Fq 'requested_fod_required_class_names' '$J6M_RUNTIME_BASE/dual_host/bin/start.sh'
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
      -DFAST_LIO_RUNTIME_DIR=/var/lib/autolabor/fast_lio/ \
      -DCATKIN_WHITELIST_PACKAGES=conventional\\;teb_local_planner\\;fast_lio\\;fast_lio_localization\\;autolabor_coverage\\;robot_bringup\\;autolabor_fod_control\\;autolabor_dual_lidar\\;autolabor_dual_host
    test -f /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    source /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    rospack find autolabor_dual_host >/dev/null
    rospack find autolabor_dual_lidar >/dev/null
    rospack find fast_lio_localization >/dev/null
    rospack find autolabor_coverage >/dev/null
    rospack find autolabor_fod_control >/dev/null
    rospack find robot_bringup >/dev/null
    rospack find teb_local_planner >/dev/null
    rosmsg md5 autolabor_coverage/CoverageRegion >/dev/null
    rosmsg md5 autolabor_coverage/CoveragePlanningParameters >/dev/null
    rosmsg md5 autolabor_coverage/CoverageStatus >/dev/null
    rosmsg md5 autolabor_coverage/EnforcedPath >/dev/null
    rosmsg md5 autolabor_coverage/HybridTransitionRequest >/dev/null
    rosmsg md5 autolabor_coverage/HybridTransitionResult >/dev/null
    rosmsg md5 autolabor_coverage/TransitProfile >/dev/null
    rossrv md5 autolabor_coverage/PlanCoverage >/dev/null
    rossrv md5 autolabor_coverage/PrecomputeTransitions >/dev/null
    rossrv md5 autolabor_coverage/CancelCoverageBatch >/dev/null
    rossrv md5 autolabor_coverage/SetCoverageOwner >/dev/null
    rossrv md5 autolabor_coverage/SetEnforcedPath >/dev/null
    rossrv md5 autolabor_coverage/SetNavigationProfile >/dev/null
    rossrv md5 autolabor_coverage/SetCoveragePlanningDefaults >/dev/null
    rossrv md5 autolabor_coverage/StartCoverageBatch >/dev/null
  '
  '$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null"

# The minimal J6M rootfs does not carry the Debian map_server package.
# Reuse the project's pinned ARM64 Noetic sysroot and place the runtime in the
# release overlay, where setup.bash provides both ROS_PACKAGE_PATH and
# LD_LIBRARY_PATH without modifying the base rootfs.
(
  cd "$navigation_runtime"
  rsync -aR "${navigation_runtime_paths[@]}" "$target:$remote_install/"
)
# Dereference SONAME symlinks so every release is self-contained even when the
# target rootfs has neither the symlink nor its versioned ELF target.
rsync -aL "${navigation_system_libraries[@]}" "$target:$remote_install/lib/"

ssh "$target" "set -eu
  '$J6M_RUNTIME_BASE/bin/mount_chroot.sh' >/dev/null
  trap \"'$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null 2>&1 || true\" EXIT
  chroot '$rootfs' /usr/bin/env RELEASE='$stamp' /bin/bash -lc '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    source /opt/autolabor/ros/install/setup.bash
    source /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/setup.bash
    rospack find map_server >/dev/null
    rospack find autolabor_coverage >/dev/null
    rospack find autolabor_fod_control >/dev/null
    rospack find teb_local_planner >/dev/null
    rosmsg md5 autolabor_coverage/CoverageRegion >/dev/null
    rosmsg md5 autolabor_coverage/CoveragePlanningParameters >/dev/null
    rosmsg md5 autolabor_coverage/CoverageStatus >/dev/null
    rosmsg md5 autolabor_coverage/EnforcedPath >/dev/null
    rosmsg md5 autolabor_coverage/HybridTransitionRequest >/dev/null
    rosmsg md5 autolabor_coverage/HybridTransitionResult >/dev/null
    rosmsg md5 autolabor_coverage/TransitProfile >/dev/null
    rossrv md5 autolabor_coverage/PlanCoverage >/dev/null
    rossrv md5 autolabor_coverage/PrecomputeTransitions >/dev/null
    rossrv md5 autolabor_coverage/CancelCoverageBatch >/dev/null
    rossrv md5 autolabor_coverage/SetCoverageOwner >/dev/null
    rossrv md5 autolabor_coverage/SetEnforcedPath >/dev/null
    rossrv md5 autolabor_coverage/SetNavigationProfile >/dev/null
    rossrv md5 autolabor_coverage/SetCoveragePlanningDefaults >/dev/null
    rossrv md5 autolabor_coverage/StartCoverageBatch >/dev/null
    test -x /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/autolabor_coverage/coverage_manager.py
    test -x /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/autolabor_coverage/hybrid_teb_command_mux.py
    test -r /opt/autolabor/dual_host/releases/"\$RELEASE"/install/share/autolabor_coverage/config/coverage.yaml
    test -r /opt/autolabor/dual_host/releases/"\$RELEASE"/install/share/autolabor_coverage/config/coverage_factory_defaults.yaml
    test -x /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/autolabor_fod_control/fod_visual_servo_node.py
    test -r /opt/autolabor/dual_host/releases/"\$RELEASE"/install/share/autolabor_fod_control/launch/visual_recovery.launch
    test -f /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libcoverage_global_planner.so
    test -f /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libcoverage_hybrid_astar.so
    test -f /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libteb_local_planner.so
    test -f /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libgps_geofence_layer.so
    grep -aFq treat_unknown_as_obstacle /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libteb_local_planner.so
    grep -aFq UnknownSpaceGuardLayer /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/libgps_geofence_layer.so
    test -x /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/fast_lio/fastlio_mapping
    grep -aFq /var/lib/autolabor/fast_lio/ /opt/autolabor/dual_host/releases/"\$RELEASE"/install/lib/fast_lio/fastlio_mapping
    if ldd /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/lib/map_server/map_server | grep -q not.found; then
      ldd /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install/lib/map_server/map_server >&2
      echo map_server.has.unresolved.shared.libraries >&2
      exit 1
    fi
    roslaunch --files robot_bringup navigation_j6m.launch use_static_map:=true >/dev/null
    roslaunch --files autolabor_fod_control visual_recovery.launch >/dev/null
    ln -sfn /opt/autolabor/dual_host/releases/\"\$RELEASE\"/install /opt/autolabor/dual_host/current
  '
  '$J6M_RUNTIME_BASE/bin/unmount_chroot.sh' >/dev/null"

echo "Deployed J6M dual-host release $stamp through $target."
echo "Run: ssh -t $target $J6M_RUNTIME_BASE/dual_host/bin/health_check.sh"
