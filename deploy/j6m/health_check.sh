#!/usr/bin/env bash
set -euo pipefail

RUNTIME_BASE="${J6M_RUNTIME_BASE:-/map/autolabor_runtime}"
ROOTFS="${J6M_ROOTFS:-$RUNTIME_BASE/rootfs}"

[[ "$(id -u)" == 0 ]] || { echo "health_check.sh must run as root on J6M." >&2; exit 2; }
cleanup() {
  "$RUNTIME_BASE/bin/unmount_chroot.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT
"$RUNTIME_BASE/bin/mount_chroot.sh" >/dev/null

df -h /map /userdata
ip -br -4 address

chroot "$ROOTFS" /bin/bash -lc '
  set -eo pipefail
  source /opt/ros/noetic/setup.bash
  source /opt/autolabor/ros/install/setup.bash
  source /opt/autolabor/dual_host/current/setup.bash
  set -u
  python3 -c "import socket; assert socket.gethostbyname(\"localhost\") == \"127.0.0.1\"; socket.gethostbyname(socket.gethostname())"
  packages=(
    autolabor_dual_host autolabor_dual_lidar autolabor_fod_control
    autolabor_fod_msgs conventional fast_lio fast_lio_localization livox_ros_driver2 move_base
    pointcloud_to_laserscan robot_bringup teb_local_planner topic_tools
    map_server
  )
  for package in "${packages[@]}"; do rospack find "$package"; done
  python3 -m py_compile \
    /opt/autolabor/dual_host/current/lib/autolabor_dual_host/cmd_vel_watchdog.py \
    /opt/autolabor/dual_host/current/lib/autolabor_dual_host/move_base_pause_bridge.py \
    /opt/autolabor/dual_host/current/lib/robot_bringup/fused_scan_mapper.py
  executables=(
    /opt/autolabor/dual_host/current/lib/autolabor_dual_lidar/optional_cloud_enhancer
    /opt/autolabor/dual_host/current/lib/robot_bringup/livox_custom_to_pointcloud
    /opt/autolabor/ros/install/lib/fast_lio/fastlio_mapping
    /opt/autolabor/dual_host/current/lib/fast_lio_localization/fast_lio_map_localizer
    /opt/autolabor/dual_host/current/lib/map_server/map_server
  )
  for executable in "${executables[@]}"; do
    test -x "$executable"
    if ldd "$executable" | grep -q "not found"; then
      ldd "$executable" >&2
      echo "Unresolved shared libraries: $executable" >&2
      exit 1
    fi
  done
  roslaunch --files autolabor_dual_host j6m_fastlio_navigation.launch >/dev/null
  rosmsg md5 livox_ros_driver2/CustomMsg
  rosmsg md5 autolabor_fod_msgs/FodDetectionArray
  test ! -e /opt/autolabor/dual_host/current/share/zed_wrapper
  ! command -v nvcc >/dev/null
'

echo "J6M dual-host static health check passed."
