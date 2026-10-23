#!/usr/bin/env bash
# This launcher never enters the vehicle ROS network or hardware bringup.
set -euo pipefail
nav_preview_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
nav_preview_unit=autolabor-navigation-tuning-preview.service
case "${1:-status}" in
  start)
    exec systemd-run --user --unit="$nav_preview_unit" --collect \
      --property=KillMode=mixed --property=TimeoutStopSec=30 \
      --setenv=DISPLAY="${DISPLAY:-:0}" --setenv=XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      "$nav_preview_ws/scripts/navigation_tuning_preview.sh" run ;;
  stop) exec systemctl --user stop "$nav_preview_unit" ;;
  status) exec systemctl --user show "$nav_preview_unit" -p ActiveState -p MainPID -p ExecStart ;;
  run|test) nav_preview_action="$1" ;;
  *) echo 'Usage: navigation_tuning_preview.sh {start|stop|status|test}' >&2;exit 2 ;;
esac
nav_preview_root="$nav_preview_ws/runtime/navigation_tuning_preview"
if [[ "$nav_preview_action" == test ]]; then
  nav_preview_root="$nav_preview_ws/validation/tuning_persistence_20261023/restart_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$nav_preview_root"/{tmp,ros-home,ros-logs,qt-config,cache}
nav_preview_mounts=()
if [[ "$nav_preview_action" == run ]]; then
  nav_preview_mounts+=(--ro-bind /tmp/.X11-unix /tmp/.X11-unix)
fi
exec bwrap --die-with-parent --unshare-net --unshare-pid \
  --ro-bind / / --bind "$nav_preview_ws" "$nav_preview_ws" \
  --bind "$nav_preview_root/tmp" /tmp "${nav_preview_mounts[@]}" \
  --dev-bind /dev /dev --proc /proc --chdir "$nav_preview_ws" \
  --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
  --setenv NAV_TUNING_PREVIEW_DIR "$nav_preview_root" \
  --setenv NAV_TUNING_PREVIEW_ACTION "$nav_preview_action" \
  --setenv ROS_HOME "$nav_preview_root/ros-home" --setenv ROS_LOG_DIR "$nav_preview_root/ros-logs" \
  --setenv XDG_CONFIG_HOME "$nav_preview_root/qt-config" --setenv XDG_CACHE_HOME "$nav_preview_root/cache" \
  bash -c '
    set -e
    source scripts/setup_env.sh
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://127.0.0.1:11543 ROS_IP=127.0.0.1
    if [[ "$NAV_TUNING_PREVIEW_ACTION" == test ]];then
      exec python3 -u scripts/navigation_tuning_preview.py --self-test
    fi
    exec python3 -u scripts/navigation_tuning_preview.py
  '
