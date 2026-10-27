#!/bin/bash
# A separate live entry; the preview and historical protected launchers stay intact.
set -euo pipefail
live_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
[[ "$live_ws" == /home/slam/robot_j6m_ws_conbine ]] || exit 2
if [[ "${1:-status}" != run ]]; then
  if (( $#==0 )); then set -- status; fi
  exec python3 -B "$live_ws/scripts/conbine_live_session.py" "$@"
fi
live_root="$live_ws/runtime/conbine_live"
mkdir -p "$live_root"/{tmp,ros-home,ros-logs,qt-config,cache}
exec >> "$live_root/supervisor.log" 2>&1
live_can="$(readlink -f /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0)"
[[ -c "$live_can" ]] || { echo 'Confirmed chassis adapter is absent' >&2; exit 2; }
live_owners="$(fuser "$live_can" 2>/dev/null || true)"
[[ -z "$live_owners" ]] || { echo "Chassis adapter is occupied by PID(s): $live_owners" >&2; exit 3; }
live_devices=(--dev-bind "$live_can" "$live_can" --ro-bind /dev/serial /dev/serial)
for live_gpu in /dev/dri /dev/nvmap /dev/nvhost-ctrl /dev/nvhost-ctrl-gpu /dev/nvhost-gpu /dev/nvhost-as-gpu /dev/nvhost-vic /dev/nvidia0 /dev/nvidiactl /dev/nvidia-modeset /dev/nvgpu; do
  [[ ! -e "$live_gpu" ]] || live_devices+=(--dev-bind "$live_gpu" "$live_gpu")
done
exec bwrap --die-with-parent --unshare-pid --ro-bind / / --bind "$live_ws" "$live_ws" \
  --dev /dev "${live_devices[@]}" --proc /proc \
  --bind "$live_root/tmp" /tmp --ro-bind /tmp/.X11-unix /tmp/.X11-unix \
  --chdir "$live_ws" --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
  --setenv ROS_MASTER_URI http://192.168.10.100:11583 --setenv ROS_IP 192.168.10.50 \
  --setenv ROS_HOME "$live_root/ros-home" --setenv ROS_LOG_DIR "$live_root/ros-logs" \
  --setenv XDG_CONFIG_HOME "$live_root/qt-config" --setenv XDG_CACHE_HOME "$live_root/cache" \
  --unsetenv ROS_HOSTNAME --unsetenv SURROUND_PREVIEW_URL \
  bash -c 'set -e; source scripts/setup_env.sh; export CMAKE_PREFIX_PATH="$PWD/devel:/home/slam/robot_j6m_ws_navigation_20260907/devel:$CMAKE_PREFIX_PATH"; export ROS_PACKAGE_PATH="$PWD/src:/home/slam/robot_j6m_ws_navigation_20260907/src:$ROS_PACKAGE_PATH"; export PYTHONPATH="$PWD/devel/lib/python3/dist-packages:/home/slam/robot_j6m_ws_navigation_20260907/devel/lib/python3/dist-packages:$PYTHONPATH"; exec python3 -B scripts/conbine_live_session.py run'
