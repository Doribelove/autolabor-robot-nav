#!/usr/bin/env bash
# Recorded navigation display, isolated from vehicle networking and devices.
set -euo pipefail
replay_ws=${CLEANING_REPLAY_WORKSPACE:-$(cd -- "$(dirname -- "$0")/.." && pwd)}
replay_snapshot=${CLEANING_REPLAY_SNAPSHOT:-}
replay_unit=autolabor-conbine-cleaning-replay.service
replay_base="$replay_ws/runtime/cleaning_replay"
case "${1:-status}" in
  start)
    replay_mode="${3:-recorded}"
    [[ "$replay_mode" == recorded || "$replay_mode" == mapping ]] || { echo 'Mode: recorded or mapping'; exit 2; }
    if systemctl --user is-active --quiet "$replay_unit"; then
      exec bash "$0" status
    fi
    replay_record=$(realpath "${CLEANING_REPLAY_RECORDING_DIR:-$replay_ws/rosbags/${2:-清扫2}}")
    [[ -f "$replay_record/recording.json" ]] || { echo 'Recording directory is missing'; exit 2; }
    replay_root="$replay_base/session_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$replay_root"/{tmp,ros-home,ros-logs,qt-config,cache,fastlio_io/Log,fastlio_io/PCD}
    ln -sfn "$replay_root" "$replay_base/current"
    systemd-run --user --unit="$replay_unit" --collect --working-directory="$replay_ws" \
      --property=KillMode=mixed --property=TimeoutStopSec=35 \
      --setenv="DISPLAY=${DISPLAY:-:0}" --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      --setenv="CLEANING_REPLAY_WORKSPACE=$replay_ws" --setenv="CLEANING_REPLAY_SNAPSHOT=$replay_snapshot" \
      --property="StandardOutput=append:$replay_root/supervisor.log" \
      --property="StandardError=append:$replay_root/supervisor.log" \
      /bin/bash "$0" run "$replay_record" "$replay_root" "$replay_mode"
    ;;
  run)
    replay_record="$2"; replay_root="$3"; replay_mode="${4:-recorded}"
    replay_devices=()
    for replay_gpu in /dev/dri /dev/nvmap /dev/nvhost-ctrl /dev/nvhost-ctrl-gpu /dev/nvhost-gpu /dev/nvhost-as-gpu /dev/nvhost-vic /dev/nvidia0 /dev/nvidiactl /dev/nvidia-modeset /dev/nvgpu; do
      [[ ! -e "$replay_gpu" ]] || replay_devices+=(--dev-bind "$replay_gpu" "$replay_gpu")
    done
    replay_overlays=()
    if [[ -n "$replay_snapshot" ]]; then
      while IFS= read -r replay_path; do
        replay_overlays+=(--ro-bind "$replay_snapshot/workspace/$replay_path" "$replay_ws/$replay_path")
      done < "$replay_snapshot/overlay_paths.txt"
      while IFS= read -r replay_path; do
        replay_overlays+=(--ro-bind "$replay_snapshot/record_inputs/$replay_path" "$replay_record/$replay_path")
      done < "$replay_snapshot/input_paths.txt"
      replay_overlays+=(--ro-bind "$replay_snapshot/bin/fastlio_mapping" /home/slam/robot_j6m_ws_navigation_20260907/devel/lib/fast_lio/fastlio_mapping)
    fi
    exec bwrap --die-with-parent --unshare-net --unshare-pid --ro-bind / / \
      "${replay_overlays[@]}" \
      --bind "$replay_root" "$replay_root" --dev /dev "${replay_devices[@]}" \
      --bind "$replay_root/fastlio_io" /home/slam/robot_j6m_ws_navigation_20260907/runtime/fast_lio \
      --proc /proc --bind "$replay_root/tmp" /tmp --ro-bind /tmp/.X11-unix /tmp/.X11-unix \
      --chdir "$replay_ws" --setenv ROBOT_OPTIMIZED_SANDBOX 1 \
      --setenv CLEANING_REPLAY_ROOT "$replay_root" --setenv CLEANING_RECORDING_DIR "$replay_record" \
      --setenv CLEANING_REPLAY_MODE "$replay_mode" \
      --setenv ROS_MASTER_URI http://127.0.0.1:11593 --setenv ROS_IP 127.0.0.1 \
      --setenv ROS_HOME "$replay_root/ros-home" --setenv ROS_LOG_DIR "$replay_root/ros-logs" \
      --setenv XDG_CONFIG_HOME "$replay_root/qt-config" --setenv XDG_CACHE_HOME "$replay_root/cache" \
      --setenv BEV_PREVIEW_URL http://127.0.0.1:18894 \
      --unsetenv ROS_HOSTNAME --unsetenv SURROUND_PREVIEW_URL \
      bash -c 'set -e; source scripts/setup_env.sh; export PYTHONDONTWRITEBYTECODE=1; exec python3 -B scripts/replay_cleaning_session.py'
    ;;
  pause|resume|restart|video)
    python3 - "$replay_base/current" "$1" <<'PY'
import json,pathlib,sys,time
root=pathlib.Path(sys.argv[1]).resolve()
tmp=root/'command.json.tmp';tmp.write_text(json.dumps(dict(action=sys.argv[2],id=time.time_ns())))
tmp.replace(root/'command.json')
PY
    ;;
  stop)
    if systemctl --user is-active --quiet "$replay_unit"; then systemctl --user stop "$replay_unit"; fi
    ;;
  status)
    systemctl --user show "$replay_unit" -p ActiveState -p MainPID
    if [[ -f "$replay_base/current/state.json" ]]; then cat "$replay_base/current/state.json"; fi
    ;;
  *) echo 'Usage: replay_cleaning_experiment.sh {start [name] [recorded|mapping]|pause|resume|restart|video|stop|status}' >&2; exit 2 ;;
esac
