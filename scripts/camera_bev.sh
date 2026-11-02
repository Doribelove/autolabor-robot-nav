#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
preview_root="$project_root/runtime/camera_bev_check_20261025"
preview_root="${CAMERA_BEV_ASSETS:-$preview_root}"
bev_config="${CAMERA_BEV_CONFIG:-$project_root/config/three_camera_bev.json}"
board_root="${CAMERA_BEV_BOARD_ROOT:-/map/conbine_camera/bev_check_20261025}"
unit=autolabor-conbine-camera-bev.service
case "${1:-start}" in
  start)
    # Check the requested rig before stopping or acquiring any existing source.
    /usr/bin/python3 -s "$project_root/scripts/three_camera_bev.py" --check \
      --config "$bev_config" --assets "$preview_root" --board-root "$board_root"
    if systemctl --user is-active --quiet "$unit"; then
      active_pid=$(systemctl --user show "$unit" -p MainPID --value)
      /usr/bin/python3 -s - "$active_pid" "$bev_config" "$preview_root" "$project_root" <<'PY'
from pathlib import Path
import sys
args=Path('/proc',sys.argv[1],'cmdline').read_bytes().decode().split('\0')
project=Path(sys.argv[4])
defaults={'--config':project/'config/three_camera_bev.json',
          '--assets':project/'runtime/camera_bev_check_20261025'}
for flag,wanted in [('--config',sys.argv[2]),('--assets',sys.argv[3])]:
    actual=args[args.index(flag)+1] if flag in args else defaults[flag]
    if Path(actual).resolve()!=Path(wanted).resolve():
        raise SystemExit('正在运行其它 BEV 配置；先用 camera_bev.sh stop 结束相机预览，再切换。')
PY
      echo '当前 BEV 配置已运行'
      exit 0
    fi
    if systemctl --user is-active --quiet autolabor-conbine-camera-bev-check.service; then
      systemctl --user stop autolabor-conbine-camera-bev-check.service
    fi
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
    capture_args=()
    if [[ "${CAMERA_PREVIEW_HEADLESS:-0}" == 1 ]]; then capture_args+=(--headless); fi
    if /usr/bin/python3 -s -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("reuse_capture") else 1)' "$bev_config"; then
      bash "$project_root/scripts/four_camera_view.sh" start
      capture_args+=(--reuse-capture)
    fi
    systemd-run --user --unit="$unit" --collect --working-directory="$project_root" \
      --setenv="DISPLAY=$DISPLAY" --setenv="XAUTHORITY=$XAUTHORITY" \
      --setenv=PYTHONUTF8=1 \
      --property=KillMode=mixed --property=TimeoutStopSec=20 \
      --property="StandardOutput=append:$preview_root/bev_viewer.log" \
      --property="StandardError=append:$preview_root/bev_viewer.log" \
      /usr/bin/python3 -s "$project_root/scripts/three_camera_bev.py" \
      --config "$bev_config" --assets "$preview_root" --board-root "$board_root" "${capture_args[@]}"
    ;;
  check)
    /usr/bin/python3 -s "$project_root/scripts/three_camera_bev.py" --check \
      --config "$bev_config" --assets "$preview_root" --board-root "$board_root"
    ;;
  stop)
    if systemctl --user is-active --quiet "$unit"; then
      systemctl --user stop "$unit"
    fi
    ;;
  status)
    systemctl --user show "$unit" -p ActiveState -p MainPID
    if [[ -f "$preview_root/bev/bev.json" ]]; then
      /usr/bin/python3 -s - "$preview_root/bev/bev.json" <<'PY'
import json, sys, time
s=json.load(open(sys.argv[1]))
print(json.dumps(dict(age_s=(time.time_ns()-s['at_epoch_ns'])/1e9,
    sequences={n:m['sequence'] for n,m in s['sources'].items()},
    processing_ms=s['processing_ms'],metric_accuracy_verified=s['metric_accuracy_verified'],
    source_exposure_synchronized=s['source_exposure_synchronized']),ensure_ascii=False))
PY
    fi
    ;;
  *) echo "Usage: $0 {start|stop|status|check}" >&2; exit 2 ;;
esac
