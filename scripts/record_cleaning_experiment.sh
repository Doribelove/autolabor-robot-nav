#!/usr/bin/env bash
# Full ROS recording plus the HTTP images actually consumed by the operator Qt.
set -euo pipefail
record_ws=$(cd -- "$(dirname -- "$0")/.." && pwd)
record_unit=autolabor-conbine-cleaning-visual-record.service
record_current="$record_ws/runtime/localization_recording/current"
case "${1:-status}" in
  start)
    bash "$record_ws/scripts/record_localization_test.sh" start
    if [[ -n "${2:-}" ]]; then
      python3 - "$record_ws" "$2" <<'PY'
import json,pathlib,sys
ws=pathlib.Path(sys.argv[1]);name=sys.argv[2]
if name in ('.','..') or '/' in name or not name.strip():raise ValueError('Expected an experiment name, not a path')
root=(ws/'runtime/localization_recording/current').resolve()
alias=ws/'rosbags'/name
if alias.exists() or alias.is_symlink():
    if alias.resolve()!=root:raise RuntimeError('Experiment name already belongs to another recording: '+name)
else:alias.symlink_to(pathlib.Path('localization')/root.name,target_is_directory=True)
p=root/'recording.json';d=json.loads(p.read_text())
d.update(experiment_name=name,named_path=str(alias),recording_state='recording')
p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
PY
    fi
    if systemctl --user is-active --quiet "$record_unit"; then
      exec bash "$record_ws/scripts/record_cleaning_experiment.sh" status
    fi
    record_dir=$(readlink -f "$record_current")
    record_take="$record_dir/visual/take_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$record_take" "$record_dir/assets"
    ln -sfn "$record_take" "$record_dir/visual/current"
    cp "$record_ws/config/four_camera_bev.json" "$record_ws/config/bev_detection_masks.json" \
       "$record_ws/config/camera_connections.json" "$record_dir/assets/"
    cp "$record_ws/src/application/autolabor_fod_control/config/bev_cleaning.yaml" "$record_dir/assets/"
    cp "$record_ws/src/application/autolabor_operator_gui/config/operator_navigation.rviz" "$record_dir/assets/"
    cp "$record_ws/runtime/bev_cleaning/latest_result.json" "$record_dir/assets/detector_at_start.json"
    systemd-run --user --unit="$record_unit" --collect --working-directory="$record_ws" \
      --setenv="DISPLAY=${DISPLAY:-:0}" --setenv="XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" \
      --property=KillMode=mixed --property=TimeoutStopSec=45 \
      --property="StandardOutput=append:$record_take/recorder.log" \
      --property="StandardError=append:$record_take/recorder.log" \
      /bin/bash "$record_ws/scripts/record_cleaning_experiment.sh" run "$record_take"
    printf 'Full cleaning recording: %s\n' "$record_dir"
    ;;
  run)
    record_dir="$2"
    set +u
    source /home/slam/robot_ws/.deps/setup.bash
    source "$record_ws/devel/setup.bash"
    set -u
    exec /usr/bin/python3 -B "$record_ws/scripts/record_cleaning_visuals.py" --output "$record_dir"
    ;;
  stop)
    if systemctl --user is-active --quiet "$record_unit"; then systemctl --user stop "$record_unit"; fi
    bash "$record_ws/scripts/record_localization_test.sh" stop
    python3 - "$record_current" <<'PY'
import json,pathlib,shutil,sys
out=pathlib.Path(sys.argv[1]).resolve()
session=json.loads((out/'session.json').read_text())
logs=out/'session_logs';logs.mkdir(exist_ok=True)
for name in ('qt.log','gateway.log','bev_cleaner.log','online_map.log','online_cloud.log'):
    src=pathlib.Path(session['local'])/name
    if src.exists():shutil.copy2(src,logs/name)
if session.get('online_coverage'):
    source=pathlib.Path(session['local_map_directory'])
    for name in ('map_fused_2d','observed_map','flat_region_prior.json','status.json','map.pcd'):
        src=source/name;dest=out/'online_map_at_recording_end'/name
        if src.is_dir():shutil.copytree(src,dest,dirs_exist_ok=True)
        elif src.is_file():
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest)
print('Recording finalized:',out)
PY
    ;;
  status)
    bash "$record_ws/scripts/record_localization_test.sh" status
    systemctl --user show "$record_unit" -p ActiveState -p MainPID
    if [[ -f "$record_current/visual/current/status.json" ]]; then cat "$record_current/visual/current/status.json"; fi
    ;;
  *) echo 'Usage: record_cleaning_experiment.sh {start [name]|stop|status}' >&2; exit 2 ;;
esac
