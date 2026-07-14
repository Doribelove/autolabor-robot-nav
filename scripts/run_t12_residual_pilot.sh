#!/usr/bin/env bash
set -euo pipefail
W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_pilot"
SCENES="$W/experiments/manifests/t12/residual_pilot_scenes.yaml"
cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs" "$ROOT/failed_attempts"

wait_shutdown() {
  local end=$((SECONDS+30))
  while ((SECONDS<end)); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1; then return 0; fi
    sleep 1
  done
  return 1
}

valid() {
  "$W/.venv/bin/python" - "$1" "${2:-}" <<'PY'
import hashlib
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1])/"t11_run_report.yaml"
if not p.exists(): raise SystemExit(1)
r=yaml.safe_load(p.read_text())
ok=r.get("task")=="T12" and r.get("passed") is True and r.get("episode_count")==4
if ok and sys.argv[2] in ("legacy","directional","residual"):
 m=yaml.safe_load((p.parent/"run_manifest.yaml").read_text())
 expected=hashlib.sha256(Path('/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_shadow.yaml').read_bytes()).hexdigest()
 ok=m['configuration'].get('runtime_safety_override_sha256')==expected
if ok and sys.argv[2] == "residual":
 expected=hashlib.sha256(Path('/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_residual_semantic_eta.yaml').read_bytes()).hexdigest()
 ok=m['configuration'].get('residual_config_sha256')==expected
raise SystemExit(0 if ok else 1)
PY
}

run_one() {
  local name="$1" safety="$2" semantic="$3" seed="$4" eval_seed="$5"
  local src="$W/artifacts/t11/runs/t11_semantic_eta_fullsafety_seed${seed}"
  local checkpoint
  checkpoint=$("$W/.venv/bin/python" -c "import yaml; print(yaml.safe_load(open('$src/model_selection.yaml'))['selected_checkpoint'])")
  local id="t12p_${name}_seed${seed}" out="$ROOT/runs/t12p_${name}_seed${seed}"
  if valid "$out" "$name"; then echo "skip $id"; return; fi
  local attempt stamp
  for attempt in 1 2 3; do
    wait_shutdown
    if [[ -e "$out" ]]; then stamp=$(date +%Y%m%dT%H%M%S); mv "$out" "$ROOT/failed_attempts/${id}_${stamp}_a${attempt}"; fi
    echo "running $id attempt $attempt/3"
    local zero_action=false
    if [[ "$semantic" == "residual_pilot" ]]; then zero_action=true; fi
    if roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
      gazebo_seed:="$eval_seed" algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" \
      phase:=ablation safety_mode:="$safety" semantic_mode:="$semantic" \
      zero_action_policy:="$zero_action" \
      checkpoint:="$checkpoint" run_id:="$id" output_dir:="$out" \
      scene_manifest:="$SCENES" evaluation_seed_override:="$eval_seed" \
      >"$ROOT/logs/${id}_a${attempt}.log" 2>&1
    then
      if valid "$out" "$name"; then return; fi
    fi
  done
  return 1
}

for seed in 101 102; do
  e=$((500+seed))
  run_one legacy T12LegacySafety cumulative "$seed" "$e"
  run_one directional T12Safety cumulative "$seed" "$e"
  run_one residual T12Safety residual_pilot "$seed" "$e"
done

wait_shutdown
"$W/.venv/bin/python" - <<'PY'
import csv,glob,hashlib,yaml
from pathlib import Path
root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_pilot')
methods={}
for method in ('legacy','directional','residual'):
 rows=[]
 for p in sorted(root.glob('runs/t12p_{}_seed*/episodes.csv'.format(method))):
  rows.extend(csv.DictReader(p.open()))
 n=len(rows); reasons={name:sum(r['termination_reason']==name for r in rows) for name in ('goal','emergency_stop','collision','planner_failure','interface_fault')}
 logs='\n'.join(p.read_text(errors='replace') for p in sorted(root.glob('logs/t12p_{}_seed*_a1.log'.format(method))))
 methods[method]={'episode_count':n,**reasons,'success_rate':reasons['goal']/float(max(1,n)),
  'process_crash_count':logs.count('exit code -11'),
  'dynamic_reconfigure_failure_count':logs.count('dynamic_reconfigure service call failed')}
gates={
 'paired_24_episodes':all(v['episode_count']==8 for v in methods.values()),
 'directional_zero_collision':methods['directional']['collision']==0,
 'directional_zero_process_crash_or_interface_fault':methods['directional']['process_crash_count']==0 and methods['directional']['dynamic_reconfigure_failure_count']==0 and methods['directional']['interface_fault']==0,
 'directional_emergency_below_legacy':methods['directional']['emergency_stop']<methods['legacy']['emergency_stop'],
 'residual_pipeline_zero_collision':methods['residual']['collision']==0,
 'residual_pipeline_zero_process_crash_or_interface_fault':methods['residual']['process_crash_count']==0 and methods['residual']['dynamic_reconfigure_failure_count']==0 and methods['residual']['interface_fault']==0,
 'residual_navigation_abort_at_most_one':methods['residual']['planner_failure']<=1,
 'residual_pipeline_minimum_success':methods['residual']['success_rate']>=0.5,
}
report={'schema_version':'1.0','task':'T12','study':'residual_semantic_eta_no_training_pilot','training_performed':False,'methods':methods,'gates':gates,'passed':all(gates.values())}
report_path=root/'t12_residual_pilot_report.yaml'
report_path.write_text(yaml.safe_dump(report,sort_keys=False))
checksum_files=[report_path]+sorted(root.glob('runs/t12p_*_seed*/checksums.sha256'))
(root/'checksums.sha256').write_text(''.join(
 hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(root))+'\n'
 for p in checksum_files))
print(yaml.safe_dump(report,sort_keys=False))
raise SystemExit(0 if report['passed'] else 1)
PY
