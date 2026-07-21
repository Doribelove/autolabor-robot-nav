#!/usr/bin/env bash
set -euo pipefail
W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_training"
cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs" "$ROOT/failed_attempts"

wait_shutdown() {
  local end=$((SECONDS+45))
  while ((SECONDS<end)); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1; then return 0; fi
    sleep 1
  done
  return 1
}

valid() {
  "$W/.venv/bin/python" - "$1" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1])
r=p/'t11_run_report.yaml'; s=p/'model_selection.yaml'
if not r.exists() or not s.exists(): raise SystemExit(1)
report=yaml.safe_load(r.read_text()); selection=yaml.safe_load(s.read_text())
ok=(report.get('task')=='T12' and report.get('passed') is True and
    report.get('safety_mode')=='T12Safety' and len(selection.get('validation',[]))==2 and
    selection.get('test_episode_count')==7 and selection.get('test_results_used_for_selection') is False)
raise SystemExit(0 if ok else 1)
PY
}

for seed in 101 102; do
  id="t12_residual_training_seed${seed}"; out="$ROOT/runs/$id"
  if valid "$out"; then echo "skip validated $id"; continue; fi
  for attempt in 1 2; do
    wait_shutdown
    if [[ -e "$out" ]]; then mv "$out" "$ROOT/failed_attempts/${id}_$(date +%Y%m%dT%H%M%S)_a${attempt}"; fi
    echo "training $id attempt $attempt/2"
    if roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
      gazebo_seed:="$seed" algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" \
      phase:=train safety_mode:=T12Safety semantic_mode:=residual_training \
      residual_config:="$W/config/thesis_experiments/t12_residual_training.yaml" \
      acceptance_timesteps:=2000 acceptance_eval_seed_limit:=1 \
      run_id:="$id" output_dir:="$out" \
      >"$ROOT/logs/${id}_a${attempt}.log" 2>&1
    then
      if valid "$out"; then break; fi
    fi
    if ((attempt==2)); then exit 1; fi
  done
done

wait_shutdown
"$W/.venv/bin/python" - <<'PY'
import csv,hashlib,yaml
from pathlib import Path
root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_training')
runs={}
for seed in (101,102):
 p=root/'runs'/f't12_residual_training_seed{seed}'
 rows=list(csv.DictReader((p/'episodes.csv').open()))
 test=[r for r in rows if r['scene_split'] in ('test_id','test_ood')]
 sel=yaml.safe_load((p/'model_selection.yaml').read_text())
 reasons={k:sum(r['termination_reason']==k for r in test) for k in ('goal','collision','emergency_stop','planner_failure','interface_fault')}
 vals=[float(v['mean_return']) for v in sel['validation']]
 log=(root/'logs'/f't12_residual_training_seed{seed}_a1.log').read_text(errors='replace')
 runs[seed]={'training_budget_steps':2000,'validation_mean_returns':vals,
  'validation_return_change':vals[-1]-vals[0],'selected_timesteps':sel['selected_timesteps'],
  'test_episode_count':len(test),'test_success_rate':reasons['goal']/len(test),**reasons,
  'process_crash_count':log.count('exit code -11'),'dynamic_reconfigure_failure_count':log.count('dynamic_reconfigure service call failed')}
gates={
 'two_seed_complete':all(v['test_episode_count']==7 for v in runs.values()),
 'zero_collision':sum(v['collision'] for v in runs.values())==0,
 'zero_interface_fault':sum(v['interface_fault'] for v in runs.values())==0,
 'zero_process_crash':sum(v['process_crash_count'] for v in runs.values())==0,
 'zero_dynamic_reconfigure_failure':sum(v['dynamic_reconfigure_failure_count'] for v in runs.values())==0,
 'both_have_finite_validation':all(len(v['validation_mean_returns'])==2 for v in runs.values()),
}
report={'schema_version':'1.0','task':'T12','study':'residual_sac_2seed_small_budget',
 'formal_result':False,'runs':runs,'gates':gates,'passed':all(gates.values()),
 'expansion_recommendation':'pending_learning_trend_review'}
rp=root/'t12_residual_training_report.yaml'; rp.write_text(yaml.safe_dump(report,sort_keys=False))
files=[rp]+sorted(root.glob('runs/*/checksums.sha256'))
(root/'checksums.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(root))+'\n' for p in files))
print(yaml.safe_dump(report,sort_keys=False))
raise SystemExit(0 if report['passed'] else 1)
PY
