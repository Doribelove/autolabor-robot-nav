#!/usr/bin/env bash
set -euo pipefail

W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_curriculum_fix_pilot"
CONFIG="$W/config/thesis_experiments/t12_residual_curriculum_fix_pilot.yaml"
cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs" "$ROOT/failed_attempts"

wait_shutdown() {
  local end=$((SECONDS + 30))
  while (( SECONDS < end )); do
    if ! pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1 \
       && ! ss -ltn 2>/dev/null | grep -q ':11311 '; then return 0; fi
    sleep 1
  done
  return 1
}

valid_run() {
  "$W/.venv/bin/python" - "$1" "$CONFIG" <<'PY'
import hashlib
from pathlib import Path
import sys,yaml
run=Path(sys.argv[1]); config=Path(sys.argv[2])
report=run/'t11_run_report.yaml'; selection=run/'model_selection.yaml'; manifest=run/'run_manifest.yaml'
if not all(path.is_file() for path in (report,selection,manifest)): raise SystemExit(1)
r=yaml.safe_load(report.read_text()); s=yaml.safe_load(selection.read_text()); m=yaml.safe_load(manifest.read_text())
expected=hashlib.sha256(config.read_bytes()).hexdigest()
ok=(r.get('task')=='T12' and r.get('passed') is True and
    s.get('selected_timesteps') in (1000,2000) and len(s.get('validation',()))==2 and
    m['configuration'].get('residual_config_sha256')==expected)
raise SystemExit(0 if ok else 1)
PY
}

for seed in 101 102; do
  id="t12_residual_curriculum_fix_seed${seed}"
  out="$ROOT/runs/$id"
  if valid_run "$out"; then echo "skip $id"; continue; fi
  for attempt in 1 2; do
    wait_shutdown
    if [[ -e "$out" ]]; then
      stamp=$(date +%Y%m%dT%H%M%S)
      mv "$out" "$ROOT/failed_attempts/${id}_${stamp}_a${attempt}"
    fi
    echo "running $id attempt $attempt/2"
    if roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
      gazebo_seed:="$seed" algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" \
      phase:=train safety_mode:=T12Safety semantic_mode:=residual_training \
      residual_config:="$CONFIG" acceptance_timesteps:=2000 \
      acceptance_eval_seed_limit:=1 run_id:="$id" output_dir:="$out" \
      >"$ROOT/logs/${id}_a${attempt}.log" 2>&1
    then
      if valid_run "$out"; then break; fi
    fi
    wait_shutdown || true
  done
  valid_run "$out"
done

wait_shutdown
"$W/.venv/bin/python" - <<'PY'
import csv,hashlib,yaml
from pathlib import Path
root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_curriculum_fix_pilot')
expected_scenes={
 't11-train-clear-straight','t11-train-clear-left','t11-train-clear-right',
 't11-train-obstacle','t11-train-corridor'}
runs={}
for seed in (101,102):
 run=root/'runs'/f't12_residual_curriculum_fix_seed{seed}'
 selection=yaml.safe_load((run/'model_selection.yaml').read_text())
 episodes=list(csv.DictReader((run/'episodes.csv').open()))
 train=[row for row in episodes if row['scene_split']=='train']
 test=[row for row in episodes if row['scene_split'] in ('test_id','test_ood')]
 values=[float(item['mean_return']) for item in selection['validation']]
 logs='\n'.join(path.read_text(errors='replace') for path in sorted(root.glob(f'logs/*seed{seed}_a*.log')))
 runs[seed]={
  'training_budget_steps':2000,
  'training_scene_ids':sorted({row['scene_id'] for row in train}),
  'training_scene_episode_counts':{scene:sum(row['scene_id']==scene for row in train) for scene in sorted(expected_scenes)},
  'validation_mean_returns':values,
  'validation_return_change':values[-1]-values[0],
  'selected_timesteps':selection['selected_timesteps'],
  'test_episode_count':len(test),
  'test_goal':sum(row['termination_reason']=='goal' for row in test),
  'test_collision':sum(row['termination_reason']=='collision' for row in test),
  'test_emergency_stop':sum(row['termination_reason']=='emergency_stop' for row in test),
  'test_interface_fault':sum(row['termination_reason']=='interface_fault' for row in test),
  'process_crash_count':logs.count('exit code -11'),
  'dynamic_reconfigure_failure_count':logs.count('dynamic_reconfigure service call failed'),
 }
gates={
 'two_seed_complete':len(runs)==2,
 'curriculum_scene_rotation_repaired':all(set(item['training_scene_ids'])==expected_scenes for item in runs.values()),
 'zero_collision':all(item['test_collision']==0 for item in runs.values()),
 'zero_interface_fault':all(item['test_interface_fault']==0 for item in runs.values()),
 'zero_process_crash':all(item['process_crash_count']==0 for item in runs.values()),
 'validation_improved_each_seed':all(item['validation_return_change']>0 for item in runs.values()),
}
integrity=all(value for name,value in gates.items() if name!='validation_improved_each_seed')
learning=gates['validation_improved_each_seed']
report={'schema_version':'1.0','task':'T12','study':'residual_curriculum_rotation_single_factor_pilot',
 'formal_result':False,'expanded_budget':False,'runs':runs,'gates':gates,
 'integrity_passed':integrity,'learning_gate_passed':learning,
 'decision':('paired_baseline_confirmation_then_budget_expansion_review' if integrity and learning else
             'stop_and_review_next_single_factor_without_budget_expansion')}
output=root/'t12_residual_curriculum_fix_pilot_report.yaml'
output.write_text(yaml.safe_dump(report,sort_keys=False))
files=[output]+sorted(root.glob('runs/*/checksums.sha256'))
(root/'checksums.sha256').write_text(''.join(
 hashlib.sha256(path.read_bytes()).hexdigest()+'  '+str(path.relative_to(root))+'\n' for path in files))
print(yaml.safe_dump(report,sort_keys=False))
raise SystemExit(0 if integrity else 1)
PY
