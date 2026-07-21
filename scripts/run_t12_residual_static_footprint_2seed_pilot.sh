#!/usr/bin/env bash
set -euo pipefail

W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_static_footprint_2seed_pilot"
CONFIG="$W/config/thesis_experiments/t12_residual_boundary_atomicity.yaml"
cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs"

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
required=(run/'t11_run_report.yaml',run/'model_selection.yaml',run/'run_manifest.yaml',run/'episodes.csv',run/'steps.csv')
if not all(path.is_file() for path in required): raise SystemExit(1)
r=yaml.safe_load(required[0].read_text()); s=yaml.safe_load(required[1].read_text()); m=yaml.safe_load(required[2].read_text())
c=m.get('configuration',{}); audit=c.get('episode_boundary_audit') or {}
expected=hashlib.sha256(config.read_bytes()).hexdigest()
ok=(r.get('task')=='T12' and r.get('passed') is True and
    s.get('selected_timesteps') in (1000,2000) and len(s.get('validation',()))==2 and
    c.get('residual_config_sha256')==expected and c.get('initialize_residual_anchor') is True and
    audit.get('protocol')=='cancel_terminal_and_plan_quiet_restore_reset_dispatch_v1' and
    audit.get('quiesce_failure_count')==0)
raise SystemExit(0 if ok else 1)
PY
}

for seed in 101 102; do
  id="t12_residual_static_footprint_seed${seed}"
  out="$ROOT/runs/$id"
  log="$ROOT/logs/${id}.log"
  wait_shutdown
  if [[ -e "$out" || -e "$log" ]]; then
    echo "refusing to overwrite existing run: $id" >&2
    exit 1
  fi
  echo "running $id"
  roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
    gazebo_seed:="$seed" algorithm:=RL-TEB-Semantic-Eta training_seed:="$seed" \
    phase:=train safety_mode:=T12Safety semantic_mode:=residual_training \
    residual_config:="$CONFIG" initialize_residual_anchor:=true \
    acceptance_timesteps:=2000 acceptance_eval_seed_limit:=1 \
    run_id:="$id" output_dir:="$out" >"$log" 2>&1
  valid_run "$out"
done

wait_shutdown
"$W/.venv/bin/python" - <<'PY'
import csv,hashlib,json,math,yaml
from pathlib import Path
root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_static_footprint_2seed_pilot')
config_path=Path('/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_residual_boundary_atomicity.yaml')
config=yaml.safe_load(config_path.read_text())
anchor=config['anchor_theta']
bounds=yaml.safe_load(Path('/home/robot/robot_ws_base_rl/src/application/teb_rl_tuner/config/t05_simulation_safety.yaml').read_text())['theta_bounds']
expected_scenes={'t11-train-clear-straight','t11-train-clear-left','t11-train-clear-right','t11-train-obstacle','t11-train-corridor'}
runs={}
for seed in (101,102):
 run=root/'runs'/f't12_residual_static_footprint_seed{seed}'
 log_path=root/'logs'/f't12_residual_static_footprint_seed{seed}.log'
 log=log_path.read_text(errors='replace')
 episodes=list(csv.DictReader((run/'episodes.csv').open()))
 steps=list(csv.DictReader((run/'steps.csv').open()))
 selection=yaml.safe_load((run/'model_selection.yaml').read_text())
 manifest=yaml.safe_load((run/'run_manifest.yaml').read_text())
 split={row['episode_id']:row['scene_split'] for row in episodes}
 grouped={}
 for row in steps: grouped.setdefault(row['episode_id'],[]).append(row)
 first_l1=[]
 for group in grouped.values():
  previous=json.loads(min(group,key=lambda row:int(row['step_id']))['theta_previous_json'])
  first_l1.append(sum(abs(2*(float(previous[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1 -
                          (2*(float(anchor[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1))
                      for name in anchor))
 train_steps=[row for row in steps if split.get(row['episode_id'])=='train']
 train_episodes=[row for row in episodes if row['scene_split']=='train']
 test=[row for row in episodes if row['scene_split'] in ('test_id','test_ood')]
 values=[float(item['mean_return']) for item in selection['validation']]
 audit=manifest['configuration']['episode_boundary_audit']
 runs[seed]={
  'training_budget_steps':2000,
  'training_scene_ids':sorted({row['scene_id'] for row in train_episodes}),
  'training_scene_episode_counts':{scene:sum(row['scene_id']==scene for row in train_episodes) for scene in sorted(expected_scenes)},
  'validation_mean_returns':values,
  'validation_return_change':values[-1]-values[0],
  'selected_timesteps':selection['selected_timesteps'],
  'projection_intervention_rate':sum(row['projection_modified'].lower()=='true' for row in train_steps)/len(train_steps),
  'safety_intervention_rate':sum(row['safety_modified'].lower()=='true' for row in train_steps)/len(train_steps),
  'first_step_previous_to_anchor_normalized_l1_max':max(first_l1),
  'boundary_audit':audit,
  'test_episode_count':len(test),
  'test_goal':sum(row['termination_reason']=='goal' for row in test),
  'test_collision':sum(row['termination_reason']=='collision' for row in test),
  'test_emergency_stop':sum(row['termination_reason']=='emergency_stop' for row in test),
  'test_interface_fault':sum(row['termination_reason']=='interface_fault' for row in test),
  'move_base_crash_count':log.count('move_base-5] process has died')+log.count('move_base-5 process has died'),
  'sigsegv_count':log.count('exit code -11'),
  'footprint_model_load_count':log.count("Footprint model 'polygon' loaded for trajectory optimization."),
  'dynamic_reconfigure_failure_mentions':log.count('dynamic_reconfigure service call failed'),
  'activation_timeout_count':sum(row['transition_drop_reason']=='parameter_activation_timeout' for row in steps),
 }
gates={
 'two_seed_complete':len(runs)==2,
 'atomic_boundary_active':all(item['boundary_audit']['protocol']=='cancel_terminal_and_plan_quiet_restore_reset_dispatch_v1' for item in runs.values()),
 'zero_boundary_quiesce_failure':all(item['boundary_audit']['quiesce_failure_count']==0 for item in runs.values()),
 'recovery_barrier_protocol_recorded':all(item['boundary_audit']['recovery_quiet_period_s']==1.0 for item in runs.values()),
 'static_footprint_loaded_once_each_seed':all(item['footprint_model_load_count']==1 for item in runs.values()),
 'all_episode_first_steps_match_anchor':all(item['first_step_previous_to_anchor_normalized_l1_max']<=1e-8 for item in runs.values()),
 'all_five_training_scenes_each_seed':all(set(item['training_scene_ids'])==expected_scenes for item in runs.values()),
 'zero_move_base_crash':all(item['move_base_crash_count']==0 and item['sigsegv_count']==0 for item in runs.values()),
 'zero_dynamic_reconfigure_failure':all(item['dynamic_reconfigure_failure_mentions']==0 for item in runs.values()),
 'zero_activation_timeout':all(item['activation_timeout_count']==0 for item in runs.values()),
 'zero_test_collision':all(item['test_collision']==0 for item in runs.values()),
 'zero_test_emergency_stop':all(item['test_emergency_stop']==0 for item in runs.values()),
 'zero_test_interface_fault':all(item['test_interface_fault']==0 for item in runs.values()),
 'test_goal_seven_each_seed':all(item['test_goal']==7 and item['test_episode_count']==7 for item in runs.values()),
 'validation_improved_each_seed':all(item['validation_return_change']>0 for item in runs.values()),
}
integrity=all(value for name,value in gates.items() if name!='validation_improved_each_seed')
learning=gates['validation_improved_each_seed']
report={'schema_version':'1.0','task':'T12','study':'residual_static_footprint_two_seed_pilot',
 'formal_result':False,'expanded_budget':False,'runs':runs,'gates':gates,
 'integrity_passed':integrity,'learning_gate_passed':learning,
 'proceed_to_frozen_pairing':integrity and learning,
 'decision':('run_frozen_three_method_pairing' if integrity and learning else
             'stop_and_review_residual_action_projection_alignment_without_budget_expansion')}
output=root/'t12_residual_static_footprint_2seed_report.yaml'
output.write_text(yaml.safe_dump(report,sort_keys=False))
files=[output]+sorted(root.glob('runs/*/checksums.sha256'))+sorted(root.glob('logs/*.log'))
(root/'checksums.sha256').write_text(''.join(hashlib.sha256(path.read_bytes()).hexdigest()+'  '+str(path.relative_to(root))+'\n' for path in files))
print(yaml.safe_dump(report,sort_keys=False))
raise SystemExit(0 if integrity else 1)
PY

