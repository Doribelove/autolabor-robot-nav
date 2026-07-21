#!/usr/bin/env bash
set -euo pipefail

W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_static_footprint_reconfigure_stress"
CONFIG="$W/config/thesis_experiments/t12_residual_boundary_atomicity.yaml"
ID=t12_residual_static_footprint_seed101
OUT="$ROOT/runs/$ID"
LOG="$ROOT/logs/${ID}.log"

cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs"
if pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1 \
   || ss -ltn 2>/dev/null | grep -q ':11311 '; then
  echo "refusing to start: ROS/Gazebo process already active" >&2; exit 1
fi
if [[ -e "$OUT" || -e "$LOG" ]]; then
  echo "refusing to overwrite stress pilot" >&2; exit 1
fi

set +e
roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
  gazebo_seed:=101 algorithm:=RL-TEB-Semantic-Eta training_seed:=101 \
  phase:=train safety_mode:=T12Safety semantic_mode:=residual_training \
  residual_config:="$CONFIG" initialize_residual_anchor:=true \
  acceptance_timesteps:=1300 acceptance_eval_seed_limit:=1 \
  run_id:="$ID" output_dir:="$OUT" >"$LOG" 2>&1
launch_status=$?
set -e

"$W/.venv/bin/python" - "$launch_status" <<'PY'
import csv,hashlib,json,math,sys,yaml
from pathlib import Path
root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_static_footprint_reconfigure_stress')
run=root/'runs'/'t12_residual_static_footprint_seed101'
log_path=root/'logs'/'t12_residual_static_footprint_seed101.log'
config_path=Path('/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_residual_boundary_atomicity.yaml')
log=log_path.read_text(errors='replace') if log_path.is_file() else ''
required=(run/'t11_run_report.yaml',run/'run_manifest.yaml',run/'model_selection.yaml',run/'episodes.csv',run/'steps.csv')
complete=int(sys.argv[1])==0 and all(path.is_file() for path in required)
episodes=[] if not (run/'episodes.csv').is_file() else list(csv.DictReader((run/'episodes.csv').open()))
steps=[] if not (run/'steps.csv').is_file() else list(csv.DictReader((run/'steps.csv').open()))
manifest={} if not (run/'run_manifest.yaml').is_file() else yaml.safe_load((run/'run_manifest.yaml').read_text())
selection={} if not (run/'model_selection.yaml').is_file() else yaml.safe_load((run/'model_selection.yaml').read_text())
audit=manifest.get('configuration',{}).get('episode_boundary_audit') or {}
config=yaml.safe_load(config_path.read_text()); anchor=config['anchor_theta']
bounds=yaml.safe_load(Path('/home/robot/robot_ws_base_rl/src/application/teb_rl_tuner/config/t05_simulation_safety.yaml').read_text())['theta_bounds']
split={row['episode_id']:row['scene_split'] for row in episodes}; grouped={}
for row in steps: grouped.setdefault(row['episode_id'],[]).append(row)
first_l1=[]
for group in grouped.values():
 previous=json.loads(min(group,key=lambda row:int(row['step_id']))['theta_previous_json'])
 first_l1.append(sum(abs(2*(float(previous[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1-
                         (2*(float(anchor[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1)) for name in anchor))
train_steps=[row for row in steps if split.get(row['episode_id'])=='train']
train_scenes=sorted({row['scene_id'] for row in episodes if row['scene_split']=='train'})
expected=sorted({'t11-train-clear-straight','t11-train-clear-left','t11-train-clear-right','t11-train-obstacle','t11-train-corridor'})
test=[row for row in episodes if row['scene_split'] in ('test_id','test_ood')]
counts={
 'move_base_crash':log.count('move_base-5] process has died')+log.count('move_base-5 process has died'),
 'sigsegv':log.count('exit code -11'),
 'footprint_model_load':log.count("Footprint model 'polygon' loaded for trajectory optimization."),
 'activation_timeout':sum(row.get('transition_drop_reason')=='parameter_activation_timeout' for row in steps),
 'interface_fault':sum(row.get('termination_reason')=='interface_fault' for row in episodes),
 'dynamic_reconfigure_failure':log.count('dynamic_reconfigure service call failed'),
 'collision':sum(row.get('termination_reason')=='collision' for row in test),
 'emergency_stop':sum(row.get('termination_reason')=='emergency_stop' for row in test),
 'test_goal':sum(row.get('termination_reason')=='goal' for row in test),
}
values=[float(item['mean_return']) for item in selection.get('validation',[])]
projection=(sum(row['projection_modified'].lower()=='true' for row in train_steps)/len(train_steps) if train_steps else math.inf)
gates={
 'run_complete':complete,
 'recovery_barrier_protocol_recorded':audit.get('recovery_quiet_period_s')==1.0,
 'static_footprint_loaded_once':counts['footprint_model_load']==1,
 'boundary_quiesce_failure_count_zero':audit.get('quiesce_failure_count')==0,
 'move_base_crash_count_zero':counts['move_base_crash']==0 and counts['sigsegv']==0,
 'activation_timeout_count_zero':counts['activation_timeout']==0,
 'interface_fault_count_zero':counts['interface_fault']==0,
 'dynamic_reconfigure_failure_count_zero':counts['dynamic_reconfigure_failure']==0,
 'all_episode_first_steps_match_anchor':bool(first_l1) and max(first_l1)<=1e-8,
 'all_five_training_scenes_observed':train_scenes==expected,
 'test_collision_count_zero':counts['collision']==0,
 'test_emergency_stop_count_zero':counts['emergency_stop']==0,
 'test_goal_count_seven':counts['test_goal']==7 and len(test)==7,
}
passed=all(gates.values())
report={'schema_version':'1.0','task':'T12','study':'residual_static_footprint_reconfigure_stress',
 'formal_result':False,'expanded_budget':False,'launch_exit_status':int(sys.argv[1]),
 'training_seed':101,'training_timesteps':1300,'boundary_audit':audit,
 'training_scene_ids':train_scenes,'episode_count':len(episodes),'step_count':len(steps),
 'first_step_previous_to_anchor_normalized_l1_max':max(first_l1) if first_l1 else None,
 'projection_intervention_rate':projection,'validation_mean_returns':values,
 'event_counts':counts,'gates':gates,'integrity_passed':passed,
 'proceed_to_full_two_seed':passed,
 'decision':('run_full_two_seed_2000_step_pilot' if passed else 'stop_without_baseline_pairing')}
out=root/'t12_residual_static_footprint_reconfigure_stress_report.yaml'; out.write_text(yaml.safe_dump(report,sort_keys=False))
files=[out,log_path]+[path for path in required if path.is_file()]
(root/'checksums.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(root))+'\n' for p in files))
print(yaml.safe_dump(report,sort_keys=False)); raise SystemExit(0 if passed else 1)
PY
