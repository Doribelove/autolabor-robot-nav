#!/usr/bin/env bash
set -euo pipefail

W=/home/robot/robot_ws_base_rl
ROOT="$W/artifacts/t12/residual_boundary_integrity_pilot"
CONFIG="$W/config/thesis_experiments/t12_residual_boundary_atomicity.yaml"
ID=t12_residual_boundary_integrity_seed101
OUT="$ROOT/runs/$ID"
LOG="$ROOT/logs/${ID}.log"

cd "$W"
source /opt/ros/noetic/setup.bash
source "$W/devel/setup.bash"
mkdir -p "$ROOT/runs" "$ROOT/logs"

if pgrep -f 't11_formal_run.py|/gazebo_ros/gzserver|/move_base/move_base' >/dev/null 2>&1 \
   || ss -ltn 2>/dev/null | grep -q ':11311 '; then
  echo "refusing to start: ROS/Gazebo process already active" >&2
  exit 1
fi
if [[ -e "$OUT" ]]; then
  echo "refusing to overwrite existing integrity run: $OUT" >&2
  exit 1
fi

set +e
roslaunch thesis_experiment t11_formal_run.launch gui:=false task:=T12 \
  gazebo_seed:=101 algorithm:=RL-TEB-Semantic-Eta training_seed:=101 \
  phase:=train safety_mode:=T12Safety semantic_mode:=residual_training \
  residual_config:="$CONFIG" initialize_residual_anchor:=true \
  acceptance_timesteps:=500 acceptance_eval_seed_limit:=1 \
  run_id:="$ID" output_dir:="$OUT" >"$LOG" 2>&1
launch_status=$?
set -e

"$W/.venv/bin/python" - "$launch_status" <<'PY'
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import yaml

root=Path('/home/robot/robot_ws_base_rl/artifacts/t12/residual_boundary_integrity_pilot')
run=root/'runs'/'t12_residual_boundary_integrity_seed101'
log_path=root/'logs'/'t12_residual_boundary_integrity_seed101.log'
config_path=Path('/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_residual_boundary_atomicity.yaml')
launch_status=int(sys.argv[1])
log=log_path.read_text(errors='replace') if log_path.is_file() else ''
required=(run/'t11_run_report.yaml',run/'run_manifest.yaml',run/'model_selection.yaml',run/'episodes.csv',run/'steps.csv')
complete=launch_status==0 and all(path.is_file() for path in required)
episodes=[] if not (run/'episodes.csv').is_file() else list(csv.DictReader((run/'episodes.csv').open()))
steps=[] if not (run/'steps.csv').is_file() else list(csv.DictReader((run/'steps.csv').open()))
manifest={} if not (run/'run_manifest.yaml').is_file() else yaml.safe_load((run/'run_manifest.yaml').read_text())
configuration=manifest.get('configuration', {})
audit=configuration.get('episode_boundary_audit') or {}
config=yaml.safe_load(config_path.read_text())
anchor=config['anchor_theta']
bounds=yaml.safe_load(Path('/home/robot/robot_ws_base_rl/src/application/teb_rl_tuner/config/t05_simulation_safety.yaml').read_text())['theta_bounds']
order=tuple(anchor)

episode_split={row['episode_id']:row['scene_split'] for row in episodes}
by_episode={}
for row in steps:
 by_episode.setdefault(row['episode_id'],[]).append(row)
first_l1=[]
for group in by_episode.values():
 first=min(group,key=lambda row:int(row['step_id']))
 previous=json.loads(first['theta_previous_json'])
 first_l1.append(sum(abs(2*(float(previous[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1 -
                         (2*(float(anchor[name])-float(bounds[name][0]))/(float(bounds[name][1])-float(bounds[name][0]))-1))
                     for name in order))
train_steps=[row for row in steps if episode_split.get(row['episode_id'])=='train']
projection_rate=(sum(row['projection_modified'].lower()=='true' for row in train_steps)/len(train_steps)
                 if train_steps else math.inf)
train_scenes=sorted({row['scene_id'] for row in episodes if row['scene_split']=='train'})
expected_scenes=sorted({'t11-train-clear-straight','t11-train-clear-left','t11-train-clear-right',
                        't11-train-obstacle','t11-train-corridor'})
test=[row for row in episodes if row['scene_split'] in ('test_id','test_ood')]
counts={
 'move_base_crash':log.count('move_base-5] process has died')+log.count('move_base-5 process has died'),
 'sigsegv':log.count('exit code -11'),
 'activation_timeout':sum(row.get('transition_drop_reason')=='parameter_activation_timeout' for row in steps),
 'interface_fault':sum(row.get('termination_reason')=='interface_fault' for row in episodes),
 'dynamic_reconfigure_failure':log.count('dynamic_reconfigure service call failed'),
 'collision':sum(row.get('termination_reason')=='collision' for row in test),
 'emergency_stop':sum(row.get('termination_reason')=='emergency_stop' for row in test),
 'test_goal':sum(row.get('termination_reason')=='goal' for row in test),
}
gates={
 'run_complete':complete,
 'atomic_boundary_protocol_recorded':audit.get('protocol')=='cancel_terminal_and_plan_quiet_restore_reset_dispatch_v1',
 'boundary_quiesce_failure_count_zero':audit.get('quiesce_failure_count')==0,
 'move_base_crash_count_zero':counts['move_base_crash']==0 and counts['sigsegv']==0,
 'activation_timeout_count_zero':counts['activation_timeout']==0,
 'interface_fault_count_zero':counts['interface_fault']==0,
 'dynamic_reconfigure_failure_count_zero':counts['dynamic_reconfigure_failure']==0,
 'all_episode_first_steps_match_anchor':bool(first_l1) and max(first_l1)<=1e-8,
 'all_five_training_scenes_observed':train_scenes==expected_scenes,
 'test_collision_count_zero':counts['collision']==0,
 'test_emergency_stop_count_zero':counts['emergency_stop']==0,
 'test_goal_count_seven':counts['test_goal']==7 and len(test)==7,
}
mechanism={
 'reference_projection_intervention_rate':0.8706,
 'observed_projection_intervention_rate':projection_rate,
 'drop_percentage_points':(0.8706-projection_rate)*100 if math.isfinite(projection_rate) else None,
 'maximum_rate':0.77,
 'projection_rate_materially_reduced':projection_rate<=0.77,
}
integrity=all(gates.values())
proceed=integrity and mechanism['projection_rate_materially_reduced']
report={
 'schema_version':'1.0','task':'T12','study':'residual_episode_boundary_integrity_pilot',
 'formal_result':False,'expanded_budget':False,'launch_exit_status':launch_status,
 'run_id':'t12_residual_boundary_integrity_seed101','training_seed':101,'training_timesteps':500,
 'configuration_sha256':hashlib.sha256(config_path.read_bytes()).hexdigest(),
 'boundary_audit':audit,'training_scene_ids':train_scenes,
 'episode_count':len(episodes),'step_count':len(steps),'train_step_count':len(train_steps),
 'first_step_previous_to_anchor_normalized_l1':{
   'episode_count':len(first_l1),'maximum':max(first_l1) if first_l1 else None,
   'mean':sum(first_l1)/len(first_l1) if first_l1 else None},
 'event_counts':counts,'gates':gates,'mechanism_gate':mechanism,
 'integrity_passed':integrity,'proceed_to_two_seed_pilot':proceed,
 'decision':('run_two_seed_2000_step_pilot' if proceed else
             'stop_without_budget_expansion_and_review_offline')}
output=root/'t12_residual_boundary_integrity_report.yaml'
output.write_text(yaml.safe_dump(report,sort_keys=False))
files=[output,log_path]+[path for path in required if path.is_file()]
(root/'checksums.sha256').write_text(''.join(
 hashlib.sha256(path.read_bytes()).hexdigest()+'  '+str(path.relative_to(root))+'\n' for path in files))
print(yaml.safe_dump(report,sort_keys=False))
raise SystemExit(0 if proceed else 1)
PY
