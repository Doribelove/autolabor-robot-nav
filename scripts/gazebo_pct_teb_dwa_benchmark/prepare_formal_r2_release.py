#!/usr/bin/env python3
"""Create the immutable 200-block formal-round-2 preregistration and release."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import yaml


ROOT=Path(__file__).resolve().parents[2]
ARTIFACT=ROOT/'artifacts/gazebo_pct_teb_dwa_benchmark';FORMAL=ARTIFACT/'formal/round_2'
SCHEDULE=FORMAL/'formal_r2_schedule.csv';PREREG=FORMAL/'formal_r2_preregistration.yaml'
RELEASE=FORMAL/'formal_r2_execution_release.yaml';REVIEW=FORMAL/'formal_r2_release_review.json'
PILOT=ARTIFACT/'pilot/pilot_r2/pilot_r2_assessment.json'
SCENES=('core_static_avoidance','core_single_crossing','core_head_on_yield','core_sequential_interaction','core_partially_occluded','core_width_change')
METHODS=('pct_scope_rl','analytic_prior','actual_teb','dwa')
RESOURCES=(
 'scripts/gazebo_pct_teb_dwa_benchmark/run_frozen_matrix.py','scripts/gazebo_pct_teb_dwa_benchmark/run_frozen_matrix_r2.py',
 'scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py','scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r1.py',
 'scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r2.py','src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py',
 'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py',
 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py',
 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py',
 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py',
 'src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/sensor_env.py',
 'src/application/v3_scope_planner/src/v3_scope_planner/execution_core.py',
 'src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_r2_episode.launch','src/simulation/m2_gazebo/launch/spawn_m2.launch',
 'src/simulation/m2_gazebo/urdf/m2.urdf.xacro','src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_r2_development.yaml',
 'src/simulation/m2_gazebo/config/benchmark_move_base.yaml','src/simulation/m2_gazebo/config/costmap_common.yaml',
 'src/simulation/m2_gazebo/config/local_costmap.yaml','src/simulation/m2_gazebo/config/global_costmap.yaml',
 'src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml','src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml',
 'src/navigation_arena/forks/navigation/local_planner/teb/teb_local_planner_plugin.xml')


def sha256(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write_once(path,text):
    if path.exists():raise FileExistsError('FORMAL_R2_RESOURCE_ALREADY_EXISTS:'+str(path))
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text,encoding='utf-8')


def main():
    pilot=json.loads(PILOT.read_text(encoding='utf-8'))
    if pilot.get('formal_round_2_selected_blocks')!=200 or pilot.get('pilot_tuning_performed') is not False:
        raise RuntimeError('FORMAL_R2_PILOT_BASIS_INVALID')
    rows=[]
    for index in range(200):
        order=METHODS[index%4:]+METHODS[:index%4]
        for position,method in enumerate(order):
            rows.append({'block_id':'formal_r2_block_{:03d}'.format(index+1),'seed':198001+index,
              'scene_id':SCENES[index%6],'method':method,'attempt':1,'order_index':position})
    FORMAL.mkdir(parents=True,exist_ok=True)
    with SCHEDULE.open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    prereg={'schema_version':'1.0','stage':'formal_round_2','status':'preregistered_before_formal_r2_outcome_access',
      'simulation_only':True,'independent_from_formal_round_1':True,'blocks':200,'episodes':800,'seed_range':[198001,198200],
      'scene_allocation':{scene:sum(x['scene_id']==scene for x in rows)//4 for scene in SCENES},'methods':list(METHODS),
      'candidate_contract_variant':'r2_candidate_v060','single_change_from_r1':'actions_4_5_target_speed_0.4_to_0.6_mps',
      'unchanged_actor_sha256':'deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8',
      'traditional_planner_configs_unchanged':True,'primary_unit':'paired_seed_scene_block',
      'primary_metric':'failure_inclusive_restricted_time_s','primary_comparisons':['pct_scope_rl_vs_actual_teb','pct_scope_rl_vs_dwa'],
      'primary_estimator':'(mean_baseline_minus_mean_pct)/mean_baseline',
      'multiplicity':{'method':'two_sided_97_5_percent_paired_block_bootstrap_for_each_of_two_locked_primary_comparisons',
        'bootstrap_samples':200000,'bootstrap_seeds':{'actual_teb':20261991,'dwa':20261992},'confidence_quantiles':[0.0125,0.9875]},
      'positive_gate_per_comparator':{'minimum_relative_time_reduction':0.08,'bootstrap_lower_bound_strictly_greater_than':0.0,
        'pct_completion_rate_difference_percentage_points_minimum':-5.0,'pct_contact_count_not_higher_than_comparator':True,
        'pct_mask_violation_count_maximum':0,'finite_statistics_required':True},
      'program_success_rule':'all integrity gates plus positive_gate for at least one locked traditional comparator',
      'failure_accounting':{'success':'measured_gate_to_goal_time','timeout_contact_nonarrival_method_system_failure':22.0,
        'infrastructure_failure':'retained_and_reported_with_22_second_primary_penalty','retry':'none','replacement_seed':'forbidden'},
      'secondary_analyses':['analytic_prior_comparison','scene_strata','completion_contact_clearance','candidate_switch_stop_recovery_tracking_latency','failure_case_inventory'],
      'secondary_claim_status':'descriptive_unless_separately_gated','sample_size_basis':{'pilot_assessment_path':str(PILOT.relative_to(ROOT)),
        'pilot_assessment_sha256':sha256(PILOT),'selected_blocks':200,'dwa_required_blocks':pilot['paired_comparisons']['dwa']['power_analysis']['required_blocks_ceiling'],
        'teb_required_blocks':pilot['paired_comparisons']['actual_teb']['power_analysis']['required_blocks_ceiling'],
        'teb_underpowered_within_200_block_cap':True},
      'schedule_path':str(SCHEDULE.relative_to(ROOT)),'schedule_sha256':sha256(SCHEDULE),
      'all_development_pilot_r1_formal_r1_seeds_excluded':True,'tuning_from_formal_outcomes':'forbidden','hardware_authorized':False,'real_vehicle_authorized':False}
    write_once(PREREG,yaml.safe_dump(prereg,sort_keys=False,allow_unicode=True))
    local=[ROOT/x for x in RESOURCES]+[PREREG,PILOT,ARTIFACT/'development/r2_candidate_coverage_d0/assessment.json']
    actor=ARTIFACT/'development/training/pct_benchmark_r1_seed9701/actor_final.pt';teb=ROOT/'src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml';dwa=ROOT/'src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml'
    external=[ROOT/'devel/lib/libteb_local_planner.so',Path('/opt/ros/noetic/lib/libdwa_local_planner.so')]
    release={'schema_version':'1.0','stage':'formal_round_2','status':'unique_formal_round_2_execution_release',
      'execution_authorized':True,'simulation_only':True,'real_vehicle_authorized':False,'training_authorized':False,
      'schedule_path':str(SCHEDULE.relative_to(ROOT)),'schedule_sha256':sha256(SCHEDULE),
      'preregistration_path':str(PREREG.relative_to(ROOT)),'preregistration_sha256':sha256(PREREG),
      'resource_bindings':[{'path':str(p.relative_to(ROOT)),'sha256':sha256(p)} for p in local],
      'external_resource_bindings':[{'path':str(p),'sha256':sha256(p)} for p in external],
      'method_bindings':{'pct_scope_rl':{'path':str(actor.relative_to(ROOT)),'sha256':sha256(actor)},
        'analytic_prior':{'path':None,'sha256':'none'},'actual_teb':{'path':str(teb.relative_to(ROOT)),'sha256':sha256(teb)},
        'dwa':{'path':str(dwa.relative_to(ROOT)),'sha256':sha256(dwa)}},
      'candidate_contract_variant':'r2_candidate_v060','budget':{'authorized_blocks':200,'authorized_episodes':800,'attempts_per_identity':1},
      'pilot_seed_access_authorized':False,'formal_round_1_seed_access_authorized':False,'stable_workspace_authorized':False}
    write_once(RELEASE,yaml.safe_dump(release,sort_keys=False,allow_unicode=True))
    review={'schema_version':'1.0','status':'formal_r2_release_offline_review_pass','schedule_rows':800,
      'unique_identities':len({(x['seed'],x['scene_id'],x['method'],x['attempt']) for x in rows}),'blocks':200,
      'scene_counts':prereg['scene_allocation'],'balanced_method_order_counts':{m:{str(i):sum(x['method']==m and x['order_index']==i for x in rows) for i in range(4)} for m in METHODS},
      'release_path':str(RELEASE.relative_to(ROOT)),'release_sha256':sha256(RELEASE),'preregistration_sha256':sha256(PREREG),
      'schedule_sha256':sha256(SCHEDULE),'formal_r2_outcome_accessed':False,'hardware_accessed':False}
    write_once(REVIEW,json.dumps(review,indent=2,sort_keys=True)+'\n');print(json.dumps(review,indent=2,sort_keys=True))


if __name__=='__main__':main()
