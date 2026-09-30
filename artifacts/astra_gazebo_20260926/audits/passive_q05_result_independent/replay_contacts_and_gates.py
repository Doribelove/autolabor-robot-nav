#!/usr/bin/env python3
"""Saved Q05 bags: independent contact/motion reduction and 13 frozen gate replay."""
import argparse,hashlib,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import evaluate_episode as e
import dense_sweep_p04 as d
import passive_sweep_q05 as q
parser=argparse.ArgumentParser();parser.add_argument('--run-id',required=True,choices=['passive_sweep_q05_static','passive_sweep_q05_actor']);args=parser.parse_args()
R=S/'runs'/args.run_id;O=D/'rows'/args.run_id
def read(p):return json.loads(p.read_bytes())
raw=read(R/'collector_raw.json');original=read(R/'assessment.json');contract=read(R/'contract.json');start,end=raw['start_ns'],raw['end_ns']
assert any(x.get('receipt',{}).get('sha256')==hashlib.sha256((R/'receipt.json').read_bytes()).hexdigest() for x in read(S/'queue/passive_sweep_q05_ledger.json')['units']if x['run_id']==args.run_id)
models,links,contacts,metadata=e.read_bag(R/'fixture.bag');world=e.WorldCollisions((R/'world.sdf').read_text());errors=[]
audit=e.contact_audit({r['stamp_ns']:r for r in contacts},world,start,end,'autolabor_m2',set(world.actor_speed_bounds),errors)
summary=dict(audit)
for key in ('illegal_robot_contacts','wheel_ground_support_contacts','actor_environment_contacts'):
 summary[key+'_count']=len(summary[key]);summary[key+'_examples']=summary[key][:10];del summary[key]
coverage=q.source_coverage([r['stamp_ns']for r in contacts],start,end)
records=d.read_p03_bag(R/'fixture.bag');states=[r for r in records['end_model_rows']if start<=r['stamp_ns']<=end]
poses={name:[r['poses'][r['names'].index(name)][:3]for r in states]for name in ['autolabor_m2']+(['q05_actor']if contract['actor']else [])}
distance=lambda rows:max((sum((b-a)**2 for a,b in zip(rows[0],p))**.5 for p in rows),default=0.)if rows else 0.
robot_distance=distance(poses['autolabor_m2']);actor_distance=distance(poses.get('q05_actor',[]))
command=q.command_audit(raw['command_events'],contract,start);status=q.physical_status_audit(raw['status_messages'],start,end);sweep=read(O/'sweep_replay/report.json')
checks13={'collector_pass':read(R/'collector_report.json')['pass'],
 'passive_source_geometry_pass':sweep['integrity_pass'],'positive_engineering_clearance':sweep['clearance_status']=='pass_within_model_scope',
 'contact_packet_coverage':coverage['pass'],'contact_metadata_complete':metadata.get('contacts_every_physics_step')is True and metadata.get('all_collision_pairs_retained')is True,
 'no_illegal_robot_contact':not errors and not audit['illegal_robot_contacts'],'no_actor_environment_contact':not audit['actor_environment_contacts'],
 'exact_publisher_clock_command_slots':command['pass'],'robot_motion_observed':robot_distance>.1,
 'actor_motion_observed_or_not_applicable':actor_distance>.25 if contract['actor']else True,
 'physical_status_coverage':status['coverage']['pass'],'physical_status_message_contract':status['message_contract_pass'],'no_physical_backend_fault':status['no_fault_timeout_brake_emergency']}
checks={'contact_summary_exact':summary==original['contact_audit'],'contact_issues_exact':errors==original['contact_issues'],
 'source_coverage_complete':coverage['pass'],'all_collision_pairs_and_every_step_metadata':checks13['contact_metadata_complete'],
 'actual_robot_and_actor_motion_exact':robot_distance==original['robot_maximum_displacement_m']and actor_distance==original['actor_maximum_displacement_m'],
 'exact_all_13_gate_roster_and_values':checks13==original['checks']and len(checks13)==13,
 'original_result_exact':all(checks13.values())==original['pass'],
 'command_and_status_report_exact':command==original['command_audit']and status==original['physical_status_audit']}
out={'checks':checks,'pass':all(checks.values()),'checks13_replayed':checks13,'contact_summary':summary,'coverage':coverage,
 'actual_motion_m':{'robot':robot_distance,'actor':actor_distance},'formal_native_model_samples':len(states),
 'interpretation':'Wheel-ground support is permitted. Zero recorded illegal contacts is not proof of collision freedom. Native model displacement is descriptive integration evidence, not navigation performance.',
 'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
with (O/'contact_and_gate_replay.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps({'pass':out['pass'],'checks':checks,'contacts':{k:v for k,v in summary.items()if k.endswith('_count')},'motion':out['actual_motion_m']}));assert out['pass']
