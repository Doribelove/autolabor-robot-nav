#!/usr/bin/env python3
"""Offline actual saved bag contact replay; no ROS node or service."""
import json,sys,hashlib
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/passive_sweep_q04_static'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import evaluate_episode as e
import passive_sweep_q04 as q
raw=json.loads((R/'collector_raw.json').read_text());original=json.loads((R/'assessment.json').read_text());start,end=raw['start_ns'],raw['end_ns']
models,links,contacts,metadata=e.read_bag(R/'fixture.bag');world=e.WorldCollisions((R/'world.sdf').read_text());errors=[]
audit=e.contact_audit({r['stamp_ns']:r for r in contacts},world,start,end,'autolabor_m2',set(world.actor_speed_bounds),errors)
summary=dict(audit)
for key in ('illegal_robot_contacts','wheel_ground_support_contacts','actor_environment_contacts'):
 summary[key+'_count']=len(summary[key]);summary[key+'_examples']=summary[key][:10];del summary[key]
coverage=q.source_coverage([r['stamp_ns']for r in contacts],start,end)
checks={'contact_summary_exact':summary==original['contact_audit'],'contact_issues_exact':errors==original['contact_issues'],'source_coverage_complete':coverage['pass'],'all_collision_pairs_and_every_step_metadata':metadata.get('contacts_every_physics_step')is True and metadata.get('all_collision_pairs_retained')is True}
out={'checks':checks,'pass':all(checks.values()),'contact_summary':summary,'coverage':coverage,'interpretation':'Wheel-ground support is permitted. Zero recorded illegal contacts is not proof of collision freedom; old geometry/COM qualification remains failed.','script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
with (D/'contact_replay.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps(checks));assert all(checks.values())
