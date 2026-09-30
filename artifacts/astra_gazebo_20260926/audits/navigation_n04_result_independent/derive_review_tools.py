#!/usr/bin/env python3
"""Derive read-only N04 replay tools from the already-used N03 audit.

This script never reads any N04 run. The generated auditors require a closed,
ledger-bound receipt before accessing a bag. Every output is under this audit.
"""
import ast,hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];old=S/'audits/navigation_n03_result_independent'
sources={}
def derive(name):
 p=old/name;t=p.read_text();sources[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest();return t
def replace(t,a,b):
 assert a in t,a[:120];return t.replace(a,b)
t=derive('audit_row.py').replace('N03','N04').replace('navigation_n03','navigation_n04')
t=replace(t,'import navigation_evidence_n02 as nav','import navigation_evidence_n04 as nav\nimport event_actor_contract_n04 as actor_event')
t=replace(t,'import dense_sweep_p04 as dense','import dense_sweep_n04 as dense')
t=replace(t,'import evaluate_episode as evaluator','import evaluate_episode_n04 as evaluator')
t=replace(t,"assert release_sha=='614991f459902e6f42b94cd968e5d885b4b6e601eee0dbdc84a316cf7c253048'","assert release_sha==ledger['release_sha256']\nplan=read(D/'planned_matrix.json'); planned=next(x for x in plan['rows'] if x['run_id']==args.run_id)")
t=replace(t,"checks['all560_release_resources_and_three_extras_real_CAS_bound']=len(release['resources'])==560 and", "checks['all_release_resources_and_three_extras_real_CAS_bound']=bool(release['resources']) and")
t=replace(t,"receipt['seed']==260926034", "receipt['seed']==planned['seed'] and receipt['method']==planned['method']")
t=replace(t,"q.ELF_ROLES,set(q.ELF_ROLES.values())-{'actor_binaries'}", "nav.ELF_ROLES,set(nav.ELF_ROLES.values())-({'actor_binaries'} if not actor_event.world_actors((R/'world.sdf').read_text()) else set())")
t=replace(t,"statuses=[];ingress=[];metadata=[];coordinator=", "statuses=[];ingress=[];metadata=[];actor_bag=[];coordinator=")
t=replace(t,"elif topic.startswith('/astra/navigation/'):","elif topic.startswith('/astra/actors/') and topic.endswith('/gate_status'):actor_bag.append(json.loads(m.data))\n  elif topic.startswith('/astra/navigation/'):")
t=replace(t,"integrity={'collector_complete'", "actor_audit=actor_event.audit_status(raw['actor_gate_status'],actor_event.world_actors((R/'world.sdf').read_text()),raw['gate'],end)\nchecks['shared_actor_gate_audit_exact_replay']=actor_audit==assessment['actor_gate_audit']\nactor_payloads={}\nfor name in actor_event.world_actors((R/'world.sdf').read_text()):\n a=[x for x in actor_bag if x.get('model_name')==name and start<=x['status_source_ns']<=end]\n b=[x for x in raw['actor_gate_status'] if x.get('model_name')==name and start<=x['status_source_ns']<=end]\n actor_payloads[name]={'bag_count':len(a),'raw_count':len(b),'formal_payloads_exact':a==b}\nchecks['actual_actor_bag_raw_formal_payloads_exact']=all(x['formal_payloads_exact'] for x in actor_payloads.values())\nintegrity={'collector_complete'")
t=replace(t,"checks['all_eight_integrity_four_safety_gates_exact']=", "integrity['shared_actor_gate_identity_timing']=actor_audit['pass_']\nif contract['method']=='frozen_r2_v060_sensor_lifecycle_v1':\n graph=[x for x in raw['node_roster'] if start<=x['source_ns']<=watermark]\n integrity['sensor_only_method_registrations']=bool(graph) and all(x.get('sensor_method_graph',{}).get('pass_') is True for x in graph)\nchecks['all_integrity_four_safety_gates_exact']=")
t=replace(t,"checks['native_images_lossless_exact_source']=bool(images)and all", "checks['native_images_lossless_exact_source']=all")
t=replace(t,"closure=dense.authenticate_closure", "checks['image_coverage_retained_separate_from_task_safety']=assessment['image_coverage']==raw['image_coverage'] and raw['image_coverage']['capture_integrity_dependency'] is False\nclosure=dense.authenticate_closure")
t=replace(t,"'native_images':images,", "'native_images':images,'image_coverage':raw['image_coverage'],'actor_gate_audit':actor_audit,'actor_bag_payloads':actor_payloads,")
t=replace(t,"'scope':'Single clear-fixture integration; single DWA config integration; no R2 row in this stage; no planner-decision or performance claim; headerless method Twist generation time is unknown.'", "'scope':'One of48 fixed development rows over6 reused parents. Physical/task/image negatives remain separate. No formal safety, generalization or performance superiority claim; headerless method Twist generation time is unknown.'")
ast.parse(t)
(D/'audit_row.py').write_text(t)
t=derive('replay_sweep.py').replace('navigation_n03','navigation_n04').replace('dense_sweep_p04.py','dense_sweep_n04.py')
ast.parse(t);(D/'replay_sweep.py').write_text(t)
report={'schema':'astra_n04_independent_audit_tool_derivation_v1','source_scripts':[{'path':p,'sha256':h} for p,h in sorted(sources.items())], 'generated_scripts':[{'path':str(D/n),'sha256':hashlib.sha256((D/n).read_bytes()).hexdigest()}for n in ('audit_row.py','replay_sweep.py')], 'N04_outcomes_read':False,'syntax_checks':'ast.parse of both generated tools passed','runtime_semantics_changes':['N04 module/library/identity and variable release resource count','actual event actor bag/raw and frozen status audit replay','optional R2 graph gate','zero native images retained as visual limitation, not physical integrity reclassification'],'execution':'not invoked; waiting for closed ledger-bound receipts'}
(D/'tool_derivation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
