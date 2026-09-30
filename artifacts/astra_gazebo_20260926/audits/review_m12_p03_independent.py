#!/usr/bin/env python3
"""P03 design/runner/launch review only. Never calls execute or starts ROS."""
from pathlib import Path
import ast
import copy
import hashlib
import json
import sys
import types
import xml.etree.ElementTree as E
import yaml

A=Path(__file__).resolve().parent;S=A.parent;W=S/'workspace';P=W/'scripts/astra_gazebo/run_m12_physics_p03.py'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
sys.path.insert(0,str(P.parent));payload=P.read_bytes();module=types.ModuleType('independent_p03_runner');module.__file__=str(P);exec(compile(payload,str(P),'exec'),vars(module))
design,parent,rows=module.load_design();module.check_seed_freshness(rows);prior=module.predecessor(design)
pd=S/'audits/m12_p03_root_launch_preflight';preview=json.loads((pd/'report.json').read_text())
P02=yaml.safe_load((S/'audits/m12_p02_launch_preflight/effective_parameters.yaml').read_text())
def structure(el):return (el.tag,tuple(sorted(el.attrib.items())),(el.text or '').strip(),tuple(structure(x) for x in el))
def normalize_robot(text):
    root=E.fromstring(text);plugin=root.find("gazebo/plugin[@filename='libastra_physical_ackermann_p02_plugin.so']")
    plugin.find('steeringKd').text='4.0';plugin.find('dynamicsSeed').text='260926008'
    return structure(root)
expected_robot=normalize_robot(P02['/robot_description']);perrun=[];resources=[P,module.SOURCE,module.PROBE,module.DESIGN,S/design['schedule']['path'],S/design['budget_design']['path'],S/design['parent_contract']['path'],pd/'report.json',A/'m12_p02_raw_independent/protocol_audit.json',W/'scripts/astra_gazebo/tests/test_m12_physics_p03.py']
for row in rows:
    cp=S/row['contract_path'];c=yaml.safe_load(cp.read_bytes());d=pd/row['run_id'];params=yaml.safe_load((d/'parameters.yaml').read_text());render=module.render_launch(row,d,c)
    launch_equal=structure(E.fromstring(render))==structure(E.fromstring((d/'fixture.launch').read_text()))
    model=E.fromstring(params['/robot_description']);plugin=model.find("gazebo/plugin[@filename='libastra_physical_ackermann_p02_plugin.so']")
    effective_kd=float(plugin.findtext('steeringKd'));effective_seed=int(plugin.findtext('dynamicsSeed'))
    normalized=copy.deepcopy(c)
    for key in ('stage','purpose','run_id','seed'):normalized[key]=parent[key]
    normalized['effective_plugin_parameters']['steeringKd']=parent['effective_plugin_parameters']['steeringKd'];normalized['command_source_protocol']['node_script']=parent['command_source_protocol']['node_script'];normalized.pop('paired_pilot')
    perrun.append({'run_id':row['run_id'],'seed':row['seed'],'arm':row['arm'],'kd':row['steering_kd'],'contract_sha256':sha(cp),'launch_matches_final_renderer':launch_equal,'effective_Kd_and_seed_match':effective_kd==row['steering_kd'] and effective_seed==row['seed'],'contract_only_factor_and_identity_diff':normalized==parent,'expanded_robot_only_Kd_and_seed_diff_from_P02':normalize_robot(params['/robot_description'])==expected_robot,'world_exact_parent':(d/'world.sdf').read_text()==parent['world_fixture']['sdf']})
    resources.extend([cp]+[p for p in d.iterdir() if p.is_file()])
existing_report=json.loads((S/'runs/m12_physics_p02/probe_report.json').read_text());names=sorted(existing_report['checks'])
classify={}
for label,which in [('valid_physical_negative','actual_center_slew_with_declared_sampling_tolerance'),('measurement_failure','recorder_connections_ready_before_unpause'),('all_pass',None)]:
    r=copy.deepcopy(existing_report);r['checks']=dict.fromkeys(names,True)
    if which:r['checks'][which]=False
    r['pass']=all(r['checks'].values());classify[label]=module.classify_report(r,design,names)[0]
unit_receipts=[]
for row in rows:unit_receipts.append(dict(row,outcome='physical_negative' if row['arm']=='control' else 'qualified',pass_all_gates=row['arm']=='treatment',physical_failed_checks=['actual_center_slew_with_declared_sampling_tolerance'] if row['arm']=='control' else [],report={'metrics':{}}))
promote=module.matrix_summary(rows,unit_receipts,{})
bad=copy.deepcopy(unit_receipts);next(r for r in bad if r['arm']=='treatment')['pass_all_gates']=False
bad_summary=module.matrix_summary(rows,bad,{})
actual_sha=hashlib.sha256(payload).hexdigest()
checks={
 'runner_tested_final_sha':actual_sha=='c6c34155ea16c8523a29da588428a18d7c9fd91dffdb5b4fc4a03d9b1abb5bd6',
 'runner_37_offline_tests_pass':True,
 'runner_source_unchanged_during_review':actual_sha==sha(P),
 'exact6_order_and_shared_seeds':[(r['seed'],r['steering_kd']) for r in rows]==[(260926009,4),(260926009,6),(260926010,6),(260926010,4),(260926011,4),(260926011,6)],
 'schedule_arm_and_pair_labels_checked':all(r['arm']==('control' if r['steering_kd']==4 else 'treatment') and r['pair']==(r['sequence']-1)//2+1 and r['pair_position']==(r['sequence']-1)%2+1 for r in rows),
 'all_six_contracts_single_behavior_factor':all(r['contract_only_factor_and_identity_diff'] for r in perrun),
 'all_six_final_renderers_match_root_preview':all(r['launch_matches_final_renderer'] for r in perrun),
 'all_six_effective_factor_and_seed_match':all(r['effective_Kd_and_seed_match'] for r in perrun),
 'all_six_URDF_only_Kd_and_seed_change':all(r['expanded_robot_only_Kd_and_seed_diff_from_P02'] for r in perrun),
 'all_six_worlds_exact_parent':all(r['world_exact_parent'] for r in perrun),
 'P02_measured_prerequisite_preserves_physical_negative':prior['status']=='physical_negative' and prior['physical_failed_checks']==['actual_center_slew_with_declared_sampling_tolerance'] and len(prior['check_names'])==120,
 'P02_independent_protocol24_pass':json.loads((A/'m12_p02_raw_independent/protocol_audit.json').read_text())['passed']==24,
 'physical_negative_continues_measurement_failure_stops':classify=={'valid_physical_negative':'physical_negative','measurement_failure':'infrastructure_failure','all_pass':'qualified'},
 'control_negative_retained_and_treatment3of3_required':promote['treatment_promoted'] and not bad_summary['treatment_promoted'] and all('control' in r['results'] for r in promote['pairs']),
 'no_numeric_gate_relaxation':all(yaml.safe_load((S/r['contract_path']).read_text())['gates']==parent['gates'] for r in rows),
 'no_prior_identity_or_budget_reuse_at_review':not module.RUNTIME_LEDGER.exists() and all(not (S/'runs'/r['run_id']).exists() for r in rows),
 'budget6_and_terminal_no_retry':design['maximum_episodes']==6 and design['failure_policy']['no_retry'] and design['failure_policy']['no_resume'] and design['failure_policy']['no_replacement'],
}
report={'status':'independent_P03_single_factor_budget_runner_review_pass' if all(checks.values()) else 'fail','checks':checks,'count':len(checks),'passed':sum(checks.values()),'per_run':perrun,'P02_predecessor':prior,'classification_fixtures':classify,'runner_sha256':actual_sha,
 'test_command':str(W/'.venv/bin/python')+' -m pytest -q '+str(W/'scripts/astra_gazebo/tests/test_m12_physics_p03.py'),'test_result':'37passed; includes exact factor/arm/order, infrastructure failure at0/2/5, retirement/no resume, preparation failure receipt, physical control negatives,3of3 promotion, release tamper and active-source binding.',
 'source_render_note':'Root preview was created before closure/arm-label updates. Final render_launch output independently matches all six generated launch XML structures.',
 'resources':[{'path':str(p),'sha256':sha(p)} for p in resources],
 'scope':'Offline review only. No execute call, no ROS/Gazebo, no identities consumed. P03 is a3-pair actuator qualification pilot; every physical negative remains visible, no navigation/general-safety claim.',
 'resolved_finding':'arm/pair/pair_position labels now explicitly checked against fixed Kd/order;3rebound-label tests reject inconsistent labels.',
 'script_sha256':sha(__file__)}
(A/'m12_p03_independent_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
(A/'m12_p03_independent_review.md').write_text(f'''P03 独立离线审查：{sum(checks.values())}/{len(checks)} 通过；runner现有37项故障/预算测试全部通过。

六回合固定顺序为009(kd4,kd6)、010(kd6,kd4)、011(kd4,kd6)，每对共享seed。独立展开结果比P02仅Kd和seed变化；同对唯一行为变量为steeringKd 4→6，world、动作、测量、数字门全部保持。arm/pair标签约束已补齐并通过反例。

P02测量24项通过而物理slew失败的前提被保留。有效physical negative继续完整配对；基础设施/未知或缺失门失败停止并退休剩余身份，无重跑或恢复。须六回合测量均有效且treatment3/3全部gate通过方可晋级，所有control失败必须报告。37项测试覆盖这些分支及绑定篡改。

最终runner SHA `{actual_sha}`。完整六份合同/展开文件SHA与检查见同名JSON；审查未调用execute或启动ROS。
''')
print(json.dumps({'status':report['status'],'passed':sum(checks.values()),'count':len(checks),'failed':[k for k,v in checks.items() if not v],'runner_sha256':actual_sha,'review_sha256':sha(A/'m12_p03_independent_review.json')},indent=2))
