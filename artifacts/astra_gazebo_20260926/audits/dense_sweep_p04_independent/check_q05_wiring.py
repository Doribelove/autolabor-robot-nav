#!/usr/bin/env python3
"""Actual Q05 prepared artifacts and assessor argv boundary. No ROS/simulation."""
import json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import dense_sweep_p04 as dense
import run_passive_sweep_q05 as runner
import passive_sweep_q04 as old
import passive_sweep_q05 as new
prepared_path=S/'contracts/passive_sweep_q05_prepared_r02/prepared.json';prepared=json.loads(prepared_path.read_text())
assert dense.sha(prepared_path)=='c0b605790463d44eab9adfdd3602476b13c7057637929d8829f02b74dc458f85'
resource_checks=[dict(x,matches=dense.sha(x['path'])==x['sha256'])for x in prepared['resources']];units=[]
for unit in prepared['units']:
 value=json.loads(new.read_bound(unit['files'][0]));new.validate_contract(value)
 manifest=json.loads(new.read_bound(unit['conversion_manifest']));resources=dense.conversion_resources(unit['conversion_manifest']['path'])
 model=dense.parse_converted_model(new.read_bound(unit['files'][2]).decode(),new.read_bound(unit['converted_sdf']).decode())
 auth=dense.authenticate_conversion(unit['conversion_manifest']['path'],unit['files'][2]['path'],unit['converted_sdf']['path'],{},True)
 units.append({'run_id':unit['run_id'],'seed':unit['seed'],'files_match':all(dense.sha(x['path'])==x['sha256']for x in unit['files']),
  'manifest_input_matches':manifest['input_urdf']==unit['files'][2],'manifest_output_matches':manifest['converted_sdf']==unit['converted_sdf'],
  'resource_roster_exact':resources==unit['conversion_resources'],'resource_count':len(resources),'all_conversion_resources_match':all(dense.sha(x['path'])==x['sha256']for x in resources),
  'launch_render_exact':new.read_bound(unit['files'][3]).decode()==runner.render_launch(S/'runs'/unit['run_id'],value),
  'actual_model_bodies':len(model['bodies']),'actual_collision_mappings':len(model['source_mapping']['collision_map']),'offline_conversion_auth':auth})
# Invoke actual assessor through the first subprocess boundary; stop before any
# assessor process starts. Verify all paths against the real prepared inputs.
fixture=D/'assess_argv_fixture';fixture.mkdir();(fixture/'collector_raw.json').write_text(json.dumps({'start_ns':1160000000,'end_ns':11160000000}))
class Captured(Exception):pass
calls=[]
def capture(command,**kwargs):calls.append(command);raise Captured()
original=runner.subprocess.run;runner.subprocess.run=capture
try:
 try:runner.assess(fixture,json.loads(new.read_bound(prepared['units'][0]['files'][0])))
 except Captured:pass
finally:runner.subprocess.run=original
assert len(calls)==1;command=calls[0]
checks={'64_prepared_resources_hash_match':len(resource_checks)==64 and all(x['matches']for x in resource_checks),
 'two_actual_converted_models_and_paths':len(units)==2 and all(all(x[k]for k in ('files_match','manifest_input_matches','manifest_output_matches','resource_roster_exact','all_conversion_resources_match','launch_render_exact'))and x['offline_conversion_auth']['pass']and x['actual_model_bodies']==7 and x['actual_collision_mappings']==9 for x in units),
 'same_frozen_command_phases_limits':old.PHASES==new.PHASES and old.STEP_NS==new.STEP_NS and old.PERIOD_NS==new.PERIOD_NS and old.DURATION_NS==new.DURATION_NS,
 'assess_calls_new_p04_actual_argv':Path(command[1]).name=='dense_sweep_p04.py'and command[command.index('--converted-sdf')+1]==str(fixture/'converted_robot.sdf')and command[command.index('--conversion-manifest')+1]==str(fixture/'conversion_manifest.json')and '--diagnostic-from-p03-closure'not in command,
 'runtime_converter_ELF_role_required':new.ELF_ROLES['libsdformat9.so']=='urdf_to_sdf_converter',
 'no_q05_execution_ledger_or_run':not(S/'queue/passive_sweep_q05_ledger.json').exists()and not any((S/'runs'/n).exists()for n,_,_ in new.FIXTURES)}
result={'pass':all(checks.values()),'checks':checks,'prepared_binding':dense.binding(prepared_path),'reviewed_runner':dense.binding(runner.__file__),'resources':resource_checks,'units':units,'actual_assess_argv_captured_before_process':command,'scope':'Actual offline prepared model/manifest/launch inputs. The assessor subprocess boundary is intercepted; no simulation or qualification is claimed.'}
with (D/'q05_wiring_review.json').open('x')as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({'pass':result['pass'],'checks':checks},indent=2));assert result['pass']
