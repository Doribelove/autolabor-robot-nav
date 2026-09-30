import ast,copy,hashlib,importlib,json,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import route_candidate_core as parent
before=dict(parent.RouteAwareRuntime.__dict__)
import route_preview_selection_n05 as selector
assert all(parent.RouteAwareRuntime.__dict__.get(k) is v for k,v in before.items())
import event_actor_contract_n04 as old
import event_actor_contract_n05 as new

def bind(p):
 p=Path(p).resolve();raw=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
def reject(fn):
 try:fn()
 except (ValueError,RuntimeError):return
 raise AssertionError('expected rejection absent')
rows=[]
derivation=json.loads((S/'audits/navigation_n05_metadata_derivation/derivation.json').read_text())
for row in derivation['files']:
 a,b=Path(row['original_path']),Path(row['new_path']);assert bind(a)['sha256']==row['original_sha256'];assert bind(b)['sha256']==row['new_sha256']
 expected=a.read_text().replace('N04','N05').replace('n04','n05')
 if a.suffix=='.h':expected=expected.replace('"astra_pct_route_v1"};','"astra_pct_route_v1", "astra_analytic_preview_eligible_n05"};')
 if a.suffix=='.py':expected=expected.replace("'astra_analytic_route_v1', 'astra_pct_route_v1',","'astra_analytic_route_v1', 'astra_pct_route_v1', 'astra_analytic_preview_eligible_n05',")
 assert b.read_text()==expected
 rows.append(dict(name='exact_whitelisted_derivation_'+a.name,passed=True,old=bind(a),new=bind(b)))
# Old protocol and policy are inherited from the independent N04 review. This
# does not compile or requalify the original actor algorithm.
assert new.MAX_TRANSPORT_NS==old.MAX_TRANSPORT_NS==50_000_000
assert set(new.METHODS)==set(old.METHODS)|{selector.METHOD}
assert new.GATE_TOPIC==old.GATE_TOPIC and new.PLUGIN=='libastra_event_actor_n05_plugin.so'
assert new.STATUS_SCHEMA=='astra_event_actor_gate_n05_v1'
rows.append(dict(name='actor_namespace_and_exact_one_added_method',passed=True))
# Independent selector reference with Python filtering/max, not ndarray masking.
prior=np.asarray([0.,2.,2.,1.,4.,3.,8.,9.],dtype=np.float32)
masks=[[False]*8,[True]*8]+[[j==i for j in range(8)] for i in range(8)]+[[True,False,True,False,True,False,True,False],[False,True,False,True,False,True,False,True]]
fulls=[np.asarray([-1.]*8),np.asarray([0.]*8),np.asarray([1.,-1e-50,np.nextafter(0.,-1.),0.,1.,-0.,-1.,np.nextafter(0.,1.)])]
for mask in masks:
 for full in fulls:
  observation=dict(action_mask=np.asarray(mask,dtype=bool),analytic_prior=prior.copy());backup=observation['action_mask'].copy();prior_before=observation['analytic_prior'].copy();full_before=full.copy()
  eligible=[i for i in range(8) if mask[i] and (i==0 or float(full[i])>=0.)]
  expected=max(eligible,key=lambda i:(float(prior[i]),-i)) if eligible else 0
  actual=selector.select_preview_eligible(observation,full)
  assert actual.chosen==expected and list(actual.selection_eligibility)==[i in eligible for i in range(8)]
  assert actual.backup_mask==tuple(mask)
  np.testing.assert_array_equal(observation['action_mask'],backup);np.testing.assert_array_equal(observation['analytic_prior'],prior_before);np.testing.assert_array_equal(full,full_before)
  if not eligible:assert actual.proposed is None
rows.append(dict(name='selector_independent_filter_reference_and_input_immutability',passed=True,cases=len(masks)*len(fulls)))
for name in ('state','step','predict','command','goal_reached'):
 assert getattr(selector.PreviewEligibleRuntime,name) is getattr(parent.RouteAwareRuntime,name)
reject(lambda:selector.PreviewEligibleRuntime(None,None,None,actor=lambda _:1))
reject(lambda:selector.PreviewEligibleRuntime(None,None,None,checkpoint_metadata={}))
rows.append(dict(name='parent_unmodified_actor_rejection_and_inherited_command',passed=True))
# Source protocol capture provides a float64 original full value. Public pure
# selector cannot reconstruct original precision from already rounded inputs.
obs=dict(action_mask=np.ones(8,dtype=bool),analytic_prior=np.arange(8,dtype=np.float32))
full=np.asarray([1.]*7+[-1e-50],dtype=np.float64)
a=selector.select_preview_eligible(obs,full);b=selector.select_preview_eligible(obs,full.astype(np.float32))
assert (a.chosen,b.chosen)==(6,7)
rows.append(dict(name='rounded_public_input_outside_unrounded_precondition',passed=True,original_choice=a.chosen,rounded_choice=b.chosen,
 boundary='Use PreviewEligibleRuntime.build original-return capture. Do not reconstruct or transport float32 full values; helper type shape checks do not prove provenance.'))
# Actual saved parent world with only explicit metadata/plugin namespace change.
prepared=json.loads((S/'contracts/navigation_n04_prepared_r01/prepared.json').read_text())
world_binding=None
for unit in prepared['units']:
 candidate=next(r for r in unit['files'] if Path(r['path']).name=='world.sdf')
 if old.world_actors(Path(candidate['path']).read_text()):world_binding=candidate;break
assert world_binding;tree=ET.fromstring(Path(world_binding['path']).read_bytes())
plugins=[p for p in tree.findall('.//model/plugin') if p.get('filename')==old.PLUGIN]
assert plugins
for p in plugins:p.set('filename',new.PLUGIN);p.find('expected_method').text=selector.METHOD
world=ET.tostring(tree,encoding='unicode');actors=new.world_actors(world)
assert actors and all(row['method']==selector.METHOD for row in actors.values())
reject(lambda:old.world_actors(world))
base=plugins[0]
for key,value in (('expected_method','foreign'),('expected_seed','true'),('expected_run_id','UNBOUND'),('gate_topic','/foreign'),('max_gate_transport_delay_ns','50000001'),('gate_delay','nan')):
 altered=copy.deepcopy(base);altered.find(key).text=value;reject(lambda p=altered:new.plugin_contract(p))
for forbidden in ('start_delay','gate_route_x'):
 altered=copy.deepcopy(base);ET.SubElement(altered,forbidden).text='0';reject(lambda p=altered:new.plugin_contract(p))
altered=copy.deepcopy(base);ET.SubElement(altered,'expected_seed').text='1';reject(lambda:new.plugin_contract(altered))
rows.append(dict(name='actual_parent_world_new_actor_python_contract_boundaries',passed=True,input=world_binding,negative_cases=10))
# Source-status wire changed, while all status timeline arithmetic stays exact.
actor_name,expected=next(iter(actors.items()));gate={'source_ns':6_000_000_000}
first=gate['source_ns']+2_000_000;activation=gate['source_ns']+expected['gate_delay_ns'];active=max(first,((activation+1_999_999)//2_000_000)*2_000_000)
end=active+20_000_000
status=dict(schema=new.STATUS_SCHEMA,run_id=expected['run_id'],seed=expected['seed'],method=expected['method'],model_name=actor_name,
 status_source_ns=end,fault=False,reason='',gate_source_ns=gate['source_ns'],receipt_ros_ns=gate['source_ns'],receipt_steady_ns=1,
 first_begin_ns=first,transport_delay_ns=2_000_000,activation_source_ns=activation,first_active_begin_ns=active,
 gate_delay_ns=expected['gate_delay_ns'],duplicate_count=0,max_gate_transport_delay_ns=50_000_000)
assert new.audit_status([status],{actor_name:expected},gate,end)['pass_']
for updates in ({'schema':old.STATUS_SCHEMA},{'first_active_begin_ns':0},{'first_active_begin_ns':end+2_000_000},{'fault':True},{'seed':True},{'method':'foreign'},{'transport_delay_ns':50_000_001}):
 assert not new.audit_status([dict(status,**updates)],{actor_name:expected},gate,end)['pass_']
assert not new.audit_status([],{actor_name:expected},gate,end)['pass_']
rows.append(dict(name='new_status_schema_and_inherited_timing_negative_boundaries',passed=True,negative_cases=8))
result=dict(schema='astra_navigation_n05_components_independent_checks_v1',passed=len(rows),total=len(rows),checks=rows,no_ros=True,no_native_build=True,no_benchmark=True)
(D/'checks.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))
