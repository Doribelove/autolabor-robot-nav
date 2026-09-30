"""Independent offline source/actual XML expansion checks; does not launch nodes."""
import ast,copy,hashlib,json,pathlib,sys,xml.etree.ElementTree as ET
import yaml,roslaunch.config,roslaunch.xmlloader
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo';A=S/'audits/baseline_integration_b04';D=pathlib.Path(__file__).parent
sys.path.insert(0,str(P))
import baseline_rear_evidence_b04 as rear
import navigation_evidence_b04 as q
READ={};checks=[]
def read(path):
 p=pathlib.Path(path).resolve();b=p.read_bytes();READ[str(p)]=hashlib.sha256(b).hexdigest();return b
def js(p):return json.loads(read(p))
def check(name,ok,details=None):
 checks.append(dict(name=name,passed=bool(ok),details=details));assert ok,(name,details)
def signature(n):return [n.tag,dict(n.attrib),(n.text or '').strip(),[signature(x) for x in n]]
def defs(p):return {n.name:ast.dump(n,include_attributes=False) for n in ast.parse(read(p)).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
def flatten(x,prefix=''):
 out={}
 for k,v in x.items():
  key=prefix+'/'+k
  if isinstance(v,dict):out.update(flatten(v,key))
  else:out[key]=v
 return out
try:
 manifest=js(A/'source_manifest.json')
 for b in manifest['resources']:assert hashlib.sha256(read(b['path'])).hexdigest()==b['sha256'],b
 check('199_author_resources_still_match',len(manifest['resources'])==199)
 old=ET.fromstring(read(G/'launch/astra_baseline_rear_b03.launch'));new=ET.fromstring(read(G/'launch/astra_baseline_rear_b04.launch'))
 old.remove(next(x for x in old.findall('arg') if x.get('name')=='collector_script'))
 next(x for x in old.findall('node') if x.get('name')=='astra_navigation_collector').set('type','astra_navigation_collector_b04.py')
 check('B03_launch_only_collector_binding_changes',signature(old)==signature(new))
 inherited=defs(P/'navigation_evidence_n02.py');actual=defs(P/'navigation_evidence_b04.py')
 identical=sorted(k for k in inherited.keys()&actual.keys() if inherited[k]==actual[k]);changed=sorted(k for k in inherited.keys()&actual.keys() if inherited[k]!=actual[k])
 check('inherited_public_task_and_drain_primitives_unchanged',all(k in identical for k in ('gate_record','terminal_record','navigation_outcome','validate_services','validate_robot_description','wait_condition','readiness_guard')),dict(identical=identical,changed=changed))
 cost=yaml.safe_load(read(G/'config/astra_navigation_common_footprint.yaml'));overlay=yaml.safe_load(read(G/'config/astra_baseline_rear_b03.yaml'))
 counts={}
 for group,field in [('global_costmap','footprint'),('local_costmap','footprint'),('TebLocalPlannerROS','footprint_model')]:
  a=cost[group][field];b=overlay[group][field]
  if field=='footprint_model':a=a['vertices'];b=b['vertices']
  assert len(a)==len(b)==(16 if field=='footprint_model' else 64)
  assert all(y==y2 and abs(x+.325-x2)<2e-16 for (x,y),(x2,y2) in zip(a,b))
  counts[group]=len(a)
 check('all64_padded_and16_raw_footprint_vertices_translate_without_reduction',True,counts)
 result=[]
 for method in rear.METHODS:
  folder=A/'launch_preflight_final_r2'/method;value=js(folder/'contract.json')
  q.validate_contract(value)
  cfg=roslaunch.config.ROSLaunchConfig();roslaunch.xmlloader.XmlLoader().load(str(folder/'fixture.launch'),cfg,verbose=False)
  params={k:v.value for k,v in cfg.params.items()};saved=yaml.safe_load(read(folder/'parameters.yaml'))
  check('actual_parameters_exact_'+method,params==saved,dict(count=len(params)))
  nodes=[dict(package=n.package,type=n.type,name=n.name,required=n.required,args=n.args,remaps=[list(x) for x in n.remap_args],launch_prefix=n.launch_prefix)for n in cfg.nodes]
  check('actual_node_roster_exact_'+method,nodes==js(folder/'resolved_nodes.json'),dict(count=len(nodes)))
  assert params['/robot_description'].encode()==read(folder/'robot_description.xml');q.validate_robot_description(params['/robot_description'],value)
  effective=js(folder/'effective_move_base.json')
  assert all(params['/move_base'+k]==v for k,v in flatten(effective).items())
  rear.planner_parameters(effective,method)
  planner='TebLocalPlannerROS' if method==rear.METHODS[0] else 'DWAPlannerROS'
  baseline=yaml.safe_load(read(G/'config'/('benchmark_teb_v0.yaml' if planner=='TebLocalPlannerROS' else 'benchmark_dwa_nonholonomic_r1.yaml')))[planner]
  allowed=set(overlay[planner])|set(cost.get(planner,{}))
  compared={k:v for k,v in baseline.items() if k not in allowed}
  assert all(effective[planner][k]==v for k,v in compared.items())
  assert effective['global_costmap']['robot_base_frame']==effective['local_costmap']['robot_base_frame']==rear.FRAME
  assert effective['FixedRouteGlobalPlanner']['route_service']=='/astra/baseline_rear/get_plan'
  assert effective[planner]['xy_goal_tolerance']==.15 and effective[planner]['yaw_goal_tolerance']==.1
  assert all(effective[c]['footprint_padding']==0 for c in ('global_costmap','local_costmap'))
  if planner=='DWAPlannerROS':assert effective[planner]['acc_lim_y']==2.5 and effective[planner]['min_vel_y']==effective[planner]['max_vel_y']==0 and effective[planner]['latch_xy_goal_tolerance'] is False
  nd={n['name']:n for n in nodes};assert nd['astra_navigation_collector']['required'] and nd['astra_baseline_rear_adapter_b03']['required']
  assert nd['move_base']['remaps']==[['cmd_vel','/astra/method_cmd'],['odom','/astra/rear_odom']]
  assert nd['navigation_command_relay']['args']=='/cmd_vel_navigation /cmd_vel'
  assert '-u ' in nd['gazebo']['args'] and '--seed 1 ' in nd['gazebo']['args']
  assert not pathlib.Path(value['output']).exists() if 'output' in value else True
  rear.static_world_only(read(value['world']['path']).decode())
  changed_params=copy.deepcopy(effective);changed_params[planner]['max_vel_x']=17.
  # Scope counterexample: overlay checker does not claim all runtime fields.
  scope_probe=rear.planner_parameters(changed_params,method)
  assert scope_probe['pass_'] is True
  row=dict(method=method,baseline_nonoverlay_fields_compared=len(compared),overlay_keys=sorted(allowed),full_effective_parameters_saved=True,overlay_validator_accepts_changed_max_vel_x_scope_limitation=True,contract_sha256=hashlib.sha256(read(folder/'contract.json')).hexdigest(),all_robot_components=9)
  result.append(row)
 check('two_actual_static_launches_private_reference_and_baseline_nonoverlay_values',True,result)
 # Independent earlier runtime review retains public base authority, partial batch and tilt negatives.
 b03=js(S/'audits/baseline_rear_runtime_b03_independent/review.json')
 check('frozen_B03_independent_review_bound',hashlib.sha256(read(S/'audits/baseline_rear_runtime_b03_independent/review.json')).hexdigest()=='cfd9f8166fec91b46b611c0fb0c20bbdeb2dc4bf86bc8c81001f6b8e543aa13a')
 status='pass'
except Exception as exc:
 status='fail';checks.append(dict(name='unexpected_exception',passed=False,details=repr(exc)));raise
finally:
 (D/'launch_source_checks_r01.json').write_text(json.dumps(dict(status=locals().get('status','fail'),checks=checks,resources=[dict(path=k,sha256=v)for k,v in sorted(READ.items())],ROS_started=False),indent=2,allow_nan=False)+'\n')
