"""Offline independent N01 source/contract binding audit; grants no authority."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

OUT=Path(__file__).resolve().parent
S=OUT.parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n01 as manager
import navigation_evidence_p04 as q
import run_passive_sweep_q05 as passive_manager
import dense_sweep_p04 as dense
from evaluate_episode import WorldCollisions

def bind(path):
    path=Path(path).resolve()
    return {'path':str(path),'sha256':q.sha(path)}

def load(path):return json.loads(Path(path).read_text())

def checked(item):
    q.read_bound(item)
    return {'path':str(Path(item['path']).resolve()),'sha256':item['sha256']}

def write(name,value):
    with (OUT/name).open('x') as stream:stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')


prepared_path=S/'contracts/navigation_n01_prepared_r02/prepared.json'
preflight_path=S/'audits/navigation_n01_root_launch_preflight_r02/report.json'
base_path=S/'audits/passive_sweep_q05_root_source_review.json'
prepared=load(prepared_path);preflight=load(preflight_path);base=load(base_path)
assert preflight['prepared']==bind(prepared_path)
assert preflight['runner']==bind(Path(manager.__file__))
assert prepared['stage']==manager.NAME and prepared['execution_authorized']is False
assert [(r['run_id'],r['seed'],r['method']) for r in prepared['units']]==manager.identities()
assert len(preflight['runs'])==4
all_resources={}
def add(item):
    item=checked(item);old=all_resources.get(item['path'])
    assert old is None or old==item,'conflicting resource'
    all_resources[item['path']]=item

required=manager.required_resources()
for p in required:add(bind(p))
required_bindings=[bind(p) for p in required]
rows=[]
for unit,expanded in zip(prepared['units'],preflight['runs']):
    assert unit['run_id']==expanded['run_id'] and expanded['pass_']is True
    assert unit['attempt']==1 and type(unit['attempt'])is int
    assert [Path(x['path']).name for x in unit['files']]==['contract.json','robot_description.xml','fixture.launch']
    for item in unit['files']:add(item)
    value=json.loads(q.read_bound(unit['files'][0]));q.validate_contract(value)
    expected=q.make_contract(stage=manager.NAME,run_id=unit['run_id'],seed=manager.SEED,method=unit['method'],
       scene_id='navigation_n01_clear',world=manager.SCENE/'world.sdf',robot_description=unit['files'][1]['path'],
       scene_manifest=manager.SCENE/'scene_manifest.yaml',shared_bundle=manager.SCENE/'common_bundle',
       spawn_world_xyz_yaw=[-5.4,0.,0.,0.],steering_kd=6.,allowed_nodes=manager.node_roster(unit['method']),
       required_nodes=manager.node_roster(unit['method']),record_topics=manager.recorder_topics(unit['method']),
       required_before_arm=manager.recorder_topics(unit['method']))
    assert q.passive.same_typed(value,expected)
    assert q.read_bound(unit['files'][2]).decode()==manager.render_launch(value,unit['files'][0]['sha256'],S/'runs'/unit['run_id'])
    params=yaml.safe_load(q.read_bound(expanded['parameters']));add(expanded['parameters'])
    q.validate_robot_description(params['/robot_description'],value)
    assert hashlib.sha256(params['/robot_description'].encode()).hexdigest()==unit['files'][1]['sha256']==expanded['robot_description_sha256']
    assert params['/astra_navigation_collector/contract_path']==str(S/'runs'/unit['run_id']/'contract.json')
    assert params['/astra_navigation_collector/contract_sha256']==unit['files'][0]['sha256']
    assert params['/astra_navigation_coordinator/method']==unit['method']
    baseline=unit['method'] in ('actual_teb_common_route','dwa_common_route')
    if baseline:
        assert params['/move_base/base_global_planner']=='astra_common_route/FixedRouteGlobalPlanner'
        local='teb_local_planner/TebLocalPlannerROS' if unit['method']=='actual_teb_common_route' else 'dwa_local_planner/DWAPlannerROS'
        assert params['/move_base/base_local_planner']==local
    conversion=load(unit['conversion_manifest']['path'])
    assert conversion['input_urdf']==unit['files'][1] and conversion['converted_sdf']==unit['converted_sdf']
    resources=dense.conversion_resources(unit['conversion_manifest']['path'])
    assert resources==unit['conversion_resources']
    for item in resources:add(item)
    authentication=dense.authenticate_conversion(unit['conversion_manifest']['path'],unit['files'][1]['path'],unit['converted_sdf']['path'],{},diagnostic=True)
    assert authentication['pass'],authentication
    model=dense.parse_converted_model(q.read_bound(unit['files'][1]).decode(),q.read_bound(unit['converted_sdf']).decode())
    assert len(model['bodies'])==7 and len(model['source_mapping']['collision_map'])==9
    assert json.loads(json.dumps(model['source_mapping']))==conversion['fixed_link_and_collision_mapping']
    world=WorldCollisions(q.read_bound(value['world']).decode())
    assert not world.errors and not world.actor_speed_bounds
    rows.append(dict(run_id=unit['run_id'],method=unit['method'],seed=unit['seed'],attempt=1,
      contract_exact=True,launch_generator_exact=True,saved_actual_launch_parameters_exact=True,
      robot_description=unit['files'][1],conversion_manifest=unit['conversion_manifest'],converted_sdf=unit['converted_sdf'],
      conversion_resource_count=len(resources),body_count=7,collision_count=9,
      actual_N01_runtime_maps_and_metadata='pending future execution; offline conversion is not actual run loading'))

# Check earlier substantive independent reviews against these actual bytes.
review_paths=[
 S/'audits/navigation_n01_p04_independent/review_r1.json',
 S/'audits/navigation_coordinator_independent/review_r2.json',
 S/'audits/common_route_components/interface_review.json',
 S/'audits/route_runtime_interface_review.json',
 S/'audits/dense_sweep_p04_independent/review.json',
 S/'audits/bound_archive_independent/review_r1.json',
 S/'audits/graphics_mapping_independent/review.json']
review_matches=[]
for path in review_paths:
    if not path.exists():
        if 'graphics_mapping_independent' in str(path):continue
        raise FileNotFoundError(path)
    document=load(path);add(bind(path));verified=[]
    for key in ('reviewed_resources','resources','source_files'):
        items=document.get(key,[])
        if not isinstance(items,list):continue
        for item in items:
            if not isinstance(item,dict) or 'sha256' not in item:continue
            original=item.get('path',item.get('original'))
            if original and str(Path(original).resolve()) in all_resources:
                assert q.sha(original)==item['sha256'],('prior reviewed active source changed',original)
                verified.append(checked(dict(path=original,sha256=item['sha256'])))
    review_matches.append(dict(review=bind(path),active_bindings_reverified=verified))

native_fields=('native_engine_report','native_engine_records','upstream_source_manifest')
for name in native_fields:add(base[name])
for item in base['engine_source_bindings']:add(item)
upstream=load(base['upstream_source_manifest']['path'])
assert len(base['engine_source_bindings'])==len(upstream)==11
assert {(x['path'],x['sha256']) for x in base['engine_source_bindings']}=={(x['path'],x['sha256']) for x in upstream}
engine=[dict(x) for x in base['engine_module_bindings'] if x['role']!='actor_binaries']
assert len(engine)==len({x['role'] for x in engine})
assert {x['role'] for x in engine}==set(q.passive.ELF_ROLES.values())-{'actor_binaries'}
for item in engine:add(item)
native_review={key:base[key] for key in native_fields+('engine_source_bindings','gazebo_callback_order_review')}
native_review['engine_module_bindings']=engine
native_facts=passive_manager.native_review_facts(native_review)
assert native_facts['native_reference_pass'] and native_facts['upstream_source_review_pass']
native_rows=[json.loads(x) for x in q.read_bound(base['native_engine_records']).decode().splitlines()]

# Source-level authority corroboration for the new navigation launch.
launch=ET.fromstring(manager.LAUNCH.read_text())
collector=launch.find("node[@name='astra_navigation_collector']")
assert collector is not None and collector.get('required')=='true'
assert not any(x.get('type')=='spawn_model' for x in launch.iter('node'))
driver=W/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp'
driver_text=driver.read_text()
assert 'SetForce(' in driver_text
assert not any(token in driver_text for token in ('SetWorldPose(', 'SetLinearVel(', 'SetAngularVel(', 'SetVelocity(', 'SetPosition('))
no_gazebo_call_nodes=['astra_navigation_coordinator.py','astra_route_coordinated_runtime.py','astra_frozen_r2_coordinated.py','astra_common_route_provider.py']
for name in no_gazebo_call_nodes:
    tree=ast.parse((W/'src/simulation/m2_gazebo/scripts'/name).read_text())
    strings={n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)}
    assert not any(v in strings for v in ('/gazebo/set_model_state','/gazebo/set_link_state','/gazebo/reset_world','/gazebo/reset_simulation','/gazebo/unpause_physics','/gazebo/pause_physics','/gazebo/spawn_urdf_model'))
assert set(q.SERVICE_PHASES)=={'/gazebo/spawn_urdf_model','/gazebo/unpause_physics','/gazebo/pause_physics','/gazebo/get_world_properties','/gazebo/get_physics_properties','/astra/navigation/arm','/astra/navigation/stop'}
for path in (prepared_path,preflight_path,base_path,OUT/'runner_boundary_tests.log',Path(__file__),
 S/'logs/navigation_evidence_p04_root_tests.log',S/'audits/navigation_n01_root_launch_preflight_r02.py'):
    add(bind(path))

audit={'schema':'astra_navigation_n01_release_source_audit_v1','pass':True,
 'required_runtime_resource_count':len(required),'prepared':bind(prepared_path),'launch_preflight':bind(preflight_path),
 'units':rows,'native_facts':native_facts,'native_probe_row_count':len(native_rows),'engine_roles':sorted(x['role'] for x in engine),
 'upstream_source_count':len(upstream),'prior_review_active_binding_checks':review_matches,
 'tests':{'runner_boundary_passed':67,'log':bind(OUT/'runner_boundary_tests.log'),'navigation_evidence_prior_test_log':bind(S/'logs/navigation_evidence_p04_root_tests.log')},
 'no_seed_registry_or_ledger_write':True,'no_ROS_or_build':True}
write('audit.json',audit);add(bind(OUT/'audit.json'))
review={'schema':'astra_navigation_source_review_v1','stage':manager.NAME,'status':'independently_reviewed',
 'reviewer':'/root/environment_audit; independent current N01 release/source review, reusing bound substantive native/geometry/runner/coordinator reviews',
 'navigation_lifecycle_authority':'collector_only_spawn_unpause_pause_fixed_gateway',
 'robot_pose_velocity_writes':[],'actor_updates':'reviewed_bounded_Begin_updates',
 **native_review,'resources':sorted(all_resources.values(),key=lambda x:x['path']),
 'source_audit':bind(OUT/'audit.json'),'prepared':bind(prepared_path),'actual_launch_preflight':bind(preflight_path),
 'scope':'Four-method single clear-road fixture software/evidence integration only; no performance, strict sensor-only fairness, route-consumption equivalence or physical formal-safety claim.',
 'writer_review':{'initial_pose_authority':'collector spawn_urdf_model before unpause only','robot_dynamic_update':'P05 six joint SetForce calls; no robot pose/velocity setter','observers':'P03 pre/End metadata and whole contacts read only','actual_world_has_actor':False,'actor_review_field':'Bound reviewed actor Begin update source is inherited; inactive in N01 static world','service_gateway':'Only declared fixed services/phases; no reset or set-model/link-state; failures stop/pause and cannot qualify','trust_scope':'Authorized launch/nodes and isolated loopback master; not malicious-process exclusion'},
 'runtime_obligations':['Actual same-process Gazebo ELF maps before unpause/after pause must match these selected role SHAs.','Actual runtime native COM/inertia/collisions and2ms source streams must pass P04 converted-model and contact evidence assessment.','TEB/DWA actual configured classes and loaded plugins must be bound in same owned move_base PID/lifetime at terminal; launch configuration alone is not execution proof.','Source collection must complete final paused watermark drain within5s; required collector exit controls launch termination.'],
 'preserved_limitations':['N01 uses frozen_r2_v060_shared_activation, whose original oracle-dependent readiness/goal stopping and raw-scan/padded history weaknesses remain; separately repaired sensor lifecycle is NOT substituted.','Frozen R2 does not consume shared global path; raster static union and shared availability do not establish consumer semantic equality.','Analytic route uses conservative nominal candidates; learned new PCT is absent because no qualified checkpoint is supplied.','Failure-inclusive task outcomes stay separate from capture integrity; no performance interpretation from four clear-road units.','Engineering normal-Euler extension and actor Begin-jump scope is not real continuous physical/formal safety.'],
 'execution_authorized':False,'seed_allocation_authorized':False,
 'pending_before_execution':['Root evidence-bound passive geometry qualification decision after Q05 independent result review.','Root final exact release, full resource rehash and verify_release success, then separate exact registry reservation.'],
 'review_does_not_grant_authority':True}
write('source_review.json',review)
release_resources=dict(all_resources)
release_resources[str((OUT/'source_review.json').resolve())]=bind(OUT/'source_review.json')
write('release_required_resources.json',{'schema':'astra_navigation_source_review_resources_v1','stage':manager.NAME,
  'source_review':bind(OUT/'source_review.json'),'resources':sorted(release_resources.values(),key=lambda x:x['path']),
  'additional_root_owned_bindings_pending':['passive_geometry_qualification decision and its exact evidence bindings','final release-specific independent evidence if added'],
  'execution_authorized':False})
print(json.dumps({'source_review':bind(OUT/'source_review.json'),'resource_list':bind(OUT/'release_required_resources.json'),
 'review_resource_count':len(all_resources),'required_runtime_resource_count':len(required),'engine_role_count':len(engine),'unit_count':len(rows)}))
