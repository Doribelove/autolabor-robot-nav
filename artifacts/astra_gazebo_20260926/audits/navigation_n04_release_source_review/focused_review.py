"""Independent N04 prepared/runtime boundaries. No ROS, execution or release."""
import ast,copy,hashlib,json,sys,types
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n04 as r
import navigation_evidence_n04 as q
import event_actor_contract_n04 as event
import dense_sweep_n04 as dense
from prepare_navigation_n04_scenes import bind_gate_identity,collision_signature

def load(p):return json.loads(Path(p).read_text())
def binding(p):return {'path':str(Path(p).resolve()),'sha256':q.sha(p)}
def save(name,x):(D/name).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
checks=[]
def check(name,value):
    assert value,name
    checks.append(name)

prepared=load(S/'contracts/navigation_n04_prepared_r01/prepared.json');design=r.design();blocks=r.input_blocks()
check('exact48_order_and_identity',[(u['run_id'],u['seed'],u['method']) for u in prepared['units']]==r.identities())
check('unique48_attempt1',len({u['run_id'] for u in prepared['units']})==48 and all(type(u['attempt'])is int and u['attempt']==1 for u in prepared['units']))
methods=design['methods'];rotations=[]
for i in range(12):
    group=prepared['units'][i*4:i*4+4];order=[u['method'] for u in group]
    check('Latin_order_block_'+str(i),order==methods[i%4:]+methods[:i%4])
    check('same_seed_block_'+str(i),{u['seed'] for u in group}=={263001001+i})
    rotations.append(order)
actual=load(S/'audits/navigation_n04_root_launch_preflight_r01/report.json');params_by_run={row['run_id']:row for row in actual['runs']}
rows=[];dedup={};block_world_signatures={};block_urdfs={}
for unit in prepared['units']:
    name=unit['run_id'];value=load(unit['files'][0]['path']);q.validate_contract(value)
    expected=r.contract_for(name,unit['files'][1]['path'],unit['files'][3]['path'])
    check('exact_contract_'+name,q.passive.same_typed(value,expected))
    check('exact_launch_'+name,q.read_bound(unit['files'][2]).decode()==r.render_launch(value,unit['files'][0]['sha256'],S/'runs'/name))
    template=Path(r.block_for(name)['world_path']).read_text();world=q.read_bound(unit['files'][3]).decode()
    check('bound_world_'+name,world==bind_gate_identity(template,name,unit['seed'],unit['method']))
    block=next(x['block_index'] for x in design['schedule'] if x['run_id']==name)
    sig=collision_signature(world);check('same_block_geometry_'+name,block_world_signatures.setdefault(block,sig)==sig)
    check('same_block_URDF_'+name,block_urdfs.setdefault(block,unit['files'][1]['sha256'])==unit['files'][1]['sha256'])
    actors=event.world_actors(world)
    check('all_actor_identity_'+name,all(all(a[k]==value[k] for k in ('run_id','seed','method')) for a in actors.values()))
    for actor in actors:
        topic=event.status_topic(actor);check('status_recording_'+name+'_'+actor,all(topic in value[k] for k in ('record_topics','required_before_arm','required_before_unpause')))
    params=yaml.safe_load(q.read_bound(params_by_run[name]['parameters']))
    q.validate_robot_description(params['/robot_description'],value)
    check('actual_URDF_'+name,hashlib.sha256(params['/robot_description'].encode()).hexdigest()==unit['files'][1]['sha256'])
    check('actual_collector_contract_'+name,params['/astra_navigation_collector/contract_sha256']==unit['files'][0]['sha256'])
    check('actual_coordinator_method_'+name,params['/astra_navigation_coordinator/method']==unit['method'])
    if unit['method'] in ('dwa_common_route','actual_teb_common_route'):
        check('actual_baseline_plugin_'+name,params['/move_base/base_local_planner']==('dwa_local_planner/DWAPlannerROS' if unit['method']=='dwa_common_route' else 'teb_local_planner/TebLocalPlannerROS'))
        check('actual_common_route_'+name,params['/move_base/base_global_planner']=='astra_common_route/FixedRouteGlobalPlanner')
        if unit['method']=='dwa_common_route':check('DWA_only_nonholonomic_sampling_'+name,params['/move_base/DWAPlannerROS/acc_lim_y']==2.5 and params['/move_base/DWAPlannerROS/min_vel_y']==params['/move_base/DWAPlannerROS/max_vel_y']==0.)
    if unit['method']=='frozen_r2_v060_sensor_lifecycle_v1':
        check('actual_frozen_R2_weights_'+name,params['/astra_frozen_r2_method/actor_checkpoint_sha256']==q.sha(r.CHECKPOINT))
        check('actual_sensor_R2_enum_'+name,params['/astra_frozen_r2_method/execution_method']==unit['method'])
    conversion=load(unit['conversion_manifest']['path'])
    check('conversion_input_'+name,conversion['input_urdf']==unit['files'][1] and conversion['converted_sdf']==unit['converted_sdf'])
    check('conversion_dependency_roster_'+name,dense.conversion_resources(unit['conversion_manifest']['path'])==unit['conversion_resources'])
    for item in unit['files']+unit['conversion_resources']+[params_by_run[name]['parameters']]:
        if item['path'] in dedup:assert dedup[item['path']]==item['sha256']
        else:q.read_bound(item);dedup[item['path']]=item['sha256']
    rows.append({'run_id':name,'seed':unit['seed'],'method':unit['method'],'actor_count':len(actors),'actual_parameters':params_by_run[name]['parameters']})
# Compile exact callback bodies, avoiding synthetic reimplementation of errors/freeze.
entry=W/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_n04.py';tree=ast.parse(entry.read_text());nodes=[next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name) for name in ('parse_callback','callback')]
class String:
    def __init__(self,data):self.data=json.dumps(data)
value=load(prepared['units'][8]['files'][0]['path']);actor=next(iter(event.world_actors(q.read_bound(value['world']).decode())));topic=event.status_topic(actor)
def callback_case(item):
    raw={'errors':[],'actor_gate_status':[]};store=q.EvidenceStore(raw)
    env=dict(q=q,json=json,String=String,raw=raw,value=value,actor_topics={topic:actor},evidence=store,lock=store.lock)
    exec(compile(ast.Module(body=copy.deepcopy(nodes),type_ignores=[]),str(entry),'exec'),env)
    env['callback'](topic,String(item));return raw,store,env
base=dict(schema=event.STATUS_SCHEMA,model_name=actor,fault=False,reason='',**{k:value[k] for k in ('run_id','seed','method')})
raw,store,env=callback_case(base);check('actual_callback_good',not raw['errors'] and len(raw['actor_gate_status'])==1)
frozen=store.freeze();env['callback'](topic,String(dict(base,fault=True,reason='late')));check('actual_callback_closed_guard',raw==frozen and not raw['errors'])
for key,bad in [('fault',True),('fault',0),('seed',True),('method','foreign'),('schema','old'),('model_name','other')]:
    raw,_,_=callback_case(dict(base,**{key:bad}));check('actual_callback_reject_'+key+str(bad),bool(raw['errors']))
    if key!='model_name':check('actual_callback_preserve_fault_payload_'+key+str(bad),len(raw['actor_gate_status'])==1)
# New actual import closure and new native actor role must be explicit.
required={p.name for p in r.required_resources()}
check('new_module_import_closure',{'prepare_navigation_n01_scene.py','scene_generator_v2.py','scene_generator.py'}<=required)
check('new_ELF_role',event.PLUGIN in q.ELF_ROLES and q.ELF_ROLES[event.PLUGIN]=='actor_binaries' and 'libastra_reactive_actor_plugin.so' not in q.ELF_ROLES)
closure=next(n for n in ast.walk(ast.parse(Path(q.__file__).read_text())) if isinstance(n,ast.FunctionDef) and n.name=='closure')
assignment=next(n for n in closure.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='required_sources' for t in n.targets))
env={'WORKSPACE':W,'root':Path('/tmp/independent_not_created')};exec(compile(ast.Module(body=[assignment],type_ignores=[]),'<actual closure assignment>','exec'),env)
check('new_closure_paths',str(W/'scripts/astra_gazebo/navigation_evidence_n04.py') in map(str,env['required_sources']) and str(W/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_n04.py') in map(str,env['required_sources']))
save('focused_review.json',{'status':'pass','checks_count':len(checks),'checks':checks,'prepared_rows':rows,'distinct_prepared_runtime_resources_rehashed':len(dedup),'source_files':[binding(r.__file__),binding(q.__file__),binding(event.__file__),binding(entry)],'no_ROS_or_seed_or_release':True})
print(json.dumps({'checks':len(checks),'rows':len(rows),'distinct_resources':len(dedup),'pass':True}))
