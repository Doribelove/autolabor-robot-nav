"""Independent minimal N03 stage/config review; no release or simulation."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import yaml

P=Path(__file__).resolve().parent;S=P.parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n02 as previous_manager
import run_navigation_n03 as manager
import navigation_evidence_n02 as evidence
import dense_sweep_p04 as dense
import run_passive_sweep_q05 as passive_manager

def bind(p):
    p=Path(p).resolve();return {'path':str(p),'sha256':evidence.sha(p)}
def load(p):return json.loads(Path(p).read_bytes())
def write(name,value):(P/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')

class Normalize(ast.NodeTransformer):
    def visit_Constant(self,n):
        if isinstance(n.value,str):
            n.value=n.value.replace('navigation_n03_dwa_r01','navigation_n02_sensor_r01').replace('navigation_n03','navigation_n02').replace('N03','N02').replace('reserved_exact_n03_release','reserved_exact_n02_release')
        elif n.value==260926034:n.value=260926033
        return n
    def visit_Assign(self,n):
        self.generic_visit(n)
        if any(isinstance(t,ast.Name) and t.id=='METHODS' for t in n.targets):
            n.value=ast.parse("('frozen_r2_v060_sensor_lifecycle_v1',)",mode='eval').body
        return n
    def visit_Tuple(self,n):
        self.generic_visit(n)
        n.elts=[x for x in n.elts if not (isinstance(x,ast.Constant) and x.value=='benchmark_dwa_nonholonomic_r1.yaml')]
        return n
    def visit_Call(self,n):
        self.generic_visit(n);n.keywords=[k for k in n.keywords if k.arg!='dwa_config'];return n

x=ast.parse(Path(previous_manager.__file__).read_text());y=Normalize().visit(ast.parse(Path(manager.__file__).read_text()))
x.body=x.body[1:];y.body=y.body[1:]
assert ast.dump(x)==ast.dump(y)
normalization={'normalized_AST_equal':True,'before':bind(previous_manager.__file__),'after':bind(manager.__file__),
    'changes':'Only new stage/seed/schedule/scene/registry identity, source filename, new config resource, and explicit dwa_config argument; module docstring excluded.',
    'review_script_note':'An initial interactive normalizer omitted the reserved_exact_n03_release literal and therefore asserted unequal; this script explicitly includes that declared registry identity. It did not reveal a behavior difference.'}
write('runner_normalized_equivalence.json',normalization)

old_review_path=S/'audits/navigation_n02_release_source_review/source_review.json';old=load(old_review_path)
preflight_path=S/'audits/navigation_n03_root_launch_preflight_r01/report.json';preflight=load(preflight_path)
prepared_path=S/'contracts/navigation_n03_prepared_r01/prepared.json';prepared=load(prepared_path)
assert preflight['runner']==bind(manager.__file__) and preflight['prepared']==bind(prepared_path)
assert prepared['maximum_episodes']==1 and prepared['execution_authorized']is False and len(prepared['units'])==1
unit=prepared['units'][0];assert (unit['run_id'],unit['seed'],unit['method'])==manager.identities()[0]
value=load(unit['files'][0]['path']);evidence.validate_contract(value)
assert value['method']=='dwa_common_route' and value['seed']==260926034
assert manager.render_launch(value,unit['files'][0]['sha256'],S/'runs'/unit['run_id'])==evidence.read_bound(unit['files'][2]).decode()
actual=yaml.safe_load(evidence.read_bound(preflight['runs'][0]['parameters']))
prior=yaml.safe_load((S/'audits/navigation_n01_root_launch_preflight_r02/astra_navigation_n01_dwa_common_route/parameters.yaml').read_text())
dwa_new={k:v for k,v in actual.items() if k.startswith('/move_base/')}
dwa_old={k:v for k,v in prior.items() if k.startswith('/move_base/')}
changes={k:[dwa_old.get(k),dwa_new.get(k)] for k in set(dwa_old)|set(dwa_new) if dwa_old.get(k)!=dwa_new.get(k)}
assert changes=={'/move_base/DWAPlannerROS/acc_lim_y':[0.0,2.5]}
assert actual['/move_base/base_local_planner']=='dwa_local_planner/DWAPlannerROS'
assert actual['/move_base/base_global_planner']=='astra_common_route/FixedRouteGlobalPlanner'
assert hashlib.sha256(actual['/robot_description'].encode()).hexdigest()==unit['files'][1]['sha256']
evidence.validate_robot_description(actual['/robot_description'],value)
assert actual['/astra_navigation_coordinator/method']=='dwa_common_route'
assert actual['/astra_navigation_collector/contract_sha256']==unit['files'][0]['sha256']
assert not any(k.startswith('/astra_frozen_r2_method/') for k in actual)
assert '/move_base' in value['required_nodes'] and '/astra_frozen_r2_method' not in value['required_nodes']
assert {'/move_base/DWAPlannerROS/local_plan','/move_base/DWAPlannerROS/global_plan'}<=set(value['required_before_arm'])
conversion=load(unit['conversion_manifest']['path'])
assert conversion['input_urdf']==unit['files'][1] and conversion['converted_sdf']==unit['converted_sdf']
assert dense.conversion_resources(unit['conversion_manifest']['path'])==unit['conversion_resources']
for row in unit['files']+unit['conversion_resources']:evidence.read_bound(row)
config_proof=load(S/'audits/navigation_n01_dwa_diagnosis/configuration_unique_diff.json')
for key in ('old','new'):evidence.read_bound(config_proof[key])
assert config_proof['changed_fields']==[{'field':'DWAPlannerROS.acc_lim_y','before':0.0,'after':2.5}]

old_scene=S/'scenes/navigation_n01_clear_r03';new_scene=manager.SCENE
scene_old=yaml.safe_load((old_scene/'scene_manifest.yaml').read_text());scene_new=yaml.safe_load((new_scene/'scene_manifest.yaml').read_text())
assert scene_new['seed']==260926034 and scene_old['seed']==260926026
scene_new['seed']=scene_old['seed'];assert scene_new==scene_old
for path in [old_scene/'world.sdf']+list((old_scene/'common_bundle').rglob('*')):
    if path.is_file():assert path.read_bytes()==(new_scene/path.relative_to(old_scene)).read_bytes()
facts=passive_manager.native_review_facts(old)
assert facts['native_reference_pass'] and facts['upstream_source_review_pass']
for row in old['engine_module_bindings']+old['engine_source_bindings']:evidence.read_bound(row)

baseline_function=next(n for n in ast.parse(Path(manager.__file__).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='capture_baseline_runtime')
prior_function=next(n for n in ast.parse(Path(previous_manager.__file__).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='capture_baseline_runtime')
assert ast.dump(baseline_function)==ast.dump(prior_function)
audit={'schema':'astra_navigation_n03_minimal_source_audit_v1','pass':True,
    'runner_equivalence':bind(P/'runner_normalized_equivalence.json'),'prepared':bind(prepared_path),'actual_preflight':bind(preflight_path),
    'all_move_base_parameter_changes':changes,'configuration_unique_diff':bind(S/'audits/navigation_n01_dwa_diagnosis/configuration_unique_diff.json'),
    'independent_raw_native_diagnosis':bind(S/'audits/navigation_n01_dwa_diagnosis/diagnosis.json'),
    'same_world_maps_route_bytes':True,'scene_manifest_only_change':'seed026 to034',
    'actual_baseline_final_map_function_unchanged':True,
    'no_ROS':True,'no_seed_allocation':True,'blocking_findings':[],
    'scope':'Single actual-DWA sampling-factor clear-fixture diagnosis. N02 image policy inherited separately; N01 result remains immutable.'}
write('audit.json',audit)

paths=set(manager.required_resources())
paths.update(Path(r['path']).resolve() for r in unit['files']+unit['conversion_resources'])
paths.update(Path(r['path']).resolve() for r in old['engine_module_bindings']+old['engine_source_bindings'])
paths.update(Path(old[k]['path']).resolve() for k in ('native_engine_report','native_engine_records','upstream_source_manifest'))
paths.update([old_review_path,prepared_path,preflight_path,Path(preflight['runs'][0]['parameters']['path']),Path(__file__),
    P/'audit.json',P/'runner_normalized_equivalence.json',S/'audits/navigation_n03_root_stage_copy.json',
    S/'audits/navigation_n01_dwa_diagnosis/diagnosis.json',S/'audits/navigation_n01_dwa_diagnosis/configuration_unique_diff.json',
    S/'audits/navigation_n01_dwa_diagnosis/native_probe_verification.json',S/'audits/navigation_n02_coordinator_root_independent.json'])
resources=[bind(p) for p in sorted(paths)]
review={k:old[k] for k in ('schema','navigation_lifecycle_authority','robot_pose_velocity_writes','actor_updates',
    'native_engine_report','native_engine_records','upstream_source_manifest','engine_source_bindings','gazebo_callback_order_review','engine_module_bindings',
    'qualified_passive_evidence_prerequisite')}
review.update(stage=manager.NAME,status='independently_reviewed',
    reviewer='/root/environment_audit independently reviewed root-owned N03 runner/stage clone and actual expanded configuration; root selected/verified new single-parameter config and independently reviewed reused coordinator/launch. N02 helper/collector review is inherited by exact hash.',
    resources=resources,source_audit=bind(P/'audit.json'),prepared=bind(prepared_path),actual_launch_preflight=bind(preflight_path),
    scope=audit['scope'],inherited_source_review=bind(old_review_path),
    writer_review={**old['writer_review'],'sensor_method':'Inactive in N03; actual move_base/DWA owns private method output. Public coordinator and physical driver unchanged.'},
    runtime_obligations=[
        'New root release/registry; record effective acc_lim_y2.5 and hard min/max_y0. Keep real /odom and strict nonzero-y command rejection.',
        'Same owned move_base PID/start ticks, exact DWA/common-global-planner loaded ELF and configured classes must pass at final pause; configuration alone is not actual plugin qualification.',
        'Actual source-window2ms geometry/contact/native-model maps and five-second terminal drain remain mandatory.',
        'Retain algorithm failure and supplementary image limitations; do not reclassify original N01 DWA.'],
    preserved_limitations=[
        'acc_lim_y2.5 is a virtual planner dynamic-window allowance, not physical lateral actuation. Zero lateral sampling only within the declared fixed50ms input domain; |vy|>.125 synthetic counterexamples remain.',
        '393 saved odometry inputs passed the offline candidate-domain check; observed maxima are not future bounds and this does not guarantee navigation success.',
        'Changes DWA candidate set and potentially scoring outcome; single new diagnostic configuration, not a tuned-performance or comparator-superiority result.',
        'N02 supplementary-image policy is an inherited measurement change, independently disclosed from the DWA behavioral factor.',
        'Engineering swept-clearance interpretation is not formal or real continuous physical safety.'],
    execution_authorized=False,seed_allocation_authorized=False,
    pending_before_execution=['Root exact release/verify_release and separate registry reservation; source review grants no authority.'],review_does_not_grant_authority=True)
write('source_review.json',review)
write('release_required_resources.json',{'schema':'astra_navigation_review_resources_v1','stage':manager.NAME,
    'resources':resources+[bind(P/'source_review.json')],'execution_authorized':False})
print(json.dumps({'review':bind(P/'source_review.json'),'resources':len(resources),'active_runtime_paths':len(manager.required_resources()),'blocking_findings':[]},indent=2))
