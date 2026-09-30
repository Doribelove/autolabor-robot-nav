"""Assemble an offline source review, never a release or allocation."""
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

P = Path(__file__).resolve().parent
S = P.parents[1]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
import run_navigation_n02 as manager
import navigation_evidence_n02 as evidence
import run_passive_sweep_q05 as passive_manager
import dense_sweep_p04 as dense
import frozen_r2_sensor_lifecycle as r2


def bind(path):
    path = Path(path).resolve()
    return {'path':str(path),'sha256':evidence.sha(path)}


def load(path):
    return json.loads(Path(path).read_bytes())


def write(name, value):
    (P / name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


old_path = S / 'audits/navigation_n01_release_source_review/source_review.json'
old = load(old_path)
prepared_path = S / 'contracts/navigation_n02_prepared_r01/prepared.json'
prepared = load(prepared_path)
preflight_path = S / 'audits/navigation_n02_root_launch_preflight_r02/report.json'
preflight = load(preflight_path)
assert preflight['runner'] == bind(manager.__file__)
assert preflight['prepared'] == bind(prepared_path)
assert len(prepared['units']) == 1 and prepared['maximum_episodes'] == 1
unit = prepared['units'][0]
assert (unit['run_id'],unit['seed'],unit['method']) == manager.identities()[0]
assert prepared['execution_authorized'] is False
value = load(unit['files'][0]['path'])
evidence.validate_contract(value)
assert value['method'] == r2.METHOD and value['seed'] == 260926033
assert manager.render_launch(value,unit['files'][0]['sha256'],S/'runs'/unit['run_id']) == evidence.read_bound(unit['files'][2]).decode()
assert r2.authenticate_frozen(W)
assert evidence.sha(manager.CHECKPOINT) == r2.FROZEN_CHECKPOINT_SHA256
params_binding = preflight['runs'][0]['parameters']
params = yaml.safe_load(evidence.read_bound(params_binding))
assert hashlib.sha256(params['/robot_description'].encode()).hexdigest() == unit['files'][1]['sha256']
evidence.validate_robot_description(params['/robot_description'],value)
for key,wanted in {
    '/astra_navigation_coordinator/method':r2.METHOD,
    '/astra_navigation_collector/contract_sha256':unit['files'][0]['sha256'],
    '/astra_navigation_collector/contract_path':str(S/'runs'/unit['run_id']/'contract.json'),
    '/astra_frozen_r2_method/execution_method':r2.METHOD,
    '/astra_frozen_r2_method/method':'pct_scope_rl',
    '/astra_frozen_r2_method/candidate_variant':'r2_candidate_v060',
    '/astra_frozen_r2_method/actor_checkpoint_path':str(manager.CHECKPOINT),
    '/astra_frozen_r2_method/actor_checkpoint_sha256':r2.FROZEN_CHECKPOINT_SHA256,
    '/astra_frozen_r2_method/seed':260926033,
}.items():
    assert params[key] == wanted, key
five = {'/astra/frozen_r2_sensor_lifecycle/'+name for name in ('identity','input_receipt','trace','command_event','report')}
assert five <= set(value['record_topics']) and five <= set(value['required_before_arm'])
assert not any('/move_base/' in k for k in params)
launch = ET.parse(manager.LAUNCH).getroot()
collector = [n for n in launch.iter('node') if n.get('name') == 'astra_navigation_collector']
assert len(collector) == 1 and collector[0].get('required') == 'true'
assert not any(n.get('type') == 'spawn_model' for n in launch.iter('node'))

old_scene = S / 'scenes/navigation_n01_clear_r03'
new_scene = manager.SCENE
scene_old = yaml.safe_load((old_scene/'scene_manifest.yaml').read_text())
scene_new = yaml.safe_load((new_scene/'scene_manifest.yaml').read_text())
assert scene_new['seed'] == 260926033 and scene_old['seed'] == 260926026
scene_new['seed'] = scene_old['seed']
assert scene_new == scene_old
same_scene = []
for original in [old_scene/'world.sdf']+sorted((old_scene/'common_bundle').rglob('*')):
    if original.is_file():
        new = new_scene / original.relative_to(old_scene)
        assert original.read_bytes() == new.read_bytes()
        same_scene.append({'old':bind(original),'new':bind(new)})

conversion = load(unit['conversion_manifest']['path'])
assert conversion['input_urdf'] == unit['files'][1]
assert conversion['converted_sdf'] == unit['converted_sdf']
assert dense.conversion_resources(unit['conversion_manifest']['path']) == unit['conversion_resources']
for row in unit['conversion_resources']:
    evidence.read_bound(row)

facts = passive_manager.native_review_facts(old)
assert facts['native_reference_pass'] and facts['upstream_source_review_pass']
roles = [r['role'] for r in old['engine_module_bindings']]
assert len(roles) == len(set(roles))
assert set(roles) == set(evidence.passive.ELF_ROLES.values()) - {'actor_binaries'}
for row in old['engine_module_bindings'] + old['engine_source_bindings']:
    evidence.read_bound(row)
for key in ('native_engine_report','native_engine_records','upstream_source_manifest'):
    evidence.read_bound(old[key])

root_review_path = S / 'audits/navigation_n02_coordinator_root_independent.json'
root_review = load(root_review_path)
for row in root_review['checks']:
    assert row['normalized_AST_equal'] is True
    assert evidence.sha(W/row['new']) == row['new_sha256']
    assert evidence.sha(W/row['original']) == row['old_sha256']
r2_review_path = S / 'audits/r2_sensor_lifecycle_independent/review_r1.json'
for row in load(r2_review_path)['sources']:
    evidence.read_bound(row)
assert '6 passed' in (P/'focused_tests.log').read_text()
assert '117 passed' in (P/'root_tests_independent_rerun.log').read_text()

active = manager.required_resources()
previous = {str(Path(r['path']).resolve()):r['sha256'] for r in old['resources']}
unchanged, new_paths = [], []
for path in active:
    row = bind(path)
    if row['path'] in previous:
        assert row['sha256'] == previous[row['path']], path
        unchanged.append(row)
    else:
        new_paths.append(row)

qualification_path = S / 'contracts/passive_q05_navigation_evidence_qualification.json'
qualification = load(qualification_path)
assert qualification['qualified_for_navigation_evidence'] is True
assert qualification['resolved_steering_kd'] == 6.0
for row in qualification['evidence_bindings']:
    evidence.read_bound(row)

audit = dict(schema='astra_navigation_n02_offline_source_audit_v1',pass_=True,
    reviewed_root_sources=[bind(manager.__file__),bind(evidence.__file__),bind(W/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_n02.py')],
    focused_tests=bind(P/'focused_tests.log'),focused_test_count=6,
    root_tests_rerun=bind(P/'root_tests_independent_rerun.log'),root_test_count=117,
    actual_saved_launch_preflight=bind(preflight_path),prepared=bind(prepared_path),
    original_r2_source_and_checkpoint_bindings=r2.authenticate_frozen(W)+[bind(manager.CHECKPOINT)],
    sensor_lifecycle_independent_r1_review=bind(r2_review_path),
    coordinator_core_and_node_independently_reviewed_by_root=bind(root_review_path),
    actual_r02_parameters=bind(params_binding['path']),same_world_map_route_bytes=same_scene,
    scene_manifest_only_semantic_change={'seed':[260926026,260926033]},
    reused_unchanged_active_resource_count=len(unchanged),new_active_paths=new_paths,
    native_review_facts=facts,engine_roles=roles,
    image_failure_result='Actual new collector terminal AST reaches stop/pause/drain/roster/final maps for saved N01 0.26s one-frame fault, zero frames and export error. Same input preserves old N01 image rejection. Missing dense tail still prevents final marker.',
    method_graph_scope='Actual master subscriptions/publishers sampled under trusted authorized launch; source review, not graph registry, covers absence of simulator service clients and dynamic truth file inputs.',
    manager_return_scope='Records execute() return separately from roslaunch return and unknown later host shell termination; execute body semantics unchanged except new stage/one-unit schedule.',
    no_ROS=True,no_execution=True,no_seed_allocation=True,blocking_findings=[])
write('audit.json',audit)

resource_paths = set(active)
resource_paths.update(Path(r['path']).resolve() for r in unit['files']+unit['conversion_resources'])
resource_paths.update(Path(r['path']).resolve() for r in old['engine_module_bindings']+old['engine_source_bindings'])
resource_paths.update(Path(old[k]['path']).resolve() for k in ('native_engine_report','native_engine_records','upstream_source_manifest'))
resource_paths.update([old_path,prepared_path,preflight_path,Path(params_binding['path']),root_review_path,r2_review_path,
    S/'audits/r2_sensor_lifecycle_implementation/implementation_report_r1.json',
    S/'audits/navigation_n02_sensor_lifecycle_integration/implementation_report.json',
    S/'audits/dense_sweep_p04_independent/review.json',
    S/'audits/graphics_mapping_independent/review.json',
    S/'audits/navigation_n02_root_launch_preflight_r02.py',qualification_path,
    P/'test_focused_review.py',P/'focused_tests.log',P/'root_tests_independent_rerun.log',P/'audit.json',Path(__file__)])
resources = [bind(p) for p in sorted(resource_paths)]
review = {key:old[key] for key in ('schema','navigation_lifecycle_authority','robot_pose_velocity_writes','actor_updates',
    'native_engine_report','native_engine_records','upstream_source_manifest','engine_source_bindings',
    'gazebo_callback_order_review','engine_module_bindings')}
review.update(stage=manager.NAME,status='independently_reviewed',
    reviewer='/root/environment_audit independently reviewed root-owned N02 runner/evidence/collector. Root independently reviewed this agent authored coordinator core/node and actual launch integration. R2 lifecycle r1 separately independently reviewed by this agent.',
    review_division={'root_owned_independently_reviewed_here':audit['reviewed_root_sources'],
        'environment_agent_owned_reviewed_by_root':bind(root_review_path),
        'launch_actual_expansion_reviewed_by_root_and_rechecked_here':bind(preflight_path),
        'sensor_adapter_designed_by_design_audit_independently_reviewed_here':bind(r2_review_path)},
    resources=resources,source_audit=bind(P/'audit.json'),prepared=bind(prepared_path),actual_launch_preflight=bind(preflight_path),
    qualified_passive_evidence_prerequisite=bind(qualification_path),
    scope='One sensor-only frozen-R2 lifecycle clear-road integration. No new learned model, shared-route consumer equivalence, performance or formal physical-safety claim.',
    writer_review={**old['writer_review'],'actor_review_field':'Inherited actor Begin review remains inactive in N02 static world',
        'sensor_method':'Only sensor/clock/map/route/gate/status inputs and private command output; actual graph is a runtime integrity gate. No simulator services or truth-state inputs in the R2 method node.'},
    runtime_obligations=[
        'Root final exact release and registry must bind this source review, all prepared fixture launch/conversion files, checkpoint and runtime dependencies.',
        'Collector alone spawns/unpauses/pauses; coordinator owns public navigation command; R2 owns only private method command.',
        'Actual same-process pre/final Gazebo ELF inventory, source join, full2ms contact/native geometry and <=5s final drain must pass.',
        'N02 actual sensor-only subscriptions/publishers and five lifecycle evidence topic recorder connections must be observed; this source review does not claim live success.',
        'Images are supplementary with exact saved/error/distinct-source coverage; absence never upgrades a method fault or excuses dense evidence loss.'],
    preserved_limitations=[
        'Frozen R2 consumes its original straight-reference/static axis-aligned algorithm; shared full path receipt is audit_only, not actual path consumption.',
        'Original actor/runtime/execute_sensor/checkpoint bytes retained; only reviewed lifecycle/real-history/TF freshness/terminal integration differs from historical R2.',
        'Simulation odometry remains sensor-style ground-derived odometry; no real-vehicle/perception robustness or general sensor accuracy claim.',
        'This evaluator reports planner_input_mask_or_decision_integrity_assessed=false; preserved method traces need their own post-run audit.',
        'N01 failures, including DWA lateral command and image/final-map failure, remain immutable. No DWA configuration repair is included in N02.',
        'Sparse camera coverage is a disclosed visual limit. Pre-gate/zero-window infrastructure failures still cannot qualify a complete formal source window.',
        'Engineering normal-Euler extension is not real continuous physical/formal safety; Kd6 qualification is the existing sampled scope.'],
    execution_authorized=False,seed_allocation_authorized=False,
    pending_before_execution=['Root final exact N02 release and verify_release success; separate exact registry reservation.'],
    review_does_not_grant_authority=True)
write('source_review.json',review)
write('release_required_resources.json',{'schema':'astra_navigation_review_resources_v1','stage':manager.NAME,
    'resources':resources+[bind(P/'source_review.json')],'execution_authorized':False})
print(json.dumps({'source_review':bind(P/'source_review.json'),'resources':len(resources),
    'required_runtime_paths':len(active),'native_roles':len(roles),'unchanged_reused':len(unchanged),'new_paths':len(new_paths)},indent=2))
