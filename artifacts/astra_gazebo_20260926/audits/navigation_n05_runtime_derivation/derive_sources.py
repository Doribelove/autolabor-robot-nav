"""Create new N05 files only; preserve all N04 execution bytes."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

S = Path(__file__).resolve().parents[2]
W = S / 'workspace'
records = []

def derive(source, target, transform):
    source, target = W/source, W/target
    original = source.read_text()
    result = transform(original)
    if target.exists():
        raise FileExistsError(target)
    target.write_text(result)
    target.chmod(source.stat().st_mode & 0o777)
    records.append(dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                        target=str(target), target_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))

def replace(text, old, new, count=1):
    if text.count(old) != count:
        raise ValueError((old, text.count(old), count))
    return text.replace(old, new)

def n05(text):
    return text.replace('n04', 'n05').replace('N04', 'N05')

for filename in ('evaluate_episode', 'dense_sweep', 'navigation_evidence'):
    def convert(text, filename=filename):
        text = n05(text)
        if filename == 'navigation_evidence':
            text = text.replace('navigation_coordinator_n02', 'navigation_coordinator_n05')
            text = replace(text, "METHODS=('astra_analytic_route_v1',", "METHODS=('astra_analytic_preview_eligible_n05','astra_analytic_route_v1',")
        return text
    derive(f'scripts/astra_gazebo/{filename}_n04.py', f'scripts/astra_gazebo/{filename}_n05.py', convert)

derive('src/simulation/m2_gazebo/scripts/astra_navigation_collector_n04.py',
       'src/simulation/m2_gazebo/scripts/astra_navigation_collector_n05.py',
       lambda text: n05(text).replace('14820', '14830'))
derive('scripts/astra_gazebo/navigation_coordinator_n02.py', 'scripts/astra_gazebo/navigation_coordinator_n05.py',
       lambda text: replace(text, "METHODS=('astra_analytic_route_v1',", "METHODS=('astra_analytic_preview_eligible_n05','astra_analytic_route_v1',"))
derive('src/simulation/m2_gazebo/scripts/astra_navigation_coordinator_n02.py',
       'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator_n05.py',
       lambda text: text.replace('navigation_coordinator_n02', 'navigation_coordinator_n05'))

def node(text):
    text = replace(text, 'Shared-gate adaptation of the frozen standalone route node. R2 is separate.',
                   'N05 analytic preview-eligibility treatment, with inherited shared-gate lifecycle.')
    text = replace(text, 'from route_candidate_core import RouteAwareRuntime,RouteConfig,OBSERVATION_SCHEMA,ACTION_NAMES,prepublish_guard,physical_backend',
                   'from route_candidate_core import RouteConfig,ACTION_NAMES,prepublish_guard,physical_backend\n    from route_preview_selection_n05 import PreviewEligibleRuntime,OBSERVATION_SCHEMA,METHOD')
    text = replace(text, "method not in ('astra_analytic_route_v1','astra_pct_route_v1')", 'method != METHOD')
    text = replace(text, 'core=RouteAwareRuntime(grid,route,urdf,config)', 'core=PreviewEligibleRuntime(grid,route,urdf,config)')
    begin = text.index("    if method=='astra_pct_route_v1':")
    end = text.index('    buffer=RouteSensorBuffer()', begin)
    text = text[:begin] + text[end:]
    text = replace(text, "'actor_inference_protocol':'SensorActor architecture only; deterministic choose_actions advantage_gate_margin0.10; proposed denotes its post-gate policy output'",
                   "'actor_inference_protocol':'none; analytic-only preview eligibility; original backup mask retained'")
    text = replace(text, "'actor_adapter_sha256':hashlib.sha256((workspace/'scripts/astra_gazebo/route_actor_adapter.py').read_bytes()).hexdigest()",
                   "'selection_adapter_sha256':hashlib.sha256((workspace/'scripts/astra_gazebo/route_preview_selection_n05.py').read_bytes()).hexdigest()")
    text = replace(text, "'prediction':built['prediction_audit'],'candidate_features'",
                   "'full_preview_clearances_m':built['full_preview_clearances_m'],'preview_selection_audit':built['preview_selection_audit'],\n                                'prediction':built['prediction_audit'],'candidate_features'")
    return text

derive('src/simulation/m2_gazebo/scripts/astra_route_coordinated_runtime.py',
       'src/simulation/m2_gazebo/scripts/astra_route_preview_runtime_n05.py', node)

def launch(text):
    text = text.replace('astra_navigation_coordinator_n02.py', 'astra_navigation_coordinator_n05.py')
    root = ET.fromstring(text)
    groups = root.findall('group')
    control = groups[0]
    control.set('if', "$(eval arg('method') == 'astra_analytic_route_v1')")
    treatment = ET.fromstring(ET.tostring(control))
    treatment.set('if', "$(eval arg('method') == 'astra_analytic_preview_eligible_n05')")
    treatment.find('node').set('type', 'astra_route_preview_runtime_n05.py')
    for old in groups[1:]: root.remove(old)
    root.insert(list(root).index(control)+1, treatment)
    for arg in root.findall('arg'):
        if arg.get('name') == 'sensor_lifecycle_evidence_topics': arg.set('value', '')
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'

derive('src/simulation/m2_gazebo/launch/astra_common_navigation_n02.launch',
       'src/simulation/m2_gazebo/launch/astra_common_navigation_n05.launch', launch)

def runner(text):
    text = n05(text).replace('navigation_coordinator_n02', 'navigation_coordinator_n05')
    text = text.replace('astra_common_navigation_n02.launch', 'astra_common_navigation_n05.launch')
    text = text.replace('263001001', '263002001').replace('14820', '14830').replace('14821', '14831')
    text = text.replace('four-method', 'two-method').replace('48-unit', '24-unit')
    text = replace(text, "NAME = 'astra_navigation_n05'", "NAME = 'astra_navigation_n05_preview_selection'")
    text = replace(text, "contracts/navigation_n05_design.json", "contracts/navigation_n05_preview_selection_design.json")
    text = replace(text, 'ce5afb9920eefb7a3c4d2bad2646405e977494a13cd61477bf248f3fa867a912',
                   '959b214b3f4a10da276ee46ce9c319c9f5ea8001551f9dc4626b97f9714baefc')
    text = replace(text, "value['maximum_episodes']!=48 or len(value['schedule'])!=48", "value['proposed_maximum_episodes']!=24 or len(value['schedule'])!=24")
    text = replace(text, 'schedule[index*4]', 'schedule[index*2]')
    text = replace(text, 'len(same_seed)!=4', 'len(same_seed)!=2')
    text = replace(text, "if method == 'astra_analytic_route_v1':", "if method in ('astra_analytic_route_v1', 'astra_analytic_preview_eligible_n05'):")
    text = replace(text, "'route_actor_adapter.py', 'frozen_r2_sensor_lifecycle.py')", "'route_actor_adapter.py', 'frozen_r2_sensor_lifecycle.py',\n        'route_preview_selection_n05.py', 'release_validation_session_n05.py')")
    text = replace(text, "'astra_route_coordinated_runtime.py', 'astra_frozen_r2_sensor_lifecycle.py')", "'astra_route_coordinated_runtime.py', 'astra_route_preview_runtime_n05.py', 'astra_frozen_r2_sensor_lifecycle.py')")
    text = replace(text, '    release, prepared, review = verify_release(release_path, release_sha)',
        "    # Live predecessor and identity checks stay outside the immutable-file cache.\n"
        "    predecessor = json.loads((STAGE/'queue/navigation_n04_ledger.json').read_text())\n"
        "    if predecessor.get('stage') != 'astra_navigation_n04' or predecessor.get('status') != 'terminal':\n"
        "        raise ValueError('N04 must be terminal before N05 execution')\n"
        "    for previous in predecessor['units']:\n"
        "        if common.owned_processes(STAGE/'runs'/previous['run_id']):\n"
        "            raise ValueError('N04 still has live owned processes')\n"
        "    from release_validation_session_n05 import UnchangedReleaseSession\n"
        "    release_session = UnchangedReleaseSession(sys.modules[__name__], release_path, release_sha)\n"
        "    release, prepared, review = release_session.verify_unchanged()")
    text = replace(text, '                verify_release(release_path, release_sha)', '                release_session.verify_unchanged()', count=2)
    return text

derive('scripts/astra_gazebo/run_navigation_n04.py', 'scripts/astra_gazebo/run_navigation_n05.py', runner)
(Path(__file__).parent/'derivation.json').write_text(json.dumps(dict(
    status='new_source_draft_not_built_not_executed_not_released', resources=records,
    note='Each source read once; no N04 mutation, no CMake mutation, no native build, no seeds allocated.'), indent=2)+'\n')
print(json.dumps(dict(new_files=len(records), status='draft')))
