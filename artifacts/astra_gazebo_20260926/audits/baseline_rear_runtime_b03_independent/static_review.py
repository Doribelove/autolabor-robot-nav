"""Read-only independent B03 manifest/config/launch/AST checks; no ROS master."""
from pathlib import Path
import ast
import hashlib
import json
import site
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

A = Path(__file__).resolve().parent
S = A.parent.parent
W = S / 'workspace'
P = W / 'scripts/astra_gazebo'
G = W / 'src/simulation/m2_gazebo'
manifest = json.loads((A.parent/'baseline_rear_runtime_b03/source_manifest.json').read_text())
records = sum((manifest[key] for key in ('owned_new_sources', 'read_only_referenced_dependencies', 'tests_and_reports')), [])
for row in records:
    raw = Path(row['path']).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == row['sha256'], row['path']
    assert len(raw) == row['bytes'], row['path']
def classes(path):
    return {node.name: ast.dump(node, include_attributes=False) for node in ast.parse(path.read_text()).body if isinstance(node, ast.ClassDef)}
assert classes(P/'navigation_coordinator_b03.py') == classes(P/'navigation_coordinator_n02.py')
old = yaml.safe_load((G/'config/astra_navigation_common_footprint.yaml').read_text())
new = yaml.safe_load((G/'config/astra_baseline_rear_b03.yaml').read_text())
errors = {}
for key in ('local_costmap', 'global_costmap'):
    expected = np.array(old[key]['footprint']) + [0.325, 0.]
    actual = np.array(new[key]['footprint'])
    assert actual.shape == (64, 2)
    errors[key] = float(np.max(np.abs(actual - expected)))
    assert errors[key] <= 1e-15
    assert new[key]['robot_base_frame'] == 'rear_axle_kinematic'
    assert new[key]['footprint_padding'] == old[key]['footprint_padding'] == 0.
expected = np.array(old['TebLocalPlannerROS']['footprint_model']['vertices']) + [0.325, 0.]
actual = np.array(new['TebLocalPlannerROS']['footprint_model']['vertices'])
assert actual.shape == (16, 2)
errors['teb_raw_16'] = float(np.max(np.abs(actual - expected)))
assert errors['teb_raw_16'] <= 1e-15
for key in ('TebLocalPlannerROS', 'DWAPlannerROS'):
    assert new[key]['odom_topic'] == '/astra/rear_odom'
    assert new[key]['xy_goal_tolerance'] == .15
    assert new[key]['yaw_goal_tolerance'] == .1
assert new['DWAPlannerROS']['latch_xy_goal_tolerance'] is False
assert new['FixedRouteGlobalPlanner']['route_service'] == '/astra/baseline_rear/get_plan'
launch = ET.parse(G/'launch/astra_baseline_rear_b03.launch').getroot()
nodes = {node.attrib['name']: node for node in launch.findall('node')}
loads = [node.attrib['file'] for node in nodes['move_base'].findall('rosparam')]
assert loads[-1] == '$(find m2_gazebo)/config/astra_baseline_rear_b03.yaml'
assert nodes['astra_baseline_rear_adapter_b03'].get('required') == 'true'
assert nodes['astra_navigation_collector'].get('required') == 'true'
assert {node.get('from'): node.get('to') for node in nodes['move_base'].findall('remap')} == {'cmd_vel':'/astra/method_cmd', 'odom':'/astra/rear_odom'}
assert launch.find("arg[@name='collector_script']").get('default') is None
assert launch.find("arg[@name='python_executable']").get('default') == '$(env THESIS_WS)/.venv_train_cuda/bin/python'
report = {
    'schema':'b03_independent_static_checks_v1',
    'status':'pass', 'author_manifest_records_verified':len(records),
    'core_class_AST_exact_equal_to_N02':True,
    'footprint_translation_max_abs_error_m':errors,
    'shared_internal_goal_overlay':{k:new[k] for k in ('DWAPlannerROS',)},
    'XML_load_order_required_nodes_and_remaps_pass':True,
    'actual_roslaunch_preflight_performed':False,
    'environment':{'executable':sys.executable,'realpath':str(Path(sys.executable).resolve()),'python':sys.version,
                   'user_site_enabled':site.ENABLE_USER_SITE,'numpy':np.__version__,'numpy_file':np.__file__},
}
(A/'static_checks.json').write_text(json.dumps(report,sort_keys=True,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,sort_keys=True))
