"""Read-only N06 final-review scope. Not a source review or execution release."""
import ast
import hashlib
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo'
def bind(p):
 p=Path(p).resolve();data=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def write(name,value):(A/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
prep_path=S/'contracts/navigation_n06_prepared_r01/prepared.json';prep=json.loads(prep_path.read_text())
prior_path=S/'audits/navigation_n06_preparation_independent/review.json';prior=json.loads(prior_path.read_text())
manifest=json.loads(Path(prior['resource_manifest']['path']).read_text())
assert all(bind(r['path'])['sha256']==r['sha256'] for r in manifest['resources'])
old_path=S/'audits/navigation_n05_root_release/source_review.json';old=json.loads(old_path.read_text())
roots=[G/'scripts'/name for name in ('astra_navigation_collector_n06.py','astra_navigation_coordinator_n06.py','astra_common_route_provider.py','astra_route_coordinated_runtime.py','astra_route_lookahead_runtime_n06.py')]
roots += [P/name for name in ('prepare_navigation_n06_static.py','navigation_evidence_n06.py','navigation_coordinator_n06.py')]
seen=set();pending=list(roots);edges=[]
while pending:
 path=pending.pop()
 if path in seen:continue
 seen.add(path)
 for node in ast.walk(ast.parse(path.read_text())):
  names=[r.name for r in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   candidates=[P/(name.split('.')[0]+'.py'),G/'scripts'/(name.split('.')[0]+'.py')]
   for local in candidates:
    if local.is_file():edges.append({'from':str(path),'to':str(local),'line':node.lineno});pending.append(local);break
# Static imports include guarded baseline/actor paths: do not imply all are executed.
direct={str(p.resolve()):bind(p) for p in roots}
config_files=[G/'launch/astra_common_navigation_n06.launch',G/'urdf/m2_physical_p05.urdf.xacro',W/'scripts/astra_env.sh',G/'package.xml',G/'CMakeLists.txt']
scan=[G/'urdf/m2_physical_p05.urdf.xacro'];seen_xml=set()
while scan:
 path=scan.pop()
 if path in seen_xml:continue
 seen_xml.add(path)
 for node in ET.fromstring(path.read_text()).iter():
  if node.tag.endswith('include') and 'filename' in node.attrib:
   value=node.attrib['filename'].replace('$(find m2_gazebo)',str(G))
   if '$(' in value:continue
   child=Path(value);child=child if child.is_absolute() else path.parent/child
   if child.is_file():scan.append(child);config_files.append(child)
native_sources=[G/'src'/name for name in ('astra_physical_ackermann_p05_plugin.cpp','astra_dense_evidence_plugin.cpp','astra_physics_metadata_p03_plugin.cpp')]
for path in list(native_sources):
 for name in re.findall(r'#include\s+["<](m2_gazebo/[^">]+)',path.read_text()):
  child=G/'include'/name
  if child.exists():native_sources.append(child)
unit_inputs=[];world_plugins=set();robot_plugins=set();geometry_pairs=[]
for unit in prep['units']:
 value=json.loads(Path(unit['contract']['path']).read_text())
 assert value['stage']=='astra_navigation_n06_lookahead_static_pilot'
 assert value['method'] in ('astra_analytic_route_v1','astra_analytic_lookahead_1p2_n06')
 assert 'actor' not in ' '.join(value['expected_models'])
 for key in ('contract','launch','gate_expectation','robot_description','world'):
  assert bind(unit[key]['path'])['sha256']==unit[key]['sha256'];unit_inputs.append(unit[key])
 for key in ('scene_manifest','shared_bundle'):unit_inputs.append(value[key])
 world=ET.fromstring(Path(unit['world']['path']).read_text());robot=ET.fromstring(Path(unit['robot_description']['path']).read_text())
 world_plugins.update(p.get('filename') for p in world.iter('plugin'));robot_plugins.update(p.get('filename') for p in robot.iter('plugin'))
 geometry_pairs.append({'run_id':unit['run_id'],'method':unit['method'],'seed':unit['seed'],'scene':value['scene_manifest'],'world':unit['world'],'robot_description':unit['robot_description'],'camera_topic':value['camera_topic'],'allowed_nodes':value['allowed_nodes'],'recorded_topic_count':len(value['record_topics'])})
assert len(geometry_pairs)==8
native=[r for r in old['engine_module_bindings'] if r.get('role')!='actor_binaries']
for row in native:assert bind(row['path'])['sha256']==row['sha256']
required={'schema':'astra_n06_source_scope_bindings_v1','is_final_source_review':False,
 'direct_runtime_and_preparation_python':[bind(p) for p in roots],
 'recursive_static_local_import_superset':[bind(p) for p in sorted(seen)],
 'recursive_import_edges':edges,
 'launch_robot_environment_build_context':[bind(p) for p in sorted(set(config_files))],
 'custom_native_sources':[bind(p) for p in sorted(set(native_sources))],
 'inherited_native_reference_candidates_excluding_actor':native,
 'prepared_unit_inputs':list({r['path']:r for r in unit_inputs}.values()),
 'pending_r03_modules':['run_navigation_n06_manager_r03.py','navigation_n06_integrity_r03.py','navigation_n06_process_r03.py','navigation_n06_admin_plan_r03.py'],
 'pending_external_runtime_closure':'canonical Gazebo/ROS executables, ELF DT_NEEDED closure, ROS/Python imports and interpreter prefix; author disk closure is not yet frozen',
 'pending_bundle_contents':'all common_bundle raster/map/guide files and original four D04 resource/negative-witness closures, not only common_route.json',
 'superset_note':'Guarded imports/default launch arguments do not imply active PCT/R2/TEB/DWA/actor behavior. All future source/config resources must nevertheless be hash-covered by final review.'}
write('required_source_bindings.json',required)
boundaries=[
 {'id':'stage_and_factor','observed':'N06 eight exact identities, four static pairs, lookahead0.7/1.2 only; original analytic backup selector, no N05 preview eligibility.','final_check':'Recheck actual prepared8 and exhaustive treatment/control source whitelist; use stage astra_navigation_n06_lookahead_static_pilot in every final API.'},
 {'id':'lifecycle','observed':'Collector is required=true; paused spawn once, recorder barrier/pre maps, unpause, readiness/arm before5s, terminal stop/pause, stable paused2ms watermark then final maps.','final_check':'Source gateway phase and exact journal match; no reset/delete/set-model-state/runtime unloading. Failure and supplementary image paths retain raw and all available final evidence.'},
 {'id':'coordinator','observed':'Only public /cmd_vel_navigation owner, private method command remap; queued future odom until visible clock, unchanged200ms guard; first actual base terminal authority.','final_check':'Inherited terminal immutability/deadline callback-order proof and exact N06 method whitelist; no route-node local complete/timeout/fault substitutes for actual public source outcome.'},
 {'id':'planner_sensor_and_route','observed':'scan+timestamp TF+odom+joint, verified map/path bundle; treatment1.2 is exact control clone with identity/config changes.','final_check':'Bind core, geometry, buffer, frame, map, predictor and static dependencies. No oracle planner transport, no actor/R2/checkpoint inference activation. Retain source/availability and prepublish drops as diagnostics, not invented source stamps.'},
 {'id':'evidence_clock_and_failure','observed':'Dense bag/raw, model roster, services, process graph, command ingress and images retain distinct source/receiver roles; common freeze closes callbacks.','final_check':'Formal integer window, paused source watermark/tails, actual map_from_odom and raw geometry task versus image integrity; exact nonempty assessment source/sweep closure in r03. No known missing evidence silently bypassed.'},
 {'id':'physical_writer','observed':'P05 effort controller calls SetForce at Begin, publishes actual End; dense/P03 observers publish without robot pose/velocity setter. World is static, so actor native role is inactive.','final_check':'Bind actual unchanged driver/helper/three customlibs, ODE/Gazebo API sources/native tests, sensor and graphics exception proof. No robot pose/velocity writes is narrower than no force writes.'},
 {'id':'process_and_administration','observed':'r01/r02 manager negative reviews remain frozen; r03 is not yet reviewed.','final_check':'Prioritize r03 focused callback/receipt/metadata closure, exact source/native/launch/conversion and cleanup14830/31; no stale r02 acceptance. Admin plan remains separate from actual CSV apply.'},
 {'id':'runtime_correspondence','observed':'Eight double-conversion saved provenance independently passed offline.','final_check':'Independent frozen disk runtime closure then live pre/final maps correspond to saved converter sdformat/native roles. Diagnostic converter pass alone cannot establish live correspondence.'}]
write('scope.json',{'schema':'astra_navigation_n06_final_source_lifecycle_review_scope_v1','status':'review_scope_prepared_final_review_pending','stage':'astra_navigation_n06_lookahead_static_pilot',
 'independent_source_review_pass':False,'execution_authorized':False,'seed_allocation_authorized':False,
 'preparation':bind(prep_path),'preparation_prior_independent_review':bind(prior_path),'preparation_bound_resources_rehashed':len(manifest['resources']),
 'N05_reference_only_not_stage_authority':bind(old_path),'required_bindings':bind(A/'required_source_bindings.json'),
 'unit_observations':geometry_pairs,'actual_world_plugin_filenames':sorted(world_plugins),'actual_robot_plugin_filenames':sorted(robot_plugins),
 'boundaries':boundaries,'pending_gaps':['r03 final author freeze, independent regression and final resource binding.', 'Actual frozen N06 runtime dependency closure and independent review.', 'New N06 source-review API with final release source/config roster; no N05 stage substitution.', 'Future read-only exact release verification and fresh identity check; no authority created here.', 'Live source joins, physical closed-loop effect, model/mappedELF correspondence remain unobserved.'],
 'inherited_conversion_review':bind(S/'audits/navigation_n06_conversion_independent/review.json'),
 'r02_negative_review':bind(S/'audits/navigation_n06_manager_independent/r02/review.json'),
 'no_actions':['No ROS/Gazebo, new launch dump, native build, simulation or training.', 'No actual execution release, run directory, seed registration or CSV write.', 'No r03 intermediate bytes read or certified.']})
(A/'scope.md').write_text('''# N06 最终 source/lifecycle 独审范围（未通过声明）

本文件只准备审查范围，`independent_source_review_pass=false`。N06 必须使用自身 stage `astra_navigation_n06_lookahead_static_pilot`；N05 source_review 仅作源/native 继承参考，不复制其身份、动态 actor 或结果资格。

已只读重哈希 prior preparation 资源，复核八行实际 contract/launch/world/URDF，提取真实运行入口、递归本地 import 静态超集、xacro include、三个 custom native plugin 与非 actor native 角色。完整表见 required_source_bindings.json。静态超集含分支未激活的旧依赖，不声称这些分支在 N06 运行。

最终必须覆盖：①四个新 D04 static parent 与原失败 witness、八份 input/conversion、0.7→1.2 唯一 factor；②required collector 的唯一 spawn/unpause/pause gateway 与失败取证；③公共 base/source 终态及未来 odom 排队；④实际 scan/TF/odom/joint、map/path 和纯 analytic route core；⑤dense/contacts/physical source、有限 watermark drain、callback freeze、任务/安全/图像完整性分离；⑥P05 仅有限力驱动和纯 observer，native ODE/Gazebo/libs 与 graphics 有界例外；⑦新 r03 one-shot/process/receipt/行政投影；⑧真实运行 ELF/maps 与离线 converter 的对应。

仍缺：r03 最终独审；N06 runtime 磁盘闭包冻结与独审；新 N06 完整资源 source-review API；未来 exact release/freshness 检查。现场物理、实际回调 source join、mapped ELF 与闭环效果尚未发生。本轮未重新执行 launch dump、native build、ROS 或仿真；prepared 的既有真实 XML 展开仅引用此前独审记录。r03 冻结后优先回归其两个回执负例，不扩测试到无关算法。
''')
print(json.dumps({'scope':bind(A/'scope.json'),'required_bindings':bind(A/'required_source_bindings.json'),'static_local_module_count':len(seen)},indent=2))
