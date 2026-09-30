"""Use saved real launch parameter dumps as offline constructor inputs. No launch invoked."""
from pathlib import Path
import hashlib,json,sys,yaml
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
from common_route import load_bundle
from route_candidate_core import physical_backend,RouteConfig,RouteAwareRuntime
from route_preview_selection_n05 import PreviewEligibleRuntime
import run_navigation_n05 as manager
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:dict(path=str(Path(p).resolve()),sha256=sha(p))
root_report=S/'audits/navigation_n05_root_launch_preflight_r01/report.json';r=json.loads(root_report.read_bytes());schedule=manager.design()['schedule'];resources=[bind(root_report)];checks=[]
assert len(r['runs'])==24
for row,expected in zip(r['runs'],schedule):
 for k in ['run_id','seed','method']:assert row[k]==expected[k]
 p=Path(row['parameter_file']['path']);assert sha(p)==row['parameter_file']['sha256'];resources.append(bind(p));v=yaml.safe_load(p.read_bytes());urdf=v['/robot_description'];assert hashlib.sha256(urdf.encode()).hexdigest()==row['urdf_sha256']
 assert v['/use_sim_time'] is True and v['/m2_gazebo/simulation_only'] is True
 for prefix in ['/astra_route_runtime/','/astra_navigation_coordinator/']:
  for k in ['run_id','seed','method']:assert type(v[prefix+k]) is type(expected[k]) and v[prefix+k]==expected[k]
 bundle=Path(v['/astra_route_runtime/bundle']);assert sha(bundle/'common_route.json')==v['/astra_route_runtime/bundle_sha256'];grid,route,data=load_bundle(bundle)
 backend=physical_backend(urdf);assert backend['seed']==expected['seed'] and backend['library_filename']=='libastra_physical_ackermann_p05_plugin.so'
 assert float(backend['parameters_without_seed']['steeringKd'])==6.
 cls=RouteAwareRuntime if expected['method']=='astra_analytic_route_v1' else PreviewEligibleRuntime
 core=cls(grid,route,urdf,RouteConfig(maximum_episode_duration_s=v['/astra_route_runtime/maximum_episode_duration_s']))
 assert core.backend['seed']==expected['seed'] and core.config.maximum_episode_duration_s==22. and core.footprint.component_count==9
 checks.append(dict(run_id=expected['run_id'],seed=expected['seed'],method=expected['method'],actual_saved_parameters_used=True,offline_constructor_pass=True,backend_library=backend['library_filename'],full_collision_components=core.footprint.component_count,urdf_sha256=row['urdf_sha256']))
for i in range(12):assert checks[2*i]['urdf_sha256']==checks[2*i+1]['urdf_sha256']
# Native production and historical before values are root observations; independently rehash only saved/current bytes.
native_path=S/'audits/navigation_n05_native_build/build_result_r01.json';native=json.loads(native_path.read_bytes());resources.append(bind(native_path));assert sha(native['new_library']['path'])==native['new_library']['sha256'];resources.append(bind(native['new_library']['path']))
for row in native['inherited_binary_comparison']:
 assert sha(row['path'])==row['after']==row['before'];resources.append(bind(row['path']))
assert sha(native['build_log'])==native['build_log_sha256'];resources.append(bind(native['build_log']))
for p in [P/'route_candidate_core.py',P/'route_preview_selection_n05.py',P/'common_route.py'] :resources.append(bind(p))
out=dict(status='saved_actual_launch_parameters_offline_constructor_check_pass',rows=checks,paired_URDF_bytes_equal=True,independently_invoked_roslaunch=False,independently_built_native=False,new_and_inherited_ELF_current_hashes_checked=13,root_claims_only=['Actual roslaunch dump-params execution and its process exits','Native build execution and pre-build observation time'],resources=resources)
(A/'saved_launch_checks.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'status':out['status'],'constructors':len(checks),'current_ELFs_rehashed':13,'ROS_started':False}))
