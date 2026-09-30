import json,os,subprocess,sys,hashlib
from pathlib import Path
import yaml
S=Path(__file__).resolve().parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n01 as manager
import navigation_evidence as q
root=S/'audits/navigation_n01_root_launch_preflight_r01';root.mkdir(exist_ok=False)
prepared=json.loads((S/'contracts/navigation_n01_prepared_r01/prepared.json').read_bytes());rows=[]
for unit in prepared['units']:
 row=root/unit['run_id'];row.mkdir();value=json.loads(q.read_bound(unit['files'][0]))
 result=subprocess.run(['roslaunch','--dump-params',unit['files'][2]['path']],capture_output=True,timeout=30,env=dict(os.environ))
 (row/'parameters.yaml').write_bytes(result.stdout);(row/'stderr.log').write_bytes(result.stderr)
 assert result.returncode==0,result.stderr
 params=yaml.safe_load(result.stdout);urdf=params['/robot_description'];q.validate_robot_description(urdf,value)
 actual=hashlib.sha256(urdf.encode()).hexdigest();assert actual==value['robot_description']['sha256'],(unit['method'],actual,value['robot_description']['sha256'])
 assert params['/astra_navigation_collector/contract_path']==str(S/'runs'/unit['run_id']/'contract.json')
 assert params['/astra_navigation_collector/contract_sha256']==unit['files'][0]['sha256']
 assert params['/astra_navigation_coordinator/method']==unit['method']
 if unit['method'] in ('actual_teb_common_route','dwa_common_route'):
  assert params['/move_base/base_global_planner']=='astra_common_route/FixedRouteGlobalPlanner'
  expected='teb_local_planner/TebLocalPlannerROS' if unit['method']=='actual_teb_common_route' else 'dwa_local_planner/DWAPlannerROS'
  assert params['/move_base/base_local_planner']==expected
 rows.append(dict(run_id=unit['run_id'],pass_=True,parameters=q.binding(row/'parameters.yaml'),robot_description_sha256=actual))
report=dict(status='four_actual_launch_expansions_exact_contract_URDF_and_method_parameters_pass',runs=rows,no_ROS_or_allocation=True,prepared=q.binding(S/'contracts/navigation_n01_prepared_r01/prepared.json'),runner=q.binding(Path(manager.__file__)))
(root/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
