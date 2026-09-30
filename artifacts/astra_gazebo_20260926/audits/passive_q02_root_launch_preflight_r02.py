import json, os, subprocess, sys
from pathlib import Path
import yaml
S=Path(__file__).resolve().parents[1]; W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import passive_sweep_q02 as q
import run_passive_sweep_q02 as manager
root=S/'audits/passive_q02_root_launch_preflight_r02';root.mkdir(exist_ok=False)
prepared=json.loads((S/'contracts/passive_sweep_q02_prepared_r01/prepared.json').read_bytes())
results=[]
for unit in prepared['units']:
 value=json.loads(q.read_bound(unit['files'][0])); q.validate_contract(value)
 row=root/unit['run_id'];row.mkdir()
 launch=q.read_bound(unit['files'][3]).decode()
 # Only the textfile consumed by offline dump-params points to the prepared copy. Runtime paths remain unchanged.
 old=str(S/'runs'/unit['run_id']/'robot_description.xml');new=unit['files'][2]['path']
 assert launch.count(old)==1
 (row/'offline.launch').write_text(launch.replace(old,new))
 command=['roslaunch','--dump-params',str(row/'offline.launch')]
 result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=25,env=os.environ.copy())
 (row/'parameters.yaml').write_bytes(result.stdout);(row/'stderr.log').write_bytes(result.stderr)
 assert result.returncode==0,result.stderr
 params=yaml.safe_load(result.stdout)
 actual=params['/robot_description']; expected=q.read_bound(unit['files'][2]).decode()
 assert actual.strip()==expected.strip(),'saved/runtime expanded URDF differs'
 q.validate_robot_description(actual,value)
 for node in ('passive_command','passive_collector'):
  assert params['/'+node+'/contract_path']==str(S/'runs'/unit['run_id']/'contract.json')
  assert params['/'+node+'/contract_sha256']==unit['files'][0]['sha256']
 assert params['/passive_collector/output']==str(S/'runs'/unit['run_id'])
 results.append(dict(run_id=unit['run_id'],seed=unit['seed'],pass_=True,contract_sha256=unit['files'][0]['sha256'],parameters=q.binding(row/'parameters.yaml')))
report=dict(status='two_actual_launch_expansions_match_frozen_prepared_contracts',runs=results,no_ros_started=True,no_episode_allocated=True,offline_textfile_mapping_only=True,resources=[q.binding(Path(__file__)),q.binding(S/'contracts/passive_sweep_q02_prepared_r01/prepared.json')])
(root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
