"""Expand each N05 launch without opening a ROS master or simulator."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import yaml

S=Path(__file__).resolve().parents[1]
W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n05 as manager
import navigation_evidence_n05 as q

out=S/'audits/navigation_n05_root_launch_preflight_r01'
out.mkdir(exist_ok=False)
prepared_path=S/'contracts/navigation_n05_prepared_r01/prepared.json'
prepared=json.loads(prepared_path.read_text())
assert prepared['stage']==manager.NAME and len(prepared['units'])==24
rows=[]
for unit in prepared['units']:
    folder=out/unit['run_id'];folder.mkdir()
    value=json.loads(q.read_bound(unit['files'][0]))
    process=subprocess.run(['roslaunch','--dump-params',unit['files'][2]['path']],
                           capture_output=True,timeout=30,env=dict(os.environ))
    (folder/'parameters.yaml').write_bytes(process.stdout)
    (folder/'stderr.log').write_bytes(process.stderr)
    if process.returncode:raise RuntimeError((unit['run_id'],process.stderr.decode(errors='replace')))
    assert manager.block_for(unit['run_id'])['seed']==str(unit['seed'])
    actors=q.event_actor.world_actors(q.read_bound(value['world']).decode())
    assert all((a['run_id'],a['seed'],a['method'])==(unit['run_id'],unit['seed'],unit['method']) for a in actors.values())
    params=yaml.safe_load(process.stdout)
    urdf=params['/robot_description']
    q.validate_robot_description(urdf,value)
    actual=hashlib.sha256(urdf.encode()).hexdigest()
    assert actual==value['robot_description']['sha256']
    assert params['/astra_navigation_collector/contract_path']==str(S/'runs'/unit['run_id']/'contract.json')
    assert params['/astra_navigation_collector/contract_sha256']==unit['files'][0]['sha256']
    assert params['/astra_navigation_coordinator/method']==unit['method']
    assert params['/astra_route_runtime/method']==unit['method']
    assert params['/astra_route_runtime/seed']==unit['seed']
    assert params['/use_sim_time'] is True and params['/m2_gazebo/simulation_only'] is True
    assert not any(k.startswith('/move_base/') or k.startswith('/astra_frozen_r2_method/') for k in params)
    rows.append(dict(run_id=unit['run_id'],seed=unit['seed'],method=unit['method'],
                     actor_count=len(actors),pass_=True,parameter_file=q.binding(folder/'parameters.yaml'),
                     urdf_sha256=actual))
report=dict(status='all24_actual_launch_expansions_exact_contract_URDF_actor_and_method_parameters_pass',
            episodes=len(rows),actor_rows=sum(r['actor_count'] for r in rows),runs=rows,
            no_ROS_master_or_Gazebo_started=True,seeds_allocated=False,
            prepared=q.binding(prepared_path),runner=q.binding(Path(manager.__file__)))
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(status=report['status'],episodes=len(rows),actor_rows=report['actor_rows'])))
