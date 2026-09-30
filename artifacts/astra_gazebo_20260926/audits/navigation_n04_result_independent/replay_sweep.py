#!/usr/bin/env python3
import argparse,hashlib,json,os,subprocess,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);a=p.parse_args();R=S/'runs'/a.run_id;O=D/'rows'/a.run_id/'sweep_replay'
load=lambda p:json.loads(p.read_bytes())
ledger=load(S/'queue/navigation_n04_ledger.json');receipt=load(R/'receipt.json')
assert receipt['integrity_pass']is True and next(x['receipt']['sha256']for x in ledger['units']if x['run_id']==a.run_id)==hashlib.sha256((R/'receipt.json').read_bytes()).hexdigest()
raw=load(R/'collector_raw.json')
cmd=[str(W/'.venv/bin/python'),str(W/'scripts/astra_gazebo/dense_sweep_n04.py'),'--bag',str(R/'fixture.bag'),'--world',str(R/'world.sdf'),'--urdf',str(R/'robot_description_runtime.xml'),'--closure',str(R/'run_closure.json'),'--converted-sdf',str(R/'converted_robot.sdf'),'--conversion-manifest',str(R/'conversion_manifest.json'),'--start-ns',str(raw['start_ns']),'--end-ns',str(raw['end_ns']),'--output',str(O)]
env=dict(os.environ,PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages:'+str(W/'devel/lib/python3/dist-packages'),PYTHONDONTWRITEBYTECODE='1')
with (O.parent/'sweep_replay.log').open('x')as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,timeout=180)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
old=load(R/'sweep_evaluation/report.json');new=load(O/'report.json')
checks={'CLI_exit_zero':r.returncode==0,'report_exact_except_output_paths':{k:v for k,v in old.items()if k!='outputs'}=={k:v for k,v in new.items()if k!='outputs'},'every_interval_endpoint_byte_exact':all(sha(R/'sweep_evaluation'/p)==sha(O/p)for p in ('intervals.jsonl','endpoints.csv'))}
out={'pass':all(checks.values()),'checks':checks,'actual_command_argv':cmd,'PYTHONPATH':env['PYTHONPATH'],'returncode':r.returncode,'source_script_sha256':sha(Path(__file__)),'dense_intervals':new['dense_solver_intervals'],'minimum_engineering_swept_xy_m':new['minimum_engineering_conservative_swept_xy_m']}
with(O.parent/'sweep_replay_audit.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps(out));assert out['pass']
