#!/usr/bin/python3
"""Offline real gazebo_ros shell wrapper with a sole non-Gazebo exec substitute."""
import argparse,hashlib,json,os,shlex,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
sys.path.insert(0,str(P));import navigation_n06_environment_r05 as policy

def bound(p):p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def write(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def command(out,name,argv,env):
 with (out/(name+'.stdout')).open('xb') as stdout,(out/(name+'.stderr')).open('xb') as stderr:r=subprocess.run(argv,env=env,cwd=str(W),stdout=stdout,stderr=stderr,timeout=30)
 write(out/(name+'.command.json'),{'argv':argv,'environment':env,'returncode':r.returncode,'stdout':bound(out/(name+'.stdout')),'stderr':bound(out/(name+'.stderr'))})
 if r.returncode!=0:raise ValueError('offline command failed: '+name)
 return (out/(name+'.stdout')).read_bytes()
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=args.output;out.mkdir(exist_ok=False)
 initial=dict(policy.BOOTSTRAP_ENV)
 for key in ('DISPLAY','XDG_RUNTIME_DIR','TZ','LC_ALL'):
  if key in os.environ:initial[key]=os.environ[key]
 initial.update(ASTRA_ROS_PORT='14830',ASTRA_GAZEBO_PORT='14831',ASTRA_ROS_HOME=str(out/'source_only_ros_home'),ASTRA_ROS_LOG_DIR=str(out/'source_only_roslog'))
 shell='PS4=\'+${BASH_SOURCE}:${LINENO} \'; set -x; source "$1"; set +x; exec /usr/bin/env -0'
 raw=command(out,'source_environment',['/bin/bash','--noprofile','--norc','-c',shell,'n06-env-observe',str(W/'scripts/astra_env.sh')],initial)
 source=dict(item.decode().split('=',1) for item in raw.split(b'\0') if item);spec=policy.make_spec(source);write(out/'execution_environment.json',spec)
 # Sole shim does not simulate Gazebo; it exposes the final actual wrapper exec.
 shim=out/'exec_substitute';shim.mkdir();script=shim/'gzserver'
 script.write_text('#!/usr/bin/python3\nimport json,os,sys\nprint(json.dumps({"argv":sys.argv,"environment":dict(os.environ),"gazebo_started":False},sort_keys=True))\n');script.chmod(0o755)
 prepared=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text());nodes=json.loads((A/'collection_attempt03/system_launcher.json').read_text())['resolved_nodes'];results=[]
 for unit in prepared['units']:
  for key in ('world','robot_description'):
   xml=ET.fromstring(Path(unit[key]['path']).read_bytes())
   if any((n.text or '').strip() for n in xml.findall('.//uri')):raise ValueError('external URI not supported by N06 localhost-only model database')
  node=next(n for n in nodes if n['run_id']==unit['run_id'] and n['package']=='gazebo_ros' and n['type']=='gzserver')
  env=policy.expected_environment(spec,'0'*64);run=S/'runs'/unit['run_id'];env.update(ROS_HOME=str(run/'ros_home'),ROS_LOG_DIR=str(run/'roslog'),ASTRA_N06_CHILD_NONCE='a'*48)
  original_path=env['PATH'];env['PATH']=str(shim)+':'+original_path
  payload=json.loads(command(out,unit['run_id'],['/opt/ros/noetic/lib/gazebo_ros/gzserver']+shlex.split(node['args']),env))
  observed=payload['environment'];prefix=str(shim)+':'
  if observed['PATH']!=prefix+original_path or sorted(x.name for x in shim.iterdir())!=['gzserver']:raise ValueError('shim changed non-Gazebo resolution')
  normalized=dict(observed);normalized['PATH']=original_path
  if normalized.get('GAZEBO_MODEL_DATABASE_URI')!=policy.MODEL_DATABASE:raise ValueError('wrapper restored remote database')
  results.append({'run_id':unit['run_id'],'world':unit['world'],'robot_description':unit['robot_description'],'node':node,'observed':payload,
    'substitution':{'only_executable':bound(script),'path_prefix_removed':prefix,'real_exec_target':bound('/usr/bin/gzserver')},'normalized_real_exec_environment':normalized,
    'effective_difference_from_manager':{k:v for k,v in normalized.items() if spec['values'].get(k)!=v}})
 # Actual shell negative: the old empty string restores the remote default.
 negative=dict(env);negative['GAZEBO_MODEL_DATABASE_URI']=''
 neg=json.loads(command(out,'empty_model_database_negative',['/opt/ros/noetic/lib/gazebo_ros/gzserver','--offline-negative-only'],negative))
 if neg['environment']['GAZEBO_MODEL_DATABASE_URI']!='http://models.gazebosim.org':raise ValueError('old empty-URI counterexample not reproduced')
 write(out/'wrapper_observations.json',{'schema':'astra_n06_wrapper_environment_observations_r05','execution_environment':bound(out/'execution_environment.json'),'wrapper':bound('/opt/ros/noetic/lib/gazebo_ros/gzserver'),'setup':bound('/usr/share/gazebo/setup.sh'),'units':results,
  'empty_uri_negative_retained':True,'no_external_uri_in_eight_worlds_and_urdfs':True,'dynamic_closure_sha_and_nonce_are_explicit_placeholders':True,'ros_gazebo_started':False})
 print(json.dumps({'units':len(results),'output':str(out),'ros_gazebo_started':False}))
if __name__=='__main__':main()
