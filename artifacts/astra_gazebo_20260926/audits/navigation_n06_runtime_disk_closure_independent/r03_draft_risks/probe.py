"""Bounded r03 draft counterexamples. No ROS/Gazebo application is invoked."""
import ast,hashlib,json,pathlib,subprocess,types
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';A=pathlib.Path(__file__).parent
D=S/'audits/navigation_n06_runtime_closure_r01/collection_attempt02'
READ={}
def read(p):
 p=pathlib.Path(p).resolve();raw=p.read_bytes();READ[str(p)]={'path':str(p),'sha256':hashlib.sha256(raw).hexdigest()};return raw
def js(p):return json.loads(read(p))
def write(n,v):(A/n).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
status=js(D/'status.json');assert status['status']=='author_disk_collection_complete'
collection=js(D/'collection.json');venv=js(D/'venv.json');system=js(D/'system_launcher.json')
manager_path=P/'run_navigation_n06_manager_r03.py';manager=read(manager_path).decode();tree=ast.parse(manager)
fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute_unit')
assignment=next(n for n in ast.walk(fn) if isinstance(n,ast.Assign) and len(n.targets)==1 and isinstance(n.targets[0],ast.Name) and n.targets[0].id=='env')
# Execute only the actual pure dict construction, with synthetic os.environ.
unsafe={k:v for k,v in venv['environment'].items() if v is not None}
injections={'LD_PRELOAD':'/not-loaded/synthetic-preload.so','LD_AUDIT':'/not-loaded/synthetic-audit.so','PYTHONHOME':'/not-read/synthetic-pythonhome','PYTHONUSERBASE':'/not-read/synthetic-userbase','__EGL_VENDOR_LIBRARY_FILENAMES':'/not-read/synthetic-vendor.json'}
unsafe.update(injections)
ns={'os':types.SimpleNamespace(environ=unsafe),'root':pathlib.Path('/not-created/synthetic-run'),'nonce':'synthetic-not-a-credential'}
exec(compile(ast.Module(body=[assignment],type_ignores=[]),'<exact frozen manager env assignment>','exec'),ns)
assert all(ns['env'][k]==v for k,v in injections.items())
collector=ast.parse(read(D/'source_snapshot/build_disk_closure.py'))
safe=next(n for n in collector.body if isinstance(n,ast.FunctionDef) and n.name=='safe_environment')
cns={'os':types.SimpleNamespace(environ=unsafe)};exec(compile(ast.Module(body=[safe],type_ignores=[]),'<exact draft collector safe environment>','exec'),cns)
assert all(k not in cns['safe_environment']() for k in injections)
envscript=read(W/'scripts/astra_env.sh').decode();unset_names=set()
for line in envscript.splitlines():
 if line.strip().startswith('unset '):unset_names.update(line.strip().split()[1:])
assert all(k not in unset_names for k in injections)
write('synthetic_parent_child_environment.json',{'source':READ[str(manager_path.resolve())],'actual_assignment':ast.get_source_segment(manager,assignment),'synthetic_keys_only':injections,'retained_by_manager_dict':{k:ns['env'][k] for k in injections},'dropped_by_draft_collection_whitelist':list(injections),'not_unset_by_astra_env':list(injections),'fake_libraries_loaded':False,'manager_or_ROS_called':False,'conclusion':'disk collection input domain differs from inherited execution input domain; current real host contamination is not asserted'})
# Replay the real shell wrapper prelude under saved explicit environment,
# replacing exactly its final gzserver invocation with /usr/bin/env.
wrapper_path=pathlib.Path('/opt/ros/noetic/lib/gazebo_ros/gzserver');wrapper=read(wrapper_path).decode()
old='GAZEBO_MASTER_URI="$desired_master_uri" GAZEBO_MODEL_DATABASE_URI="$desired_model_database_uri" gzserver $final'
new='GAZEBO_MASTER_URI="$desired_master_uri" GAZEBO_MODEL_DATABASE_URI="$desired_model_database_uri" /usr/bin/env'
assert wrapper.count(old)==1
probe=wrapper.replace(old,new);(A/'wrapper_environment_only.sh').write_text(probe)
for p in ['/opt/ros/noetic/lib/gazebo_ros/libcommon.sh','/usr/share/gazebo/setup.sh','/usr/share/gazebo-11/setup.sh','/usr/bin/pkg-config','/opt/ros/noetic/bin/catkin_find','/bin/sh','/usr/bin/env']:read(p)
base={k:v for k,v in venv['environment'].items() if v is not None};base['PYTHONDONTWRITEBYTECODE']='1';base['PYTHONNOUSERSITE']='1'
# Source may inspect real directories/package indexes but launches no server.
r=subprocess.run(['/bin/sh','-c',probe,str(wrapper_path)],env=base,cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=15)
(A/'wrapper_environment_only.stdout').write_bytes(r.stdout);(A/'wrapper_environment_only.stderr').write_bytes(r.stderr);assert r.returncode==0,r.stderr
post=dict(line.split('=',1) for line in r.stdout.decode().splitlines() if '=' in line)
keys=('GAZEBO_PLUGIN_PATH','LD_LIBRARY_PATH','GAZEBO_MODEL_PATH','GAZEBO_RESOURCE_PATH','GAZEBO_MODEL_DATABASE_URI')
changes={k:{'before':base.get(k),'after':post.get(k)} for k in keys if base.get(k)!=post.get(k)}
assert set(changes)==set(keys) and post['GAZEBO_MODEL_DATABASE_URI']=='http://models.gazebosim.org'
plugins=js(D/'configured_plugins.json');selection=[]
for row in plugins:
 candidates=[pathlib.Path(d)/row['filename'] for d in post['GAZEBO_PLUGIN_PATH'].split(':') if d and (pathlib.Path(d)/row['filename']).is_file()]
 assert candidates
 selected=str(candidates[0].resolve());selection.append({'filename':row['filename'],'draft_selected':row['selected'],'wrapper_selected':selected,'same':selected==row['selected']})
assert len(selection)==9 and all(x['same'] for x in selection)
write('wrapper_environment_difference.json',{'method':'exact real wrapper text with only final gzserver command replaced by env; source and command variant bound','argv':['/bin/sh','-c','<bound wrapper_environment_only.sh bytes>',str(wrapper_path)],'cwd':str(W),'base_environment_source':READ[str((D/'venv.json').resolve())],'exit_code':r.returncode,'changes':changes,'configured_plugin_selection':selection,'no_remote_lookup_tested':True,'no_gzserver_or_ROS_started':True,'scope':'The wrapper restores a remote model URI; no actual download or runtime model use is claimed. Existing nine configured plugin selections match despite changed path ordering.'})
write('reviewed_draft_summary.json',{'schema':'astra_n06_r03_disk_draft_risk_probe_v1','formal_final_closure_review':False,'draft_collection_status':status,'disk_resource_count_declared':collection['resource_count'],'declared_elf_nodes':collection['elf_nodes'],'declared_import_files':collection['python_modules'],'resolved_node_rows':collection['resolved_node_rows'],'declared_disk_files_all_rehashed_by_this_probe':False,'two_confirmed_environment_context_gaps':True,'actual_r03_episode_started':False,'r05_new_closure_required_before_formal_review':True})
write('read_source_bindings.json',{'resources':sorted(READ.values(),key=lambda r:r['path'])})
print(json.dumps({'status':'two_source_environment_counterexamples_preserved','plugin_selections_unchanged':len(selection),'commands_executed':'only modified environment-observer shell prelude, no ROS/Gazebo main','read_sources':len(READ)}))
