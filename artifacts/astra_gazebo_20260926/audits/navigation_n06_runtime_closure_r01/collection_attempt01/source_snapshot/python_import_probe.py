#!/usr/bin/env python3
"""Offline import/launch-resolution probe. Never initializes or launches ROS."""
import argparse,ast,contextlib,importlib,importlib.util,io,json,os,sys
from pathlib import Path
import shutil


def main():
 parser=argparse.ArgumentParser();parser.add_argument('--profile',choices=['venv','system_launcher'],required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--manager-revision',choices=['r02','r03'],required=True);args=parser.parse_args()
 W=Path(os.environ['THESIS_WS']);P=W/'scripts/astra_gazebo';S=W.parent;G=W/'src/simulation/m2_gazebo'
 sys.path.insert(0,str(P));imported=[]
 modules=['roslaunch','roslaunch.parent','roslaunch.xmlloader','roslaunch.config','rosmaster','rosgraph','roslib','roslib.packages','rospkg','catkin_pkg','xacro']
 if args.profile=='venv':modules += ['rospy','rosbag','genpy','genpy.dynamic','tf2_ros','tf2_py','numpy','scipy.ndimage','PIL.Image','PIL.PngImagePlugin','yaml','shapely.geometry','shapely.ops',
  'geometry_msgs.msg','nav_msgs.msg','nav_msgs.srv','sensor_msgs.msg','diagnostic_msgs.msg','std_msgs.msg','std_srvs.srv','gazebo_msgs.msg','gazebo_msgs.srv','rosgraph_msgs.msg',
  'run_navigation_n06_manager_'+args.manager_revision,'navigation_n06_integrity_'+args.manager_revision,'navigation_n06_process_'+args.manager_revision,'navigation_n06_admin_plan_'+args.manager_revision,
  'common_route','route_candidate_core','route_sensor_buffer','navigation_coordinator_n06','navigation_evidence_n06','dense_sweep_n05','evaluate_episode_n05','run_passive_sweep_q05','bound_archive','mapped_elf_inventory_v2']
 for name in modules:importlib.import_module(name);imported.append(name)
 if 'rospy' in sys.modules:
  def forbidden(*_a,**_k):raise RuntimeError('offline probe forbids ROS initialization')
  sys.modules['rospy'].init_node=forbidden
 # Import actual entry-point modules without calling their main functions.
 entries=[]
 if args.profile=='venv':
  for name in ('astra_common_route_provider.py','astra_navigation_coordinator_n06.py','astra_navigation_collector_n06.py','astra_route_coordinated_runtime.py','astra_route_lookahead_runtime_n06.py'):
   p=G/'scripts'/name;spec=importlib.util.spec_from_file_location('n06_probe_'+p.stem,p);m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m);entries.append(str(p.resolve()))
 nodes=[];launch_files=[];package_paths={}
 if args.profile=='system_launcher':
  import roslaunch.xmlloader,roslaunch.config,roslib.packages,rospkg
  rp=rospkg.RosPack();prepared=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text())
  for unit in prepared['units']:
   config=roslaunch.config.ROSLaunchConfig()
   with contextlib.redirect_stdout(sys.stderr):roslaunch.xmlloader.XmlLoader().load(unit['launch']['path'],config,verbose=False)
   launch_files.extend(config.roslaunch_files)
   for node in config.nodes:
    paths=roslib.packages.find_node(node.package,node.type,rospack=rp)
    if not paths or len(paths)!=1:raise ValueError('ambiguous/missing executable '+node.package+'/'+node.type+': '+repr(paths))
    path=Path(paths[0]);data=path.read_bytes();prefix=node.launch_prefix
    interpreter=None
    if prefix:interpreter={'kind':'launch_prefix','invocation':prefix,'canonical':str(Path(prefix).resolve())}
    elif data.startswith(b'#!'):
     words=data.splitlines()[0][2:].decode().split()
     invoked=shutil.which(words[1]) if words[0]=='/usr/bin/env' else words[0]
     if not invoked:raise ValueError('unresolved shebang')
     interpreter={'kind':'shebang','raw':data.splitlines()[0].decode(),'invocation':invoked,'canonical':str(Path(invoked).resolve())}
    nodes.append({'run_id':unit['run_id'],'method':unit['method'],'package':node.package,'type':node.type,'name':node.name,'namespace':node.namespace,
      'entry_alias':str(path.absolute()),'entry_canonical':str(path.resolve()),'launch_prefix':prefix,'interpreter':interpreter,'args':node.args})
    package_paths[node.package]=rp.get_path(node.package)
 # Small in-memory package operations expose lazy PNG/scientific support, not planner timing or evidence.
 ctypes_libraries=[]
 if args.profile=='venv':
  import numpy as np
  from scipy.ndimage import distance_transform_edt
  from PIL import Image
  from shapely.geometry import box
  from shapely import geos
  assert distance_transform_edt(np.eye(2)).shape==(2,2)
  assert box(0,0,1,1).distance(box(2,0,3,1))==1
  output=io.BytesIO();Image.fromarray(np.zeros((2,2,3),dtype=np.uint8)).save(output,format='PNG');Image.open(io.BytesIO(output.getvalue())).load()
  ctypes_libraries=[str(geos._lgeos._name)]
 rows=[];packages={};versions={}
 for name,module in list(sys.modules.items()):
  p=getattr(module,'__file__',None)
  if p and Path(p).is_file():
   rows.append({'module':name,'path':str(Path(p).resolve()),'cached':getattr(module,'__cached__',None)})
  if '.' not in name and getattr(module,'__path__',None):
   roots=[str(Path(x).resolve()) for x in module.__path__ if Path(x).is_dir()]
   if roots:packages[name]=roots
   version=getattr(module,'__version__',None)
   if isinstance(version,(str,int,float)):versions[name]=str(version)
 safe_keys=('PATH','PYTHONPATH','LD_LIBRARY_PATH','CMAKE_PREFIX_PATH','ROS_PACKAGE_PATH','ROS_ROOT','ROS_DISTRO','ROS_MASTER_URI','GAZEBO_MASTER_URI','GAZEBO_PLUGIN_PATH','GAZEBO_RESOURCE_PATH','GAZEBO_MODEL_PATH','GAZEBO_MODEL_DATABASE_URI','OGRE_RESOURCE_PATH','PKG_CONFIG_PATH','VIRTUAL_ENV','THESIS_WS','PYTHONNOUSERSITE','PYTHONDONTWRITEBYTECODE','ROS_HOME','ROS_LOG_DIR','OMP_NUM_THREADS','MKL_NUM_THREADS')
 result={'schema':'astra_n06_offline_python_import_probe_v1','profile':args.profile,'sys_executable':sys.executable,'canonical_executable':str(Path(sys.executable).resolve()),
  'sys_prefix':sys.prefix,'base_prefix':sys.base_prefix,'no_user_site':bool(sys.flags.no_user_site),'sys_path':sys.path,'python_version':sys.version,
  'environment':{k:os.environ.get(k) for k in safe_keys},'explicit_imports':imported,'entry_modules_imported_without_main':entries,'modules':rows,'package_roots':packages,
  'package_versions':versions,'ctypes_library_names':ctypes_libraries,'resolved_nodes':nodes,'launch_files':sorted(set(launch_files)),'package_paths':package_paths,
  'ros_initialization_called':False,'ros_gazebo_process_started':False,'scope':'Offline imports/XML expansion and disk resolution; no live node or Gazebo mapped ELF observation.'}
 with args.output.open('x') as f:json.dump(result,f,indent=2,sort_keys=True);f.write('\n')
 print(json.dumps({'profile':args.profile,'module_files':len(rows),'packages':len(packages),'resolved_nodes':len(nodes),'output':str(args.output)}))
if __name__=='__main__':main()
