#!/usr/bin/env python3
"""Versioned offline N06 disk dependency collection. No ROS/world launch."""
import argparse,ast,hashlib,json,os,re,shutil,stat,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
NAME='astra_navigation_n06_lookahead_static_pilot'


def binding(path):
 p=Path(path).resolve(strict=True);before=p.stat()
 if not stat.S_ISREG(before.st_mode):raise ValueError('regular disk file required: '+str(p))
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 after=p.stat()
 if (before.st_ino,before.st_dev,before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_ino,after.st_dev,after.st_size,after.st_mtime_ns,after.st_ctime_ns):raise ValueError('file changed during read: '+str(p))
 return {'path':str(p),'sha256':h.hexdigest()}

def write(path,value):
 with Path(path).open('x') as f:json.dump(value,f,sort_keys=True,indent=2,allow_nan=False);f.write('\n')

def parse_ldd(text):
 paths=[];virtual=[]
 for raw in text.splitlines():
  line=raw.strip()
  if not line:continue
  if 'not found' in line:raise ValueError('unresolved ELF dependency: '+line)
  if line.startswith('linux-vdso.so.') or line.startswith('linux-gate.so.'):
   virtual.append(line);continue
  match=re.match(r'([^ ]+)\s+=>\s+(/.+?)\s+\(0x[0-9a-f]+\)$',line)
  if match:paths.append({'soname':match[1],'alias':match[2]});continue
  match=re.match(r'(/.+?)\s+\(0x[0-9a-f]+\)$',line)
  if match:paths.append({'soname':Path(match[1]).name,'alias':match[1]});continue
  if line in ('statically linked','not a dynamic executable'):continue
  raise ValueError('unrecognized ldd output: '+line)
 return paths,virtual

def dynamic_needed(text):return re.findall(r'\(NEEDED\).*Shared library: \[([^\]]+)\]',text)

def inherited_resolution(text,path,completed):
 """Keep standalone failures; use only a prior root's actual full ldd table.

 Auditwheel private dependencies can rely on a parent's transitive RPATH. No
 new LD_LIBRARY_PATH or guessed same-directory lookup is introduced here.
 """
 missing=re.findall(r'^\s*(\S+)\s+=> not found\s*$',text,re.M)
 clean='\n'.join(line for line in text.splitlines() if not re.match(r'^\s*\S+\s+=> not found\s*$',line))
 found,virtual=parse_ldd(clean);contexts=[]
 if missing:
  choices={}
  for root in completed:
   if path not in {item['binding']['path'] for item in root['resolved']}:continue
   table={item['soname']:item for item in root['resolved']}
   if all(name in table for name in missing):
    contexts.append(root['file']);choices[tuple(table[name]['binding']['path'] for name in missing)]=[table[name] for name in missing]
  if len(choices)!=1:raise ValueError('unresolved or ambiguous parent loader context: '+path+' '+repr(missing))
  found.extend({'soname':item['soname'],'alias':item['alias']} for item in next(iter(choices.values())))
 return found,virtual,missing,contexts

def safe_environment():
 allowed=('HOME','USER','LOGNAME','LANG','LC_ALL','DISPLAY','XDG_RUNTIME_DIR','TZ')
 env={k:os.environ[k] for k in allowed if k in os.environ};env.update(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 return env

class Collector:
 def __init__(self,output,manager_revision='r03'):
  self.output=Path(output);self.output.mkdir(exist_ok=False);(self.output/'commands').mkdir();self.commands=[];self.files={};self.aliases={};self.roles={};self.elf=set();self.environ=safe_environment()
  self.manager_revision=manager_revision
 def add(self,path,role):
  alias=Path(path).absolute();bound=binding(alias);canonical=bound['path']
  if canonical in self.files and self.files[canonical]!=bound:raise ValueError('resource changed')
  self.files[canonical]=bound;self.roles.setdefault(canonical,set()).add(role)
  if str(alias)!=canonical:
   chain=[];current=alias;seen=set()
   # Parent directory links are represented by the full alias/canonical pair;
   # final-component links additionally retain exact readlink bytes.
   while current.is_symlink():
    if str(current) in seen:raise ValueError('symlink cycle')
    seen.add(str(current));target=os.readlink(current);chain.append({'path':str(current),'link_text':target});current=Path(target) if Path(target).is_absolute() else current.parent/target
   self.aliases[str(alias)]={'alias':str(alias),'canonical':canonical,'final_component_links':chain}
  with Path(canonical).open('rb') as f:magic=f.read(4)
  if magic==b'\x7fELF':self.elf.add(canonical)
  return bound
 def command(self,argv,label,env=None,timeout=120):
  index=len(self.commands);prefix=self.output/'commands'/('%04d_%s'%(index,label));start=time.monotonic()
  with prefix.with_suffix('.stdout').open('xb') as out,prefix.with_suffix('.stderr').open('xb') as err:
   try:r=subprocess.run([str(x) for x in argv],cwd=str(W),env=env or self.environ,stdout=out,stderr=err,timeout=timeout);code=r.returncode;failure=None
   except BaseException as exc:code=None;failure=type(exc).__name__+': '+str(exc)
  row={'argv':[str(x) for x in argv],'cwd':str(W),'returncode':code,'error':failure,'wall_s':time.monotonic()-start,
    'stdout':binding(prefix.with_suffix('.stdout')),'stderr':binding(prefix.with_suffix('.stderr'))}
  write(prefix.with_suffix('.json'),row);self.commands.append(row)
  if code!=0:raise RuntimeError('offline command failed; raw output retained: '+str(prefix))
  return prefix.with_suffix('.stdout').read_text(),prefix.with_suffix('.stderr').read_text()
 def collect(self):
  envscript=W/'scripts/astra_env.sh';probe=A/'python_import_probe.py'
  shell='PS4=\'+${BASH_SOURCE}:${LINENO} \'; set -x; source "$1"; set +x; exec "$2" "$3" --profile "$4" --output "$5" --manager-revision "$6"'
  profiles={}
  for profile,python in (('venv',W/'.venv/bin/python'),('system_launcher',Path('/usr/bin/python3'))):
   env=safe_environment();env.update(ASTRA_ROS_PORT='14830',ASTRA_GAZEBO_PORT='14831',ASTRA_ROS_HOME=str(self.output/'offline_ros_home'),ASTRA_ROS_LOG_DIR=str(self.output/'offline_roslog'))
   out=self.output/(profile+'.json');_,trace=self.command(['/bin/bash','--noprofile','--norc','-c',shell,'n06-offline-probe',envscript,python,probe,profile,out,self.manager_revision],profile,env)
   profiles[profile]=json.loads(out.read_text())
   for path in re.findall(r'^\++(/[^:\n]+):\d+ ',trace,re.M):
    if Path(path).is_file():self.add(path,'sourced_shell_or_catkin_hook')
  v=profiles['venv'];system=profiles['system_launcher']
  if v['sys_prefix']!=str(W/'.venv') or not v['no_user_site'] or system['sys_prefix']!='/usr' or not system['no_user_site']:raise ValueError('interpreter identities differ')
  self.environ.update({k:value for k,value in v['environment'].items() if value is not None})
  # Bind exact authored manager/prepared resource closures without modifying them.
  for path in (S/'audits/navigation_n06_preparation/source_evidence_manifest.json',S/('audits/navigation_n06_manager_'+self.manager_revision+'/source_evidence_manifest.json')):
   self.add(path,'frozen_input_manifest');value=json.loads(path.read_text())
   groups=('owned_sources','root_owned_and_frozen_runtime_nodes','prepared_inputs','read_only_dependencies') if 'prepared_inputs' in value else ('owned_sources',)
   for group in groups:
    for item in value[group]:
     if self.add(item['path'],'frozen_source_or_prepared_input')!= {k:item[k] for k in ('path','sha256')}:raise ValueError('frozen input drift')
  # Actual runtime entry points, plus rosmaster/rosout and wrapper tools.
  executable_roots=['/usr/bin/gzserver','/opt/ros/noetic/bin/roslaunch','/opt/ros/noetic/bin/rosmaster','/opt/ros/noetic/bin/roscore',
   '/opt/ros/noetic/lib/rosout/rosout','/opt/ros/noetic/bin/xacro','/opt/ros/noetic/bin/catkin_find',
   '/usr/bin/env','/bin/sh','/bin/bash','/usr/bin/dirname','/usr/bin/readlink','/usr/bin/uname','/usr/bin/expr','/usr/bin/grep','/usr/bin/cut','/usr/bin/pkg-config',
   '/opt/ros/noetic/lib/gazebo_ros/libcommon.sh','/usr/bin/ldd','/usr/bin/readelf','/sbin/ldconfig','/sbin/ldconfig.real']
  for path in executable_roots:self.add(path,'required_runtime_or_resolution_tool')
  for profile in profiles.values():
   self.add(profile['sys_executable'],'python_interpreter')
   for node in profile['resolved_nodes']:
    self.add(node['entry_alias'],'resolved_ros_node')
    if node['interpreter']:self.add(node['interpreter']['invocation'],'resolved_node_interpreter')
   for path in profile['launch_files']:self.add(path,'resolved_launch_include')
   for pkg,path in profile['package_paths'].items():self.add(Path(path)/'package.xml','resolved_ros_package_manifest')
   for module in profile['modules']:
    self.add(module['path'],'actually_imported_python_module')
    cached=module.get('cached')
    if cached and Path(cached).is_file():self.add(cached,'existing_python_cache_paired_with_source')
   # Conservative disk package envelope includes lazy code/data; it does not
   # assert every file is executed. Bytecode caches are bound only if observed.
   for package,roots in profile['package_roots'].items():
    for root in roots:
     for path in sorted(Path(root).rglob('*')):
      if path.is_file() and not {'__pycache__','tests','test','testing'}&set(path.relative_to(root).parts) and path.suffix not in ('.pyc','.pyo'):
       self.add(path,'python_package_envelope:'+package)
  for path in (W/'.venv/pyvenv.cfg',*list((W/'.venv/lib/python3.8/site-packages').glob('*.pth')),*list(Path('/usr/lib/python3/dist-packages').glob('*.pth'))):self.add(path,'python_startup_configuration')
  # External environment configuration and schema/media used by Gazebo/SDFormat.
  for path in (Path('/etc/ld.so.cache'),Path('/etc/ld.so.conf'),*Path('/etc/ld.so.conf.d').glob('*.conf')):self.add(path,'loader_configuration')
  for directory in ('/usr/share/sdformat9','/usr/share/gazebo-11/media','/usr/share/glvnd/egl_vendor.d'):
   for path in sorted(Path(directory).rglob('*')):
    if path.is_file():self.add(path,'schema_render_or_graphics_configuration')
  self.command(['/usr/bin/pkg-config','--variable=prefix','gazebo'],'gazebo_prefix')
  _,_=self.command(['/opt/ros/noetic/bin/catkin_find','--first-only','libgazebo_ros_paths_plugin.so'],'catkin_paths_plugin')
  _,_=self.command(['/opt/ros/noetic/bin/catkin_find','--first-only','libgazebo_ros_api_plugin.so'],'catkin_api_plugin')
  prepared=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text());plugins=set()
  for unit in prepared['units']:
   for field in ('world','robot_description'):
    root=ET.fromstring(Path(unit[field]['path']).read_bytes());plugins.update(node.get('filename') for node in root.findall('.//plugin'))
  plugins.update(('libgazebo_ros_paths_plugin.so','libgazebo_ros_api_plugin.so'))
  paths=[Path(x) for x in v['environment']['GAZEBO_PLUGIN_PATH'].split(':') if x]
  plugin_rows=[]
  for name in sorted(plugins):
   candidates=[path/name for path in paths if (path/name).is_file()]
   if not candidates:raise ValueError('unresolved configured plugin '+name)
   selected=candidates[0];self.add(selected,'configured_gazebo_plugin');plugin_rows.append({'filename':name,'search_paths':[str(p) for p in paths],'candidates':[str(p) for p in candidates],'selected':str(selected.resolve())})
  # Dlopen roots absent from DT_NEEDED: OGRE renderers + the exact already
  # frozen graphics-policy libraries; all are disk-only conservative roots.
  for path in Path('/usr/lib/x86_64-linux-gnu/OGRE-1.9.0').glob('*.so'):self.add(path,'ogre_dlopen_envelope')
  sys.path.insert(0,str(P));import mapped_elf_inventory_v2 as inventory
  for item in inventory.GRAPHICS_BINDINGS.values():
   if self.add(item['path'],'exact_frozen_graphics_policy')!=item:raise ValueError('graphics policy library drift')
  ldconfig,_=self.command(['/sbin/ldconfig','-p'],'ldconfig_cache')
  for name in v['ctypes_library_names']:
   candidates=[Path(d)/name for d in self.environ.get('LD_LIBRARY_PATH','').split(':') if d and (Path(d)/name).is_file()]
   if Path(name).is_absolute():candidates=[Path(name)]
   if not candidates:
    candidates=[Path(path) for soname,flags,path in re.findall(r'^\s*(\S+)\s+\(([^)]+)\)\s+=>\s+(\S+)',ldconfig,re.M) if soname==name and 'x86-64' in flags]
   if len({str(x.resolve()) for x in candidates})!=1:raise ValueError('ambiguous/unresolved ctypes dependency '+name)
   self.add(candidates[0],'explicit_python_ctypes_dependency')
  initial_elf=sorted(self.elf);done=set();edges=[]
  while self.elf-done:
   path=sorted(self.elf-done,key=lambda p:(p not in initial_elf,p))[0];before=self.files[path]
   dynamic,_=self.command(['/usr/bin/readelf','--dynamic',path],'dynamic')
   program,_=self.command(['/usr/bin/readelf','--program-headers',path],'program_headers')
   interpreters=re.findall(r'Requesting program interpreter: ([^\]]+)\]',program)
   if len(interpreters)>1:raise ValueError('multiple ELF interpreters')
   needed=dynamic_needed(dynamic)
   if needed or interpreters:
    ldtext,_=self.command(['/usr/bin/ldd',path],'ldd');found,virtual,missing,contexts=inherited_resolution(ldtext,path,edges)
   else:found,virtual,missing,contexts=[],[],[],[]
   resolved={row['soname'] for row in found}
   if set(needed)-resolved:raise ValueError('DT_NEEDED absent from ldd resolution '+path+': '+repr(set(needed)-resolved))
   deps=[]
   for row in found:deps.append(dict(row,binding=self.add(row['alias'],'recursive_elf_loader_dependency')))
   loaders=[self.add(x,'elf_program_interpreter') for x in interpreters]
   if binding(path)!=before:raise ValueError('ELF changed during dependency resolution')
   edges.append({'file':before,'needed':needed,'resolved':deps,'program_interpreters':loaders,'virtual_not_disk_resources':virtual,
    'standalone_unresolved':missing,'inherited_root_contexts':contexts});done.add(path)
  write(self.output/'elf_graph.json',{'roots':initial_elf,'nodes':edges,'scope':'ldd/readelf disk resolution under saved environment; not Gazebo mapped ELF'})
  write(self.output/'aliases.json',list(self.aliases.values()));write(self.output/'configured_plugins.json',plugin_rows)
  write(self.output/'resource_roles.json',{p:sorted(r) for p,r in sorted(self.roles.items())})
  write(self.output/'commands.json',self.commands)
  resources=[self.files[p] for p in sorted(self.files)]
  if any(binding(item['path'])!=item for item in resources):raise ValueError('resource drift at final collection')
  write(self.output/'collection.json',{'schema':'astra_n06_runtime_disk_collection_v1','resources':resources,'elf_roots':len(initial_elf),'elf_nodes':len(done),
    'commands':len(self.commands),'python_modules':{k:len(v['modules']) for k,v in profiles.items()},'resolved_node_rows':len(system['resolved_nodes']),
    'resource_count':len(resources),'total_bytes':sum(Path(x['path']).stat().st_size for x in resources),
    'evidence_paths':[str(p) for p in sorted(self.output.rglob('*')) if p.is_file()],'ros_gazebo_started':False,'live_mapped_elf_observed':False})
  return {'resource_count':len(resources),'elf_nodes':len(done),'commands':len(self.commands)}

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--manager-revision',choices=['r02','r03'],required=True);args=parser.parse_args();collector=Collector(args.output,args.manager_revision)
 try:result=collector.collect();write(args.output/'status.json',{'status':'author_disk_collection_complete',**result});print(json.dumps(result))
 except BaseException as exc:
  write(args.output/'failure.json',{'status':'failed_preserved','error':type(exc).__name__+': '+str(exc),'commands_completed':len(collector.commands),'resources_seen':len(collector.files),'ros_gazebo_started':False});raise
if __name__=='__main__':main()
