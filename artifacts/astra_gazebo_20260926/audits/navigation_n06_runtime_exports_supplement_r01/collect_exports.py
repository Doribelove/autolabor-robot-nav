#!/usr/bin/python3
"""Read-only rospack export supplement; never initializes ROS or Gazebo."""
import argparse,collections,hashlib,json,os,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';R=S/'audits/navigation_n06_runtime_closure_r01'
sys.path.insert(0,str(R));from build_disk_closure import binding,write,parse_ldd
ATTRS=('plugin_path','gazebo_media_path','gazebo_model_path')

def exports(text):
 rows=[]
 for line in text.splitlines():
  if not line.strip():continue
  parts=line.split(None,1)
  if len(parts)!=2 or not parts[0] or not Path(parts[1]).is_absolute():raise ValueError('invalid actual rospack export row')
  rows.append({'package':parts[0],'declared_path':parts[1]})
 return rows

def manifest_exports(path,attrib):
 path=Path(path);root=ET.fromstring(path.read_bytes());name=root.findtext('name')
 return [{'package':name,'declared_path':n.attrib[attrib].replace('${prefix}',str(path.parent))} for n in root.findall('./export/gazebo_ros') if attrib in n.attrib]

def command(out,label,argv,env):
 p=out/label
 with p.with_suffix('.stdout').open('xb') as stdout,p.with_suffix('.stderr').open('xb') as stderr:r=subprocess.run(argv,cwd=str(W),env=env,stdout=stdout,stderr=stderr,timeout=30)
 row={'argv':argv,'environment':env,'cwd':str(W),'returncode':r.returncode,'stdout':binding(p.with_suffix('.stdout')),'stderr':binding(p.with_suffix('.stderr'))}
 write(p.with_suffix('.json'),row)
 if r.returncode!=0:raise ValueError('retained offline query failure '+label)
 return p.with_suffix('.stdout').read_text(),row

def collect(out):
 out.mkdir(exist_ok=False);(out/'commands').mkdir();base=json.loads((R/'closure.json').read_text());old={i['path']:i for i in base['resources']}
 obs=json.loads((R/'environment_r05_attempt01/wrapper_observations.json').read_text());env=dict(obs['units'][0]['normalized_real_exec_environment'])
 # The only query-environment change isolates rospack's disposable cache from
 # canonical future run directories. No service or ROS node is started.
 env['ROS_HOME']=str(out/'isolated_rospack_cache');Path(env['ROS_HOME']).mkdir();commands=[];actual={}
 for attr in ATTRS:
  text,row=command(out/'commands',attr,['/opt/ros/noetic/bin/rospack','plugins','--attrib='+attr,'gazebo_ros'],env);commands.append(row);actual[attr]=exports(text)
 packages=sorted({r['package'] for rows in actual.values() for r in rows}|{'gazebo_ros'});manifests={}
 for package in packages:
  text,row=command(out/'commands','find_'+package,['/opt/ros/noetic/bin/rospack','find',package],env);commands.append(row);path=Path(text.strip())/'package.xml'
  if ET.fromstring(path.read_bytes()).findtext('name')!=package:raise ValueError('manifest package identity differs')
  manifests[package]=binding(path)
 for attr in ATTRS:
  expanded=[r for pkg,b in manifests.items() if pkg!='gazebo_ros' for r in manifest_exports(b['path'],attr)]
  key=lambda r:(r['package'],r['declared_path'])
  if collections.Counter(map(key,expanded))!=collections.Counter(map(key,actual[attr])):raise ValueError('rospack output/manifest expansion differs: '+attr)
 additions=[b for pkg,b in manifests.items() if b['path'] not in old]
 if {pkg for pkg,b in manifests.items() if b['path'] not in old}!={'gazebo_plugins','simulator_setup','turtlebot3_gazebo'}:raise ValueError('unexpected exporter additions')
 plugins=json.loads((R/'collection_r05_attempt04/configured_plugins.json').read_text());candidates=[];absent=[]
 for exported in actual['plugin_path']:
  for plugin in plugins:
   alias=Path(exported['declared_path'])/plugin['filename']
   if alias.is_file():
    b=binding(alias)
    if old.get(b['path'])!=b or b['path']!=plugin['selected']:raise ValueError('export adds unbound/different configured plugin')
    candidates.append(dict(exported,filename=plugin['filename'],alias=str(alias),binding=b,same_as_existing_selected=True))
   else:absent.append(dict(exported,filename=plugin['filename'],alias=str(alias),observed_exists=alias.exists()))
 gazeborc=Path(manifests['gazebo_ros']['path']).parent/'.do_not_use_gazeborc'
 if gazeborc.exists() or gazeborc.is_symlink():raise ValueError('unexpected GAZEBORC sentinel exists')
 source=S/'audits/navigation_n06_runtime_disk_closure_independent/r05_final/upstream_gazebo_ros_paths_plugin_2.9.3.cpp';source_text=source.read_text()
 required=['getPlugins("gazebo_ros","'+a+'"' for a in ATTRS]+['getPath("gazebo_ros")+"/.do_not_use_gazeborc"','setenv("GAZEBORC",gazeborc.c_str(),1)']
 if any(token not in source_text for token in required):raise ValueError('versioned source semantics differs')
 # The CLI is evidence tooling; runtime uses already-bound librospack/libroslib.
 tool=binding('/opt/ros/noetic/bin/rospack');text,row=command(out/'commands','rospack_ldd',['/usr/bin/ldd',tool['path']],env);commands.append(row)
 deps,virtual=parse_ldd(text);tool_deps=[binding(d['alias']) for d in deps]
 if any(old.get(b['path'])!=b for b in tool_deps):raise ValueError('query tool requires previously unbound ELF dependency')
 prepared=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text())
 for unit in prepared['units']:
  for field in ('world','robot_description'):
   xml=ET.fromstring(Path(unit[field]['path']).read_bytes())
   if any((n.text or '').strip() for n in xml.findall('.//uri')):raise ValueError('external media/model URI requires broader asset closure')
 result={'schema':'astra_n06_ros_package_exports_supplement_v1','status':'author_export_supplement_pass_independent_review_pending',
  'parent_closure':binding(R/'closure.json'),'parent_resources_unchanged':all(binding(p)==b for p,b in old.items()),
  'execution_environment_source':binding(R/'environment_r05_attempt01/wrapper_observations.json'),'query_env_only_override':{'ROS_HOME':env['ROS_HOME']},
  'export_counts':{a:len(r) for a,r in actual.items()},'actual_exports':actual,'all_exporter_and_gazebo_ros_manifests':manifests,'runtime_resources_added':additions,
  'configured_plugins_in_export_paths':candidates,'configured_plugin_absence_observations':absent,
  'new_configured_ELF_files':0,'GAZEBORC':{'source':binding(source),'derived_path':str(gazeborc),'observed_exists':False,
    'scope':'Source-derived override and current disk absence; no execution of system plugin, no immutable-absence/live claim.'},
  'versioned_upstream_binary_equivalence_claimed':False,'query_tool':tool,'query_tool_elf_dependencies':tool_deps,'virtual_dependencies':virtual,
  'commands':commands,'external_world_model_or_media_uris':0,'ros_gazebo_started':False,'execution_authorized':False,
  'scope':'Package export inputs and all configured-library candidates; not enumeration/qualification of every unused exported asset tree or future directory contents.'}
 if not result['parent_resources_unchanged']:raise ValueError('parent closure drift')
 write(out/'supplement.json',result);return result

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
 try:r=collect(args.output);print(json.dumps({'added':len(r['runtime_resources_added']),'counts':r['export_counts'],'same_configured_candidates':len(r['configured_plugins_in_export_paths'])}))
 except BaseException as exc:
  if args.output.exists():write(args.output/'failure.json',{'error':repr(exc),'ros_gazebo_started':False})
  raise
if __name__=='__main__':main()
