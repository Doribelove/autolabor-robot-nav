"""Independent saved-converter evidence audit; never invokes gz, ROS or Gazebo."""
import copy,hashlib,json,os,sys,tempfile,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import dense_sweep_n05 as dense
B=S/'contracts/navigation_n06_conversion_r01/bundle.json';R=S/'audits/navigation_n06_root_conversion_review_r01.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def b(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def read(item):
 assert sha(item['path'])==item['sha256'];return json.loads(Path(item['path']).read_text())
def maps(path):
 rows=[]
 for line in Path(path).read_text().splitlines():
  fields=line.split(None,5)
  if len(fields)<6:continue
  _addr,perms,_off,dev,inode,name=fields
  if 'x' not in perms or not name.startswith('/'):continue
  a,c=dev.split(':');rows.append({'name':name,'major':int(a,16),'minor':int(c,16),'inode':int(inode)})
 return rows
assert sha(B)=='51dc54250991519254dcca93da54461dac9bd05fc3b0a6b96087221646b21339'
assert sha(R)=='0bed9675a02081ea93a28ef6d6d8e61ee9f1ec3c1299e7864b78353fc5610c01'
bundle=json.loads(B.read_text());root=json.loads(R.read_text());prepared=read(bundle['prepared']);all_resources={x['path']:x for x in root['resources']}
assert len(all_resources)==438
for item in all_resources.values():assert sha(item['path'])==item['sha256']
assert len(bundle['units'])==len(prepared['units'])==8
rows=[];lifetimes=set();negative=[]
for unit,expected in zip(bundle['units'],prepared['units']):
 assert unit['run_id']==expected['run_id'] and unit['input_urdf']==expected['robot_description']
 m=read(unit['conversion_manifest']);assert m['input_urdf']==unit['input_urdf'] and m['converted_sdf']==unit['converted_sdf']
 assert dense.conversion_resources(unit['conversion_manifest']['path'])==unit['conversion_resources']
 for item in unit['conversion_resources']:assert all_resources.get(item['path'])==item
 left=Path(m['converted_sdf']['path']).read_bytes();right=Path(m['reproduced_sdf']['path']).read_bytes();assert left==right and Path(m['converted_sdf']['path'])!=Path(m['reproduced_sdf']['path'])
 module_expected={x['path']:x['sha256'] for x in [m['converter_executable']]+m['loaded_libraries']};processes=[]
 assert len(m['process_evidence'])==2
 for i,pb in enumerate(m['process_evidence']):
  process=read(pb);identity=(process['pid'],process['proc_start_ticks']);assert identity not in lifetimes;lifetimes.add(identity)
  assert process['exit_code']==0 and process['command']==[m['converter_executable']['path'],'sdf','-p',unit['input_urdf']['path']]
  assert process['stdout']==m['converted_sdf' if i==0 else 'reproduced_sdf']
  assert sha(process['maps']['path'])==process['maps']['sha256'];parsed=maps(process['maps']['path'])
  assert {x['path']:x['sha256'] for x in process['mapped_elf_files']}==module_expected
  for module in process['mapped_elf_files']:
   assert any(x=={'name':module['mapped_path'],'major':module['device_major'],'minor':module['device_minor'],'inode':module['inode']} for x in parsed)
   stat=Path(module['path']).stat();assert stat.st_ino==module['inode'] and os.major(stat.st_dev)==module['device_major'] and os.minor(stat.st_dev)==module['device_minor']
   assert sha(module['path'])==module['sha256']
  processes.append({'pid':identity[0],'start_ticks':identity[1],'record':pb,'raw_maps':process['maps'],'mapped_elf_count':len(process['mapped_elf_files']),'output':process['stdout']})
 sdf=ET.fromstring(left);model=sdf.find('model');assert model is not None
 links=model.findall('link');collisions=model.findall('link/collision');assert len(links)==7 and len(collisions)==9
 assert abs(sum(float(x.findtext('inertial/mass')) for x in links)-79.6)<1e-12
 driver=next(x for x in model.findall('plugin') if x.get('filename')=='libastra_physical_ackermann_p05_plugin.so')
 assert int(driver.findtext('dynamicsSeed'))==expected['seed'] and float(driver.findtext('steeringKd'))==6.
 auth=dense.authenticate_conversion(unit['conversion_manifest']['path'],unit['input_urdf']['path'],unit['converted_sdf']['path'],{},diagnostic=True)
 assert auth['pass'] and auth['runtime_converter_correspondence_qualified']is False
 rows.append({'run_id':unit['run_id'],'seed':expected['seed'],'pass_':True,'diagnostic_authentication':auth,'body_count':7,'collision_count':9,'mass_kg':sum(float(x.findtext('inertial/mass')) for x in links),'two_distinct_processes':processes,'same_output_bytes':True})
 # For first actual input, fault copies preserve original files and prove strict rejection.
 if len(rows)==1:
  with tempfile.TemporaryDirectory(prefix='n06_conversion_fault_') as temp:
   path=Path(temp)/'manifest.json'
   for name,change in [('wrong_schema',lambda x:x.update(schema='foreign')),('duplicate_process',lambda x:x.update(process_evidence=[x['process_evidence'][0]]*2)),('false_reproduction',lambda x:x.update(output_bytes_reproduced=False)),('missing_runtime_role',lambda x:x.update(runtime_correspondence_libraries=[]))]:
    altered=copy.deepcopy(m);change(altered);path.write_text(json.dumps(altered))
    result=dense.authenticate_conversion(path,unit['input_urdf']['path'],unit['converted_sdf']['path'],{},diagnostic=True);assert not result['pass'];negative.append({'case':name,'rejected':True,'issues':result['issues']})
   no_live=dense.authenticate_conversion(unit['conversion_manifest']['path'],unit['input_urdf']['path'],unit['converted_sdf']['path'],{},diagnostic=False)
   assert not no_live['pass'];negative.append({'case':'no_actual_runtime_mapped_sdformat','rejected':True,'issues':no_live['issues']})
assert len(lifetimes)==16
pairs=[]
for i in range(0,8,2):
 a,c=bundle['units'][i:i+2];assert a['converted_sdf']['sha256']==c['converted_sdf']['sha256'];pairs.append({'runs':[a['run_id'],c['run_id']],'same_SDF_bytes':True})
for item in all_resources.values():assert sha(item['path'])==item['sha256']
extra=[B,R,Path(dense.__file__),P/'mapped_elf_inventory.py',Path(__file__)]
for p in extra:all_resources[str(p.resolve())]=b(p)
(A/'resources.json').write_text(json.dumps({'resources':list(all_resources.values()),'scope':'saved conversion source/provenance, not runtime release'},indent=2)+'\n')
report={'schema':'astra_n06_conversion_independent_v1','status':'independent_eight_offline_conversion_provenance_pass_runtime_pending','independent_review':True,'bundle':b(B),'root_report':b(R),'unique_original_resources_rehashed':438,'resources':b(A/'resources.json'),'units':rows,'four_pairs':pairs,'distinct_converter_lifetimes':16,'counterexamples':negative,'runtime_converter_correspondence_qualified':False,'Gazebo_physical_execution_qualified':False,'review_invoked_converter':False,'ROS_or_Gazebo_started':False,'release_created':False,'seed_registered':False,'limitations':['Recorded converter maps, argv, input/output provenance and saved exact byte reproduction verified; this review did not independently rerun gz sdf -p.','Live Gazebo must later prove mapped libsdformat matches this exact binary and its converted-model closure; absence correctly fails diagnostic=False.','Four pairs identical SDF within pair does not prove identical eventual physics outcomes or lookahead performance.']}
(A/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');(A/'review.md').write_text('# N06 八份离线转换独审\n\n438 原始资源前后重哈希一致；8份URDF输入、prepared顺序及每份2次转换输出逐字节一致。独立解析16份raw maps并核对真实PID/startticks、argv/input、stdout、全部ELF的执行映射device/inode和当前文件SHA；16个进程生命周期均不同。四对SDF字节分别一致；每份7个合并刚体、9个collision、质量79.6kg，P05/Kd6/seed保持。\n\n8次diagnostic authenticate通过；错schema、重复process、假重现标志、缺sdformat角色均拒绝。没有实际Gazebo maps时diagnostic=False按预期拒绝。未重跑converter、未启动ROS/Gazebo、未登记seed或创建release；这里只认证已保存转换出处和字节重现，实际运行sdformat对应与物理跟踪仍未资格。\n');print(json.dumps({'review':b(A/'review.json'),'resources':b(A/'resources.json'),'units':8,'converter_lifetimes':16},ensure_ascii=False))
