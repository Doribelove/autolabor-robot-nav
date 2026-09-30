#!/usr/bin/env python3
"""Saved GLX/Q03 mapping replay and exact narrow-policy faults; no ROS/GL live call."""
import ast,copy,datetime,hashlib,importlib.util,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=S/'audits/mapped_graphics_probe_v1';R=S/'runs/passive_sweep_q03_static'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for block in iter(lambda:f.read(1048576),b''):h.update(block)
 return h.hexdigest()
for name in ('mapped_elf_inventory.py','mapped_elf_inventory_v2.py'):
 p=W/'scripts/astra_gazebo'/name;dest=D/'snapshot'/name
 with dest.open('xb') as f:f.write(p.read_bytes())
sys.path.insert(0,str(D/'snapshot'));import mapped_elf_inventory as base;import mapped_elf_inventory_v2 as v2
q03=W/'scripts/astra_gazebo/passive_sweep_q03.py';roles=ast.literal_eval(next(n.value for n in ast.parse(q03.read_text()).body if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='ELF_ROLES'for t in n.targets)));required=set(roles.values())-{'actor_binaries'}
probe=json.loads((P/'report.json').read_bytes());captures=[]
for index,x in enumerate(probe['captures']):
 assert sha(x['path'])==x['sha256'];text=Path(x['path']).read_text();parsed=base.parse_maps(text);out=v2.inventory_from_text(text,{},set());aliases=v2.graphics_aliases(parsed);deleted=[r['raw_line']for r in parsed if r['deleted']]
 captures.append(dict(x,index=index,mapping_count=len(parsed),deleted_rows_exact=deleted==x['deleted_rows'],external_groups=len(out['external_graphics_dynamic_code']),aliases=aliases,loaded_graphics=out['loaded_modules']))
rawmaps=(R/'elf_pre.maps.txt').read_text();parsed=base.parse_maps(rawmaps);baseline=v2.inventory_from_text(rawmaps,roles,required);old_error=None
try:base.inventory_from_text(rawmaps,roles,required)
except base.InventoryError as exc:old_error=type(exc).__name__+': '+str(exc)
verified_cache={}
def cached_actual(path,row):
 key=(path,row['device_major'],row['device_minor'],row['inode'])
 if key not in verified_cache:verified_cache[key]=base.inspect_elf(path,row)
 return copy.deepcopy(verified_cache[key])
# Exercise actual source parser against actual saved map rows; altered rows are explicit counterexamples.
alias=baseline['external_graphics_dynamic_code'][0]['mapping_rows'];rx=next(x for x in alias if x['permissions']=='r-xs');rw=next(x for x in alias if x['permissions']=='rw-s');original=rawmaps.splitlines();cases=[]
def render(rows):return '\n'.join(r['raw_line']for r in rows)
def test(name,text,expected,inspector=cached_actual):
 error=None
 try:v2.inventory_from_text(text,roles,required,inspector)
 except base.InventoryError as exc:error=str(exc)
 passed=error is not None and expected in error;cases.append({'name':name,'synthetic_mutation_of_saved_map':True,'error':error,'expected_substring':expected,'pass':passed})
def replace_row(row,new):return '\n'.join(new if i+1==row['line_number']else t for i,t in enumerate(original))
test('missing_RW_alias','\n'.join(t for i,t in enumerate(original)if i+1!=rw['line_number']),'exactly one RX')
test('missing_RX_alias','\n'.join(t for i,t in enumerate(original)if i+1!=rx['line_number']),'exactly one RX')
for name,before,after,expected in [('private_RX','r-xs','r-xp','permissions differ'),('RWX_code','r-xs','rwxs','permissions differ'),('different_RW_inode',' '+str(rw['inode'])+' ',' '+str(rw['inode']+1)+' ','share backing inode'),('wrong_RW_offset','00000000','00001000','backing/size/offset'),('wrong_RW_device','00:01','00:02','backing/size/offset'),('not_deleted',' (deleted)','','backing/size/offset')]:
 row=rw if name.startswith('different_RW')or name.startswith('wrong_RW')else rx;test(name,replace_row(row,row['raw_line'].replace(before,after)),expected)
test('wrong_exact_memfd_name',replace_row(rx,rx['raw_line'].replace('/.glXXXXXX','/.glOTHER')),'exactly one RX')
test('wrong_code_size',replace_row(rx,rx['raw_line'].replace(format(rx['end'],'x'),format(rx['end']-4096,'x'),1)),'backing/size/offset')
for role_name in ('libgazebo_ode.so','libastra_physical_ackermann_p05_plugin.so','libgazebo_ros_api_plugin.so'):
 rows=[r for r in parsed if not Path(r['backing_path']).name.startswith(role_name)];test('required_role_removed_'+role_name,render(rows),'required ELF missing')
firstphysics=next(r for r in parsed if Path(r['backing_path']).name.startswith('libastra_physical_ackermann_p05_plugin.so'));test('deleted_required_custom_driver',replace_row(firstphysics,firstphysics['raw_line']+' (deleted)'),'deleted executable or role/ELF')
remaining=[r for r in parsed if r['line_number'] not in {a['line_number']for a in alias}]
for name,path,deleted,expected in [('unrelated_deleted_executable','/memfd:unrelated',True,'deleted executable or role/ELF'),('unknown_custom_plugin','/tmp/libastra_unreviewed.so',False,'unreviewed custom Astra')]:
 extra=dict(rx,raw_line=rx['raw_line'].replace(rx['mapped_path'],path+(' (deleted)'if deleted else'')));test(name,render(sorted(remaining+[extra],key=lambda r:r['start'])),expected)
for role in v2.GRAPHICS_BINDINGS:
 rows=[r for r in parsed if r['backing_path']!=v2.GRAPHICS_BINDINGS[role]['path']];test('missing_graphics_'+role,render(rows),'required ELF missing')
for name in ('digest','path'):
 def altered(path,row):
  x=cached_actual(path,row)
  if path==v2.GRAPHICS_BINDINGS['graphics_nvidia_glcore']['path']:x['sha256'if name=='digest'else'path']='f'*64 if name=='digest'else'/tmp/copied-driver'
  return x
 test('wrong_bound_graphics_'+name,rawmaps,'mapped graphics driver differs',altered)
# Fail-before-parse persistence with an offline fake proc tree, never live /proc.
fixture=D/'fake_proc';proc=fixture/'123';proc.mkdir(parents=True,exist_ok=True);bad='\n'.join(t for i,t in enumerate(original)if i+1!=rw['line_number']);(proc/'maps').write_text(bad);(proc/'stat').write_text('123 (fixture) '+' '.join(['1']*20));saved=D/'counterexample.maps.txt';error=None
try:v2.inventory(123,roles,required,saved,fixture)
except base.InventoryError as exc:error=str(exc)
audit=json.loads(Path(str(saved)+'.audit.json').read_bytes());cases.append({'name':'failed_exact_alias_keeps_raw_before_exception','pass':error is not None and saved.read_text()==bad and audit['raw_maps']['sha256']==sha(saved) and audit['interpretation_pass']is False,'error':error,'fixture':'Synthetic offline proc tree only'})
release=json.loads((S/'contracts/passive_sweep_q03_release.json').read_bytes());inheritance=[dict(x,current_matches=sha(x['path'])==x['sha256'])for x in release['resources']];driverbindings=[x for x in baseline['loaded_modules']if any(r in v2.GRAPHICS_ROLES.values()for r in x['roles'])];q03aliases=v2.graphics_aliases(parsed);normal=lambda rows:sorted((r['backing_path'],r['permissions'],r['deleted'],r['end']-r['start'],r['offset'],r['device_major'],r['device_minor'])for r in rows)
checks={'eight_exact_captures_and_SHA':len(captures)==8 and all(x['deleted_rows_exact']for x in captures),'strict_monotonic_capture_times':all(b['monotonic_ns']>a['monotonic_ns']for a,b in zip(captures,captures[1:])),'GL_probe_success_and_explicit_render_vendor':probe['success']is True and probe['GL']['vendor']=='NVIDIA Corporation','alias_absent_before_present_after_current_absent_after_cleanup':[x['external_groups']for x in captures]==[0,0,0,0,0,1,1,0],'Q03_same_normalized_observed_alias_pattern':normal(q03aliases)==normal(captures[5]['aliases']),'original_Q03_parser_reproduces_failed_result':old_error=='InventoryError: deleted executable or role/ELF mapping: /memfd:/.glXXXXXX (deleted)','actual_Q03_v2_all_required_physics_ROS_custom_roles_complete':required<={r for x in baseline['loaded_modules']for r in x['roles']},'three_actual_graphics_files_bound_SHA_BuildID':len(driverbindings)==3 and all(x.get('build_id') and x['sha256']==sha(x['path'])==v2.GRAPHICS_BINDINGS[x['roles'][0]]['sha256']for x in driverbindings),'all_focused_counterexamples_rejected':all(x['pass']for x in cases),'all766_Q03_resources_immutable':len(inheritance)==766 and all(x['current_matches']for x in inheritance),'base_inventory_source_exact_Q03_binding':any(x['path']==str(W/'scripts/astra_gazebo/mapped_elf_inventory.py')and x['sha256']==sha(D/'snapshot/mapped_elf_inventory.py')for x in inheritance),'source_bytes_stable_during_audit':all(sha(W/'scripts/astra_gazebo'/n)==sha(D/'snapshot'/n)for n in ('mapped_elf_inventory.py','mapped_elf_inventory_v2.py'))}
out={'status':'independent_saved_GL_mapping_and_v2_policy_review_pass'if all(checks.values())else'independent_graphics_mapping_review_failed','pass':all(checks.values()),'independent_review':True,'no_ROS_or_live_GL':True,'checks':checks,'reviewed_v2_sha256':sha(D/'snapshot/mapped_elf_inventory_v2.py'),'inherited_base_sha256':sha(D/'snapshot/mapped_elf_inventory.py'),'captures':captures,'actual_Q03_inventory_replay':baseline,'actual_graphics_bindings':driverbindings,'counterexamples':cases,'Q03_inherited_resources':inheritance,'qualification_scope':'Transparent narrow exception for exact locally observed graphics dynamic-code mapping pattern with all three exact disk-ELF renderdriver bindings. Every required physics/ROS/custom ELF retains strict role, ordinary-file backing, mapped-device/inode, SHA and BuildID checks.','non_claims':['Does not read or hash anonymous executable bytes.','GLX temporal appearance/disappearance and matching Q03 pattern are measured correlation, not byte-origin theorem.','Trusted local process scope, not protection against adversarial spoofed memfd patterns.','No Q03 result reclassification or live Q04 execution; no navigation/physical safety claim.'],'inputs':[{'path':str(p),'sha256':sha(p)}for p in [P/'report.json',P/'probe_glx.py',P/'probe.log',R/'elf_pre.maps.txt',R/'elf_pre.maps.txt.audit.json',S/'contracts/passive_sweep_q03_release.json',q03,D/'snapshot/mapped_elf_inventory.py',D/'snapshot/mapped_elf_inventory_v2.py']],'script_sha256':sha(__file__),'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()};(D/'review.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'pass':out['pass'],'checks':checks,'counterexamples':len(cases),'failed_cases':[x for x in cases if not x['pass']],'graphics':driverbindings,'report_sha256':sha(D/'review.json')},indent=2));assert out['pass']
