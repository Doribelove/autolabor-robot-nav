"""Incremental independent review of frozen export supplement and r02 union."""
import collections, copy, hashlib, json, os, pathlib, subprocess
import xml.etree.ElementTree as ET

S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
W=S/'workspace';A=pathlib.Path(__file__).resolve().parent
D=S/'audits/navigation_n06_runtime_exports_supplement_r01'
R=S/'audits/navigation_n06_runtime_closure_r01'
R2=S/'audits/navigation_n06_runtime_closure_r02'
checks=[];read_bindings={}
def binding(p):
    p=pathlib.Path(p).resolve();h=hashlib.sha256()
    with p.open('rb') as f:
        before=os.fstat(f.fileno())
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
        after=os.fstat(f.fileno())
    assert (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns,before.st_ctime_ns)==(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns,after.st_ctime_ns)
    r={'path':str(p),'sha256':h.hexdigest()};read_bindings[str(p)]=r;return r
def js(p):binding(p);return json.loads(pathlib.Path(p).read_text())
def bound(r):assert binding(r['path'])==r;return pathlib.Path(r['path']).read_bytes()
def check(n,c,detail=None):checks.append({'name':n,'pass':bool(c),'detail':detail});assert c,(n,detail)
def write(name,value):
    p=A/name;assert not p.exists();p.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')

for p,h in [(D/'implementation_report.json','51552cdf11efda12c46fb13b83a5bbe5920e45e6072a0c6060519b9a1b5592d3'),(D/'source_evidence_manifest.json','a722745d92b7e4b3c0a123c7d623d3d0d23adad0b2beee4958cb40417e03fed6'),(R2/'closure.json','af6c053e0efcd78493b889f40b873c172ff94ec5cd9140274dd17babb568d6de')]:
    assert binding(p)['sha256']==h
report=js(D/'implementation_report.json');manifest=js(D/'source_evidence_manifest.json')
for group in ('owned_sources','read_only_dependencies','author_evidence'):
    for r in manifest[group]:bound(r)
bound(manifest['new_closure'])
old=js(R/'closure.json');new=js(R2/'closure.json');supp=json.loads(bound(report['supplement']))
original_manifest=js(R/'source_evidence_manifest.json');old_all={}
for group in ('owned_sources','runtime_resources','author_evidence','manager_and_risk_bindings'):
    for r in original_manifest[group]:old_all[r['path']]=r
for r in old_all.values():bound(r)
check('all_26921_original_author_resources_unchanged',len(old_all)==26921)
check('original_closure_exact_historical_hash',binding(R/'closure.json')['sha256']=='25f882db92b529ee5a16b1350807d56d4ed053383f9ccc41d2ff93fba0280c93')
prior=S/'audits/navigation_n06_runtime_disk_closure_independent/r05_final/review.json'
check('prior_negative_exact_bytes_retained',binding(prior)['sha256']=='5fc6cb26f7ffe3ca90d257e7e2372ebb371b1dee750e1e870c3e3d07449b21cf')
old_rows={r['path']:r for r in old['resources']};new_rows={r['path']:r for r in new['resources']}
for r in new_rows.values():bound(r)
expected_add={r['path']:r for r in report['added_runtime_resources']}
assert set(new_rows)-set(old_rows)==set(expected_add)
assert not set(old_rows)-set(new_rows) and all(new_rows[p]==r for p,r in old_rows.items())
assert all(new_rows[p]==r for p,r in expected_add.items())
check('5217_exact_resource_union_three_manifests_only',len(new['resources'])==len(new_rows)==5217 and len(expected_add)==3)
changed={k for k in set(new)|set(old) if old.get(k)!=new.get(k)}
check('same_running_candidate_semantics_and_environment',changed=={'resources','parent_closure','export_input_supplement','prior_independent_negative_review'},sorted(changed))
check('r02_is_pending_no_authority',new['status']=='authored_disk_dependency_closure_independent_review_pending' and new['execution_authorized'] is False and new['independent_review'] is False and new['live_mapped_elf_observed'] is False)
assert new['parent_closure']==binding(R/'closure.json') and new['export_input_supplement']==report['supplement'] and new['prior_independent_negative_review']==binding(prior)

obs=json.loads(bound(new['wrapper_environment_observations']));base_env=obs['units'][0]['normalized_real_exec_environment']
cache=A/'rospack_cache';cache.mkdir(exist_ok=False);env=dict(base_env,ROS_HOME=str(cache))
outputs={};commands=[]
attrs=('plugin_path','gazebo_media_path','gazebo_model_path')
def parse(text):
    return [tuple(line.split(' ',1)) for line in text.splitlines() if line]
for attr in attrs:
    argv=['/opt/ros/noetic/bin/rospack','plugins','--attrib='+attr,'gazebo_ros']
    p=subprocess.run(argv,env=env,cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=30)
    (A/(attr+'.stdout')).write_bytes(p.stdout);(A/(attr+'.stderr')).write_bytes(p.stderr)
    commands.append({'argv':argv,'environment':env,'returncode':p.returncode,'stdout':binding(A/(attr+'.stdout')),'stderr':binding(A/(attr+'.stderr'))})
    assert p.returncode==0 and not p.stderr
    outputs[attr]=parse(p.stdout.decode())
    assert outputs[attr]==[(r['package'],r['declared_path']) for r in supp['actual_exports'][attr]]
    author=next(c for c in supp['commands'] if c['argv']==argv)
    assert author['returncode']==0 and author['cwd']==str(W)
    assert {k:v for k,v in author['environment'].items() if k!='ROS_HOME'}=={k:v for k,v in base_env.items() if k!='ROS_HOME'}
    assert pathlib.Path(author['environment']['ROS_HOME']).parent==D/'attempt01'
    assert p.stdout==bound(author['stdout']) and not bound(author['stderr'])
write('actual_query_commands.json',commands)
check('actual_rospack_19_19_20_exact_ordered_replay', [len(outputs[a]) for a in attrs]==[19,19,20])

manifests=supp['all_exporter_and_gazebo_ros_manifests'];expected_exports={a:[] for a in attrs}
for name,r in manifests.items():
    raw=bound(r);root=ET.fromstring(raw)
    assert root.findtext('name')==name and new_rows[r['path']]==r
    author=next(c for c in supp['commands'] if c['argv']==['/opt/ros/noetic/bin/rospack','find',name])
    assert pathlib.Path(bound(author['stdout']).decode().strip())==pathlib.Path(r['path']).parent
    for node in root.findall('./export/gazebo_ros'):
        for attr in attrs:
            if attr in node.attrib:
                value=node.attrib[attr].replace('${prefix}',str(pathlib.Path(r['path']).parent))
                assert value.startswith('/') and '$' not in value
                expected_exports[attr].append((name,value))
def validate_export_rows(rows):
    for a in attrs:
        assert collections.Counter(rows[a])==collections.Counter(expected_exports[a])
validate_export_rows(outputs)
check('all_five_package_identities_and_exact_XML_exports',len(manifests)==5)
faults=[]
for name,mutate in [('missing_export',lambda x:x['plugin_path'].pop()),('duplicate_export',lambda x:x['plugin_path'].append(x['plugin_path'][0])),('different_path',lambda x:x['plugin_path'].__setitem__(0,(x['plugin_path'][0][0],'/synthetic/not-loaded'))),('wrong_package',lambda x:x['plugin_path'].__setitem__(0,('wrong',x['plugin_path'][0][1])))]:
    copy_rows=copy.deepcopy(outputs);mutate(copy_rows)
    try:validate_export_rows(copy_rows)
    except AssertionError:faults.append(name)
    else:raise AssertionError(name)
check('four_export_equality_negative_cases_rejected',len(faults)==4,faults)

plugins=js(R/'collection_r05_attempt04/configured_plugins.json');observed=[];absences=[]
for package,directory in outputs['plugin_path']:
    for row in plugins:
        p=pathlib.Path(directory)/row['filename']
        if p.is_file():
            r=binding(p);assert r==old_rows[r['path']] and r['path']==row['selected']
            observed.append((package,directory,row['filename'],str(p),r['path'],r['sha256']))
        else:absences.append((package,directory,row['filename'],str(p),p.exists()))
declared=[(r['package'],r['declared_path'],r['filename'],r['alias'],r['binding']['path'],r['binding']['sha256']) for r in supp['configured_plugins_in_export_paths']]
declared_abs=[(r['package'],r['declared_path'],r['filename'],r['alias'],r['observed_exists']) for r in supp['configured_plugin_absence_observations']]
check('all_171_export_directory_filename_pairs_checked_same_six_ELFs',observed==declared and len(observed)==6 and absences==declared_abs and len(absences)==165)
source=bound(supp['GAZEBORC']['source']).decode();gazeborc=pathlib.Path(manifests['gazebo_ros']['path']).parent/'.do_not_use_gazeborc'
assert 'getPath("gazebo_ros")+"/.do_not_use_gazeborc"' in source and 'setenv("GAZEBORC",gazeborc.c_str(),1)' in source
check('GAZEBORC_source_derivation_and_current_absence_only',str(gazeborc)==supp['GAZEBORC']['derived_path'] and not gazeborc.exists() and not gazeborc.is_symlink() and supp['GAZEBORC']['observed_exists'] is False and supp['versioned_upstream_binary_equivalence_claimed'] is False)
for a in attrs:assert 'getPlugins("gazebo_ros","'+a+'"' in source
bound(supp['query_tool'])
for r in supp['query_tool_elf_dependencies']:assert old_rows[r['path']]==r;bound(r)
for u in js(S/'contracts/navigation_n06_prepared_r01/prepared.json')['units']:
    for field in ('world','robot_description'):
        root=ET.fromstring(bound(u[field]));assert not any((n.text or '').strip() for n in root.iter('uri'))
check('query_tool_dependencies_in_old_graph_and_16_inputs_have_no_external_URI',True)
write('results.json',{'status':'independent_N06_disk_export_supplement_pass','checks':checks,'original_resource_hashes_verified':len(old_all),'runtime_resource_hashes_verified':len(new_rows),'negative_cases':faults,'read_bindings':sorted(read_bindings.values(),key=lambda r:r['path']),'execution_authorized':False,'ROS_or_Gazebo_started':False})
print(json.dumps({'checks_passed':len(checks),'runtime_resources':len(new_rows),'old_resources_unchanged':len(old_all),'export_counts':{a:len(outputs[a]) for a in attrs},'negative_cases':len(faults),'pass':True}))
