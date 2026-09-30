"""Independent ELF-byte, raw-evidence and root audit; never starts ROS/Gazebo."""
import copy, hashlib, json, os, pathlib, re, struct, subprocess, sys
import xml.etree.ElementTree as ET

S = pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
W = S/'workspace'
A = pathlib.Path(__file__).parent
D = S/'audits/navigation_n06_runtime_closure_r01'
C = D/'collection_r05_attempt04'
checks = []

def read(path):
    return json.loads(pathlib.Path(path).read_text())

def check(name, condition, detail=None):
    checks.append({'name': name, 'pass': bool(condition), 'detail': detail})
    assert condition, (name, detail)

def write(name, value):
    p = A/name
    assert not p.exists(), str(p)
    p.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')

def elf_bytes(path):
    """Read ELF64 structures directly, independent of author readelf regex."""
    b = pathlib.Path(path).read_bytes()
    assert b[:6] == b'\x7fELF\x02\x01', path
    h = struct.unpack_from('<HHIQQQIHHHHHH', b, 16)
    assert h[1] == 62 and h[2] == 1
    phoff, shoff, phentsize, phnum, shentsize, shnum = h[4], h[5], h[8], h[9], h[10], h[11]
    assert phentsize in (0, 56) and shentsize == 64 and shnum > 0
    interpreters = []
    for i in range(phnum):
        ph = struct.unpack_from('<IIQQQQQQ', b, phoff+i*phentsize)
        if ph[0] == 3:
            interpreters.append(b[ph[2]:ph[2]+ph[5]].rstrip(b'\0').decode())
    sections = [struct.unpack_from('<IIQQQQIIQQ', b, shoff+i*shentsize) for i in range(shnum)]
    needed, rpath, runpath = [], [], []
    for sh in sections:
        if sh[1] != 6:
            continue
        strings = sections[sh[6]]
        table = b[strings[4]:strings[4]+strings[5]]
        assert sh[9] == 16
        for off in range(sh[4], sh[4]+sh[5], 16):
            tag, val = struct.unpack_from('<qQ', b, off)
            if tag == 0:
                break
            if tag in (1, 15, 29):
                text = table[val:table.index(b'\0', val)].decode()
                {1:needed, 15:rpath, 29:runpath}[tag].append(text)
    return {'needed':needed, 'interpreters':interpreters, 'rpath':rpath, 'runpath':runpath}

def raw_ldd(text):
    resolved, missing, virtual = [], [], []
    for line in text.splitlines():
        words = line.split()
        if not words:
            continue
        if words[0].startswith(('linux-vdso.', 'linux-gate.')):
            virtual.append(line.strip())
        elif words[-2:] == ['not', 'found']:
            assert len(words) == 4 and words[1] == '=>'
            missing.append(words[0])
        elif len(words) == 4 and words[1] == '=>' and words[2].startswith('/'):
            resolved.append((words[0], words[2]))
        elif len(words) == 2 and words[0].startswith('/'):
            resolved.append((pathlib.Path(words[0]).name, words[0]))
        else:
            assert line.strip() in ('statically linked', 'not a dynamic executable'), line
    return resolved, missing, virtual

collection = read(C/'collection.json')
closure = read(D/'closure.json')
graph = read(C/'elf_graph.json')
commands = read(C/'commands.json')
resources = {r['path']:r for r in collection['resources']}
check('same_5214_closed_resource_roster', len(resources)==5214 and closure['resources']==collection['resources'])
check('author_status_not_execution_authority', closure['status']=='authored_disk_dependency_closure_independent_review_pending' and closure['independent_review'] is False and closure['execution_authorized'] is False)
contexts = graph['contexts']
check('two_exact_contexts', set(contexts)=={'manager_python','gzserver_after_actual_wrapper'})
obs = read(D/'environment_r05_attempt01/wrapper_observations.json')
spec = read(D/'environment_r05_attempt01/execution_environment.json')
check('context_environment_links', contexts['manager_python']==dict(spec['values'],ASTRA_N06_RUNTIME_CLOSURE_SHA256='0'*64) and contexts['gzserver_after_actual_wrapper']==obs['units'][0]['normalized_real_exec_environment'])
index = {}
for row in commands:
    assert row['returncode']==0 and row['error'] is None and row['cwd']==str(W)
    env_context = [k for k,v in contexts.items() if v==row['environment']]
    assert len(env_context)==1
    key = (env_context[0], tuple(row['argv']))
    assert key not in index
    index[key] = row
    for field in ('stdout','stderr'):
        data=pathlib.Path(row[field]['path']).read_bytes()
        assert hashlib.sha256(data).hexdigest()==row[field]['sha256']
    raw_record = pathlib.Path(row['stdout']['path']).with_suffix('.json')
    assert read(raw_record)==row
check('3593_raw_commands_exact_environment_and_records', len(index)==3593)

parsed = {}
for p in {n['file']['path'] for n in graph['nodes']}:
    parsed[p] = elf_bytes(p)
nodes = {(n['loader_context'],n['file']['path']):n for n in graph['nodes']}
check('602_ELFs_two_contexts_no_duplicate', len(parsed)==602 and len(nodes)==1204 and len(graph['nodes'])==1204 and all((c,p) in nodes for c in contexts for p in parsed))

def validate_node(n, resource_map=resources):
    p=n['file']['path']; context=n['loader_context']; actual=parsed[p]
    assert n['file']==resource_map[p]
    assert n['needed']==actual['needed']
    assert [str(pathlib.Path(p).resolve()) for p in actual['interpreters']]==[r['path'] for r in n['program_interpreters']]
    for bound in n['program_interpreters']:
        assert bound==resource_map[bound['path']] and (context,bound['path']) in nodes
    for flag in ('--dynamic','--program-headers'):
        row=index[(context,('/usr/bin/readelf',flag,p))]
        assert row['environment']==contexts[context]
    if actual['needed'] or actual['interpreters']:
        row=index[(context,('/usr/bin/ldd',p))]
        direct, missing, virtual = raw_ldd(pathlib.Path(row['stdout']['path']).read_text())
    else:
        direct, missing, virtual = [], [], []
    assert missing==n['standalone_unresolved'] and virtual==n['virtual_not_disk_resources']
    expected = {name:str(pathlib.Path(alias).resolve()) for name,alias in direct}
    actual_resolved = {}
    for dep in n['resolved']:
        assert dep['binding']==resource_map[dep['binding']['path']]
        assert str(pathlib.Path(dep['alias']).resolve())==dep['binding']['path']
        assert (context,dep['binding']['path']) in nodes
        assert dep['soname'] not in actual_resolved
        actual_resolved[dep['soname']]=dep['binding']['path']
    assert set(actual_resolved)==set(expected)|set(missing)
    assert set(actual['needed']) <= set(actual_resolved)
    for name,path in expected.items(): assert actual_resolved[name]==path
    assert bool(missing)==bool(n['inherited_root_contexts'])
    choices = set()
    for parent in n['inherited_root_contexts']:
        src=nodes[(context,parent['path'])]
        assert src['file']==parent and p in {d['binding']['path'] for d in src['resolved']}
        table={d['soname']:d['binding']['path'] for d in src['resolved']}
        assert all(name in table for name in missing)
        choices.add(tuple(table[name] for name in missing))
    if missing:
        assert len(choices)==1 and tuple(actual_resolved[name] for name in missing)==next(iter(choices))

for node in graph['nodes']: validate_node(node)
check('1204_actual_ELF_headers_raw_ldd_recursive_edges_and_inherited_RPATH', True, {'private_context_rows':sum(bool(n['standalone_unresolved']) for n in graph['nodes']), 'interpreter_rows':sum(bool(n['program_interpreters']) for n in graph['nodes'])})
faults=[]
sample=next(n for n in graph['nodes'] if n['needed'])
cases=[]
n=copy.deepcopy(sample); n['needed']=[]; cases.append(('removed_actual_needed',n,resources))
n=copy.deepcopy(sample); n['resolved'][0]['soname']='wrong.so'; cases.append(('wrong_soname',n,resources))
n=copy.deepcopy(sample); n['resolved'][0]['binding']['sha256']='0'*64; cases.append(('wrong_library_hash',n,resources))
n=copy.deepcopy(sample); n['loader_context']='unobserved'; cases.append(('unobserved_environment',n,resources))
pr=next(n for n in graph['nodes'] if n['standalone_unresolved'])
n=copy.deepcopy(pr); n['inherited_root_contexts']=[]; cases.append(('invented_unproven_parent_resolution',n,resources))
inter=next(n for n in graph['nodes'] if n['program_interpreters'])
n=copy.deepcopy(inter); n['program_interpreters']=[]; cases.append(('missing_actual_interpreter',n,resources))
missing=dict(resources);del missing[sample['resolved'][0]['binding']['path']];cases.append(('dependency_absent_from_roster',sample,missing))
for name,n,r in cases:
    try: validate_node(n,r)
    except (AssertionError,KeyError):faults.append(name)
    else:raise AssertionError(name)
check('seven_graph_mutations_rejected_independently', len(faults)==7, faults)
aliases=read(C/'aliases.json')
for row in aliases:
    assert str(pathlib.Path(row['alias']).resolve())==row['canonical'] and row['canonical'] in resources
    for link in row['final_component_links']:assert os.readlink(link['path'])==link['link_text']
check('all_alias_targets_and_link_text_match',True,len(aliases))

prepared=read(S/'contracts/navigation_n06_prepared_r01/prepared.json')
plugins=set()
for u in prepared['units']:
    for field in ('world','robot_description'):
        root=ET.parse(u[field]['path']).getroot()
        plugins.update(n.attrib['filename'] for n in root.iter('plugin'))
        assert not any((n.text or '').strip() for n in root.iter('uri'))
plugins.update(('libgazebo_ros_paths_plugin.so','libgazebo_ros_api_plugin.so'))
plugin_rows=read(C/'configured_plugins.json')
assert plugins=={r['filename'] for r in plugin_rows}
for row in plugin_rows:
    paths=[p for p in contexts['gzserver_after_actual_wrapper']['GAZEBO_PLUGIN_PATH'].split(':') if p]
    choices=[str(pathlib.Path(p)/row['filename']) for p in paths if (pathlib.Path(p)/row['filename']).is_file()]
    assert row['search_paths']==paths and row['candidates']==choices
    assert row['selected']==str(pathlib.Path(choices[0]).resolve()) and row['selected'] in graph['roots']
check('nine_actual_configured_plugins_final_wrapper_precedence',len(plugin_rows)==9)
for name in ('/usr/bin/gzserver','/usr/bin/env','/bin/sh','/bin/bash','/opt/ros/noetic/lib/rosout/rosout'):
    assert str(pathlib.Path(name).resolve()) in graph['roots']
for name in ('/opt/ros/noetic/bin/roslaunch','/opt/ros/noetic/bin/rosmaster','/opt/ros/noetic/bin/roscore','/opt/ros/noetic/lib/gazebo_ros/libcommon.sh','/etc/ld.so.cache','/etc/ld.so.conf'):
    assert str(pathlib.Path(name).resolve()) in resources
roles=read(C/'resource_roles.json')
for role in ('wrapper_pkg_config_input','ogre_dlopen_envelope','exact_frozen_graphics_policy','explicit_python_ctypes_dependency','python_startup_configuration'):
    assert any(role in r for r in roles.values())
check('runtime_shell_gzserver_loader_dlopen_and_startup_roots_present',True)

# Repeat imports/launch XML expansion under exact frozen safe environment.
# The reviewed probe does no constructor/main/ROS initialization calls.
profile_results=[]
for profile,python in [('venv',W/'.venv/bin/python'),('system_launcher',pathlib.Path('/usr/bin/python3'))]:
    output=A/(profile+'_independent_replay.json')
    argv=[str(python),str(D/'python_import_probe.py'),'--profile',profile,'--output',str(output),'--manager-revision','r05']
    assert not output.exists()
    proc=subprocess.run(argv,env=contexts['manager_python'],cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=60)
    (A/(profile+'_replay.stdout')).write_bytes(proc.stdout);(A/(profile+'_replay.stderr')).write_bytes(proc.stderr)
    write(profile+'_replay_command.json',{'argv':argv,'environment':contexts['manager_python'],'returncode':proc.returncode,'ros_gazebo_started':False})
    assert proc.returncode==0
    actual=read(output);expected=read(C/(profile+'.json'))
    assert actual==expected, sorted(k for k in actual if actual[k]!=expected.get(k))
    assert actual['complete_environment']==contexts['manager_python'] and actual['no_user_site']
    for mod in actual['modules']:
        assert mod['path'] in resources
        if mod['cached'] and pathlib.Path(mod['cached']).is_file():assert str(pathlib.Path(mod['cached']).resolve()) in resources
    assert not any('/.local/' in p for p in actual['sys_path'])
    for row in actual['resolved_nodes']:
        assert row['entry_canonical'] in resources and not row['env_args'] and not row['roslaunch_environment_delta']
        assert not row['interpreter'] or row['interpreter']['canonical'] in resources
    profile_results.append({'profile':profile,'module_count':len(actual['modules']),'node_count':len(actual['resolved_nodes']),'byte_structure_equal':True})
check('both_real_interpreter_import_and_64_launch_node_replays_equal',True,profile_results)
write('independent_results.json',{'status':'independent_disk_graph_roots_imports_checks_pass','checks':checks,'elf_byte_records':parsed,'ros_gazebo_started':False,'release_or_seed_writes':False})
print(json.dumps({'checks_passed':len(checks),'graph_negative_cases':len(faults),'ELFs':len(parsed),'profiles':profile_results}))
