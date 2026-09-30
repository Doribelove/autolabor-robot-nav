"""Read-only real release check. Only --verify/--inspect-bootstrap may spawn."""
import copy,csv,hashlib,io,json,os,pathlib,stat,subprocess,time
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
W=S/'workspace';A=pathlib.Path(__file__).resolve().parent
RELEASE=S/'contracts/navigation_n06_exact_release_r01.json'
SHA='3846af40b95b72ac61dd87e2bf3da9a411732992b5c96c196f54634197cccf9a'
READ={};checks=[]
def read(p):
    p=pathlib.Path(p);assert p.is_absolute() and p.resolve()==p
    with p.open('rb') as f:
        before=os.fstat(f.fileno());assert stat.S_ISREG(before.st_mode)
        h=hashlib.sha256();chunks=[]
        for c in iter(lambda:f.read(1024*1024),b''):h.update(c);chunks.append(c)
        after=os.fstat(f.fileno())
    key=lambda st:(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns,st.st_ctime_ns)
    assert key(before)==key(after)==key(p.stat())
    READ[str(p)]={'path':str(p),'sha256':h.hexdigest()};return b''.join(chunks)
def binding(p):read(p);return READ[str(p)]
def bound(b):
    data=read(b['path']);assert READ[b['path']]==b,b;return data
def js(p):return json.loads(read(p))
def check(n,c,detail=None):checks.append({'name':n,'pass':bool(c),'detail':detail});assert c,(n,detail)
def write(n,v):
    p=A/n;assert not p.exists();p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')
def snapshot(phase,units):
    rows={}
    for name in ('seed_registry.csv','EXPERIMENTS.csv','run_manifest.csv'):
        p=S/name;data=p.read_bytes();target=A/(phase+'_'+name);assert not target.exists();target.write_bytes(data)
        rows[name]={'sha256':hashlib.sha256(data).hexdigest(),'snapshot':str(target),'size_bytes':len(data)}
    paths=[S/'queue/navigation_n06_ledger.json',S/'queue/navigation_n06_events',S/'queue/navigation_n06_manager_return.json']+[S/'runs'/u['run_id'] for u in units]
    return {'administrative_files':rows,'identity_paths':[{'path':str(p),'exists':p.exists() or p.is_symlink()} for p in paths]}

try:
    release=js(RELEASE);check('exact_release_SHA',READ[str(RELEASE)]['sha256']==SHA)
    resources={}
    for item in release['resources']:
        bound(item);assert item['path'] not in resources;resources[item['path']]=item
    check('5688_unique_resource_hashes_all_match',len(resources)==5688)
    for key,item in release.items():
        if isinstance(item,dict) and set(item)=={'path','sha256'}:bound(item);assert resources[item['path']]==item
    check('authority_exact_bounded_and_no_retry_training_vehicle', release['execution_authorized'] is True and release['maximum_episodes']==8 and type(release['maximum_episodes']) is int and release['training_authorized'] is False and release['real_vehicle_authorized'] is False and release['resume_allowed'] is False and release['retry_allowed'] is False)
    review=json.loads(bound(release['source_review']));runtime=json.loads(bound(release['runtime_closure']))
    check('exact_source_review_and_runtime_lineage',release['source_review']['sha256']=='d9ab161799d0ef03a82839b3c8134407c23a4dbd3b5b7c2d3792d0f7a21ea6c6' and release['runtime_closure']['sha256']=='48986889780c681a87cacc20cf53d22e01e967096b02354732e43fc8a3c50c36')
    roster=json.loads(bound(review['candidate_roster']))
    assert review['candidate_roster']['sha256']=='d0ebfdfc8e1f4ccf1fbd107c9dfb28ad3dbe58ca398a5c79a21933952b0b3471'
    for item in roster['resources']:assert resources[item['path']]==item
    reviewed={r['path']:r for r in review['resources']}
    source_ext={'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}
    missing=[p for p,r in resources.items() if pathlib.Path(p).suffix in source_ext and reviewed.get(p)!=r]
    check('r04_5669_candidates_and_all_release_source_configs_reviewed',len(roster['resources'])==5669 and not missing,{'release_extra_paths':sorted(set(resources)-{r['path'] for r in roster['resources']})})
    for item in runtime['resources']:assert resources[item['path']]==item
    check('reviewed_disk_roster_5217_preserved',len(runtime['resources'])==5217 and runtime['status']=='reviewed_disk_dependency_closure' and runtime['independent_review'] is True and runtime['runtime_ready'] is False)
    negative=runtime['prior_independent_negative_review'];check('old_export_negative_bound_and_preserved',negative['sha256']=='5fc6cb26f7ffe3ca90d257e7e2372ebb371b1dee750e1e870c3e3d07449b21cf' and resources[negative['path']]==negative)
    limits=runtime['future_live_check_boundary'];gazeborc=pathlib.Path(limits['gazeborc_path'])
    check('GAZEBORC_current_absence_limit_not_promoted',not gazeborc.exists() and not gazeborc.is_symlink() and limits['gazeborc_currently_absent'] is True and limits['gazeborc_immutable_absence_guarantee'] is False and limits['current_manager_dedicated_gazeborc_absence_gate'] is False and limits['native_SystemPaths_plugin_executed'] is False and limits['live_mapped_elf_observed'] is False)
    schedule=json.loads(bound(release['schedule']));prepared=json.loads(bound(release['prepared']));units=prepared['units'];expected=[]
    for block in range(4):
        order=('control','treatment') if block%2==0 else ('treatment','control')
        for pair,arm in enumerate(order,1):
            expected.append({'arm':arm,'attempt':1,'block_index':block,'layout_id':'d04r05_development_static_avoidance_%03d'%block,'lookahead_m':0.7 if arm=='control' else 1.2,'method':'astra_analytic_route_v1' if arm=='control' else 'astra_analytic_lookahead_1p2_n06','pair_position':pair,'run_id':'astra_navigation_n06_b%03d_%s'%(block,arm),'scene_id':'navigation_n06_static_b%03d'%block,'seed':263003001+block,'seed_status':'proposal_unallocated','sequence':len(expected)+1})
    assert schedule['schedule']==expected and len(units)==8
    for u,r in zip(units,expected):assert all(type(u[k]) is type(v) and u[k]==v for k,v in r.items())
    check('exact_eight_counterbalanced_four_seed_pairs',True,expected)
    before=snapshot('before',units);assert not any(r['exists'] for r in before['identity_paths'])
    live=(S/'seed_registry.csv').read_bytes();baseline=bound(release['registry_baseline'])
    registry=list(csv.DictReader(io.StringIO(live.decode())))
    ids={u['run_id'] for u in units};seeds={str(u['seed']) for u in units}
    check('live_registry_equals_frozen_baseline_zero_allocations',live==baseline and not any(r.get('run_id') in ids or r.get('seed') in seeds for r in registry))
    predecessor=json.loads(bound(release['predecessor_ledger']));check('N05_predecessor_terminal',predecessor['status']=='terminal' and len(predecessor['units'])==24)
    qualification=json.loads(bound(release['passive_geometry_qualification']))
    projection=js(S/'audits/navigation_n06_candidate_roster_preparation_r02/qualification_projection.json');original=json.loads(bound(projection['original']))
    assert projection['projected']==release['passive_geometry_qualification']
    reverted=copy.deepcopy(qualification);reverted['evidence_bindings'][0]['path']=original['evidence_bindings'][0]['path']
    check('Q05_projection_changes_only_one_path_no_qualification_extension',reverted==original and qualification['evidence_bindings'][0]['sha256']==original['evidence_bindings'][0]['sha256'] and qualification['qualified_for_navigation_evidence'] is True and qualification['formal_safety'] is False and qualification['real_continuous_physical_safety'] is False and qualification['navigation_performance_evidence'] is False)
    for b in qualification['evidence_bindings']:assert resources[b['path']]==b
    argv=['/usr/bin/env','-i','HOME=/home/robot','USER=robot','LOGNAME=robot','LANG=C.UTF-8','PATH=/usr/bin:/bin','/bin/bash','--noprofile','--norc',str(W/'scripts/astra_gazebo/navigation_n06_entry_r05.sh'),'--verify','--release',str(RELEASE),'--release-sha256',SHA]
    results={}
    for mode in ('inspect','verify'):
        command=argv+(['--inspect-bootstrap'] if mode=='inspect' else [])
        assert '--execute' not in command and '--worker' not in command
        started=time.monotonic();p=subprocess.run(command,env={'PATH':'/usr/bin:/bin'},cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=180)
        (A/(mode+'.stdout')).write_bytes(p.stdout);(A/(mode+'.stderr')).write_bytes(p.stderr)
        row={'argv':command,'parent_environment':{'PATH':'/usr/bin:/bin'},'returncode':p.returncode,'wall_s':time.monotonic()-started,'stdout':binding(A/(mode+'.stdout')),'stderr':binding(A/(mode+'.stderr'))};write(mode+'_command.json',row)
        assert p.returncode==0,(mode,p.stderr.decode()[-1000:]);results[mode]=json.loads(p.stdout)
        if mode=='inspect':
            x=results[mode];assert x['manager_imported'] is False and x['execution_performed'] is False
            assert x['environment']==dict(runtime['execution_environment']['values'],ASTRA_N06_RUNTIME_CLOSURE_SHA256=release['runtime_closure']['sha256'])
            assert x['argv']==[str(W/'.venv/bin/python'),str(W/'scripts/astra_gazebo/run_navigation_n06_manager_r05.py'),'--verify','--release',str(RELEASE),'--release-sha256',SHA]
        else:assert results[mode]=={'status':'release_bytes_verified_readonly','units':8,'allocation_performed':False}
    check('real_env_i_inspect_and_full_verify_exit_zero',True,results)
    after=snapshot('after',units)
    check('verify_has_no_admin_or_identity_side_effects',all(before['administrative_files'][k]['sha256']==after['administrative_files'][k]['sha256'] for k in before['administrative_files']) and before['identity_paths']==after['identity_paths'])
    for r in release['resources']:bound(r)
    check('post_verify_release_and_all_resources_unchanged',binding(RELEASE)['sha256']==SHA)
    write('results.json',{'status':'independent_exact_release_readonly_preexecution_pass','checks':checks,'before':before,'after':after,'release':binding(RELEASE),'resources_checked':5688,'read_bindings':sorted(READ.values(),key=lambda r:r['path']),'execution_performed':False,'seed_registration_performed':False})
    print(json.dumps({'pass':True,'checks':len(checks),'resources':5688,'units':8,'allocation_performed':False}))
except BaseException as exc:
    write('failure.json',{'status':'failed_stopped_readonly','error':type(exc).__name__+':'+str(exc),'checks':checks,'execution_performed':False})
    raise
