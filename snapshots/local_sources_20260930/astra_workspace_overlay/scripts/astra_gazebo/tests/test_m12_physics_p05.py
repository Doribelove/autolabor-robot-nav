"""Offline P05 identity, attribution, closure, and terminal-budget regressions."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location('p05_runner',SCRIPTS/'run_m12_physics_p05.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


@pytest.fixture
def design_rows():
    design,parent,rows=m.load_design()
    return copy.deepcopy(design),copy.deepcopy(parent),copy.deepcopy(rows)


def passing_report(design):
    names=set(design['prerequisite']['full_check_roster'])
    return {'pass':True,'assessment_incomplete':False,'checks':dict.fromkeys(names,True),'metrics':{}}


def receipt(row,outcome='qualified'):
    report={'metrics':{'actual_center_rate_radps':{'maximum_absolute':.9 if row['arm']=='control' else .8}}}
    return dict(row,outcome=outcome,pass_all_gates=outcome=='qualified',physical_failed_checks=[] if outcome=='qualified' else ['actual_center_slew_with_declared_sampling_tolerance'],report=report)


def test_exact_six_pairs_and_contract_nonfactor_equivalence(design_rows):
    design,parent,rows=design_rows
    assert [(r['seed'],r['steering_kd']) for r in rows]==[(260926017,4),(260926017,6),(260926018,6),(260926018,4),(260926019,4),(260926019,6)]
    assert len({r['run_id'] for r in rows})==6
    for row in rows:
        contract=yaml.safe_load(m.read_bound(m.STAGE/row['contract_path'],row['contract_sha256']))
        assert contract['maximum_episodes']==1
        assert contract['world_fixture']['sdf']==parent['world_fixture']['sdf']
        for key in parent['effective_plugin_parameters']:
            assert contract['effective_plugin_parameters'][key]==(row['steering_kd'] if key=='steeringKd' else parent['effective_plugin_parameters'][key])


@pytest.mark.parametrize('index',range(6))
def test_launch_uses_exact_p02_driver_and_probe_with_explicit_factor(index,design_rows,tmp_path):
    _,_,rows=design_rows;row=rows[index]
    contract=yaml.safe_load((m.STAGE/row['contract_path']).read_bytes())
    tree=ET.fromstring(m.render_launch(row,tmp_path,contract))
    assert tree.find('include').get('file').endswith('/spawn_m2_physical_p05.launch')
    args={n.get('name'):n.get('value') for n in tree.findall('include/arg')}
    assert int(args['seed'])==row['seed']
    assert float(args['steering_kd'])==row['steering_kd']
    for arg,key in m.ARG_MAP.items():assert float(args[arg])==contract['effective_plugin_parameters'][key]
    nodes={n.get('name'):n for n in tree.findall('node')}
    assert nodes['actuation_command_source'].get('type')=='astra_actuation_command_source_p05.py'
    assert nodes['fixture_probe'].get('type')=='astra_physical_actuation_probe_p05.py'
    assert set(yaml.safe_load(nodes['fixture_probe'].find('rosparam').text))==set(m.TOPICS)
    assert 'metadata_p05' not in ET.tostring(tree,encoding='unicode')


def test_command_source_has_only_stage_guard_change(design_rows):
    design,_,_=design_rows
    original=m.read_bound(m.STAGE/design['command_source_parent']['path'],design['command_source_parent']['sha256']).decode()
    assert m.SOURCE.read_text()==original.replace("'astra_m12_physics_p02'","'astra_m12_physics_p05'")


@pytest.mark.parametrize('field,value',[('arm','treatment'),('pair',2),('pair_position',2)])
def test_rebound_schedule_cannot_relabel_factor_or_pairs(field,value,design_rows,monkeypatch):
    design,_,rows=design_rows;rows[0][field]=value
    original=m.read_bound
    def rebound(path,digest):
        if Path(path)==m.STAGE/design['schedule']['path']:return json.dumps(rows).encode()
        return original(path,digest)
    monkeypatch.setattr(m,'read_bound',rebound)
    with pytest.raises(ValueError,match='arm/pair label'):m.load_design()


@pytest.mark.parametrize('failed',[
    None,'actual_center_slew_with_declared_sampling_tolerance','internal_accel_limit',
    'no_non_ground_robot_contacts','commanded_brake_actual_stopped'])
def test_physical_negative_is_preserved_without_infrastructure_stop(failed,design_rows):
    design,_,_=design_rows;report=passing_report(design)
    if failed:report['checks'][failed]=False;report['pass']=False
    outcome,failures=m.classify_report(report,design,list(report['checks']))
    assert outcome==('qualified' if failed is None else 'physical_negative')
    assert failures==([] if failed is None else [failed])


@pytest.mark.parametrize('mutation',[
    'measurement_false','measurement_missing','unknown_false','incomplete','exception',
    'nonbool','overall_mismatch','missing_report','changed_roster'])
def test_measurement_or_identity_roster_defect_stops_pilot(mutation,design_rows):
    design,_,_=design_rows;report=passing_report(design);expected=list(report['checks'])
    key=design['prerequisite']['minimum_measurement_checks'][0]
    if mutation=='measurement_false':report['checks'][key]=False;report['pass']=False
    if mutation=='measurement_missing':report['checks'].pop(key)
    if mutation=='unknown_false':report['checks']['unrecognized_gate']=False;report['pass']=False
    if mutation=='incomplete':report['assessment_incomplete']=True
    if mutation=='exception':report['assessment_exception']='error'
    if mutation=='nonbool':report['checks'][key]=1
    if mutation=='overall_mismatch':report['pass']=False
    if mutation=='missing_report':report=None
    if mutation=='changed_roster':report['checks']['new_true_gate']=True
    assert m.classify_report(report,design,expected)[0]=='infrastructure_failure'


def test_observed_p02_is_only_frozen_protocol_prerequisite(design_rows):
    design,_,_=design_rows
    prior=m.p02_protocol(design)
    assert len(prior['check_names'])==120 and prior['status']=='physical_negative'
    assert prior['physical_failed_checks']==['actual_center_slew_with_declared_sampling_tolerance']
    # Old P02 is not a complete new P05 report without the watermark gate.
    report=json.loads((m.STAGE/'runs/m12_physics_p02/probe_report.json').read_bytes())
    assert m.classify_report(report,design)[0]=='infrastructure_failure'


@pytest.mark.parametrize('case', ['all_treatment_pass','one_treatment_negative','infrastructure_report_none','incomplete'])
def test_summary_retains_controls_and_requires_three_complete_treatments(case,design_rows):
    _,_,rows=design_rows
    receipts=[receipt(r,'physical_negative' if r['arm']=='control' else 'qualified') for r in rows]
    if case=='one_treatment_negative':receipts[1]=receipt(rows[1],'physical_negative')
    if case=='infrastructure_report_none':receipts[1]=receipt(rows[1],'infrastructure_failure');receipts[1]['report']=None
    if case=='incomplete':receipts=receipts[:3]
    summary=m.matrix_summary(rows,receipts,{})
    assert summary['treatment_promoted']==(case=='all_treatment_pass')
    assert 'control' in summary['pairs'][0]['results']
    if case=='all_treatment_pass':
        assert len(summary['pairs'])==3
        assert summary['pairs'][0]['treatment_minus_control_peak_center_rate_radps']==pytest.approx(-.1)


@pytest.fixture
def isolated_runtime(tmp_path,monkeypatch,design_rows):
    design,parent,rows=design_rows;stage=tmp_path/'stage';workspace=stage/'workspace'
    for name in ('queue','runs','audits','code_blobs','workspace'):(stage/name).mkdir(parents=True,exist_ok=True)
    (stage/'seed_registry.csv').write_text('namespace,split,purpose,seed,run_id,status,source_binding\n')
    for name in ('run_manifest.csv','EXPERIMENTS.csv'):(stage/name).write_text('run_id,status,outcome,error,receipt_path\n')
    monkeypatch.setattr(m,'STAGE',stage);monkeypatch.setattr(m,'WORKSPACE',workspace)
    monkeypatch.setattr(m,'RUNTIME_LEDGER',stage/'queue/budget_runtime.json')
    monkeypatch.setenv('THESIS_WS',str(workspace))
    monkeypatch.setattr(m,'load_design',lambda:(design,parent,rows))
    monkeypatch.setattr(m,'verify_release',lambda *a:({}, {'check_names':['sentinel']}))
    def forbid(*a,**kw):raise AssertionError('offline test attempted external process')
    monkeypatch.setattr(m.subprocess,'Popen',forbid);monkeypatch.setattr(m.subprocess,'run',forbid)
    return stage,workspace,design,rows


@pytest.mark.parametrize('failure_index',[0,2,5])
def test_infrastructure_failure_consumes_once_and_retires_remainder(failure_index,isolated_runtime,monkeypatch):
    stage,_,_,rows=isolated_runtime;calls=[]
    def run(row,*args):
        calls.append(row['run_id']);m.append_registry(row,'consumed')
        return receipt(row,'infrastructure_failure' if row['sequence']==failure_index+1 else 'qualified')
    monkeypatch.setattr(m,'execute_row',run)
    assert m.execute(stage/'release.json','fake')==3
    state=json.loads(m.RUNTIME_LEDGER.read_bytes())
    assert state['consumed_units']==failure_index+1
    assert state['retired_units']==5-failure_index
    assert len(calls)==failure_index+1
    assert state['status']=='stopped_infrastructure'
    with (stage/'seed_registry.csv').open() as f:registered=list(csv.DictReader(f))
    assert len(registered)==6 and len({r['run_id'] for r in registered})==6
    with pytest.raises(ValueError,match='consumed|restarted'):m.execute(stage/'release.json','fake')


def test_control_physical_failures_do_not_abort_any_pair(isolated_runtime,monkeypatch):
    stage,_,_,_=isolated_runtime;calls=[]
    def run(row,*args):
        calls.append(row['run_id']);m.append_registry(row,'consumed')
        return receipt(row,'physical_negative' if row['arm']=='control' else 'qualified')
    monkeypatch.setattr(m,'execute_row',run)
    assert m.execute(stage/'release.json','fake')==0
    state=json.loads(m.RUNTIME_LEDGER.read_bytes())
    assert len(calls)==6 and state['consumed_units']==6 and state['retired_units']==0
    assert state['status']=='terminal_complete'


def test_preparation_exception_keeps_failed_identity_and_retires_budget(isolated_runtime,monkeypatch):
    stage,_,_,rows=isolated_runtime
    def run(row,*args):
        if row['sequence']==2:raise RuntimeError('synthetic prelaunch failure')
        m.append_registry(row,'consumed');return receipt(row)
    monkeypatch.setattr(m,'execute_row',run)
    assert m.execute(stage/'release.json','fake')==3
    state=json.loads(m.RUNTIME_LEDGER.read_bytes())
    assert state['consumed_units']==2 and state['retired_units']==4
    failed=json.loads((stage/'runs'/rows[1]['run_id']/'receipt.json').read_bytes())
    assert failed['outcome']=='infrastructure_failure' and failed['retry_allowed'] is False
    with (stage/'seed_registry.csv').open() as f:registered=list(csv.DictReader(f))
    assert len(registered)==6 and len({r['run_id'] for r in registered})==6


def test_bind_archives_only_release_roster_and_row_inputs(isolated_runtime,monkeypatch):
    stage,workspace,design,rows=isolated_runtime
    root=stage/'runs'/rows[0]['run_id'];root.mkdir()
    source=workspace/'active.py';source.write_text('active original\n')
    future=workspace/'unrelated_future.py';future.write_text('must remain outside closure\n')
    for name in ('world.sdf','scene_manifest.yaml','fixture.launch','actuation_contract.yaml'):(root/name).write_text(name)
    release_path=stage/'release.json';release_path.write_text('{}')
    release={'resources':[{'path':str(source),'sha256':m.common.sha(source)}]}
    monkeypatch.setattr(m,'verify_release',lambda *a:(release,{}))
    commands=[]
    def diff(command,**kwargs):
        commands.append(command);kwargs['stdout'].write(b'test patch\n')
    monkeypatch.setattr(m.subprocess,'run',diff)
    digest=m.bind_from_release(root,release_path,m.common.sha(release_path),design,rows)
    bound=json.loads((root/'bindings.json').read_bytes())
    assert m.common.sha(root/'bindings.json')==digest
    paths={r['path'] for r in bound['resources']}
    assert str(future) not in paths and len(paths)==6
    assert commands==[['git','diff','--binary','--','active.py']]
    for item in bound['resources']:assert (stage/'code_blobs'/item['sha256']).read_bytes()==Path(item['path']).read_bytes()
    source.write_text('changed active input\n')
    with pytest.raises(ValueError,match='active dependency changed'):m.bind_from_release(root,release_path,m.common.sha(release_path),design,rows)


def test_release_rejects_tampering_and_missing_dependencies(tmp_path,monkeypatch):
    source=tmp_path/'actual.py';source.write_text('frozen')
    parent=tmp_path/'parent_release.json';parent.write_text(json.dumps({'resources':[{'path':str(source),'sha256':m.common.sha(source)}]}))
    design={'stage':'astra_m12_physics_p05','runtime_resource_parent':{'path':str(parent),'sha256':m.common.sha(parent)}}
    prior={'status':'physical_negative','check_names':['all']}
    monkeypatch.setattr(m,'required_resources',lambda *a:[source])
    monkeypatch.setattr(m,'predecessor',lambda *a:prior)
    release={'stage':'astra_m12_physics_p05','status':'root_reviewed_release','execution_authorized':True,'maximum_episodes':6,
             'resources':[{'path':str(source),'sha256':m.common.sha(source)}],'predecessor_measurement_qualified':True,'predecessor':prior}
    path=tmp_path/'release.json';path.write_text(json.dumps(release));digest=m.common.sha(path)
    assert m.verify_release(path,digest,design,[])[1]==prior
    source.write_text('tampered')
    with pytest.raises(ValueError,match='changed bound input'):m.verify_release(path,digest,design,[])
    release['resources']=[];path.write_text(json.dumps(release))
    with pytest.raises(ValueError,match='incomplete'):m.verify_release(path,m.common.sha(path),design,[])


@pytest.mark.parametrize('mutation',['missing','pending','not_independent','wrong_probe','changed_gates','count120','missing_watermark','duplicate_name','valid'])
def test_watermark_repair_requires_real_matching_independent_proof(mutation,tmp_path,monkeypatch,design_rows):
    design,_,_=design_rows;probe=tmp_path/'probe_p05.py';probe.write_text('fixture only')
    review_path=tmp_path/'independent_review.json';design['prerequisite']['ingress_repair_review']['path']=str(review_path)
    monkeypatch.setattr(m,'PROBE',probe)
    driver_review=tmp_path/'driver_review.json';design['prerequisite']['driver_root_review']['path']=str(driver_review)
    driver_review.write_text(json.dumps({'pass':True,'independent_driver_review':True,'controller_formula_unchanged':True,'driver_sha256':m.common.sha(m.WORKSPACE/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp')}))
    review={'status':'offline_driver_ingress_measurement_review_pass','independent_probe_review':True,'reviewed_probe_sha256':m.common.sha(probe),
            'numeric_thresholds_unchanged':True,'total_gate_count':122,'check_names':list(design['prerequisite']['full_check_roster'])}
    if mutation=='pending':review['status']='pending'
    if mutation=='not_independent':review['independent_probe_review']=False
    if mutation=='wrong_probe':review['reviewed_probe_sha256']='0'*64
    if mutation=='changed_gates':review['numeric_thresholds_unchanged']=False
    if mutation=='count120':review['total_gate_count']=120
    if mutation=='missing_watermark':review['check_names'].remove('final_paused_source_watermark_drained')
    if mutation=='duplicate_name':review['check_names'].append(review['check_names'][0])
    if mutation!='missing':review_path.write_text(json.dumps(review))
    if mutation=='valid':assert m.ingress_review(design)['status']=='offline_driver_ingress_measurement_review_pass'
    else:
        with pytest.raises((ValueError,FileNotFoundError)):m.ingress_review(design)


def test_watermark_negative_is_infrastructure_failure_not_a_physical_negative(design_rows):
    design,_,_=design_rows;report=passing_report(design)
    report['checks']['final_paused_source_watermark_drained']=False;report['pass']=False
    outcome,failed=m.classify_report(report,design)
    assert outcome=='infrastructure_failure' and failed==['final_paused_source_watermark_drained']


@pytest.mark.parametrize('mutation',['counts','reordered_retirement','promoted','diagnosis_missing','replay_false'])
def test_p03_terminal_result_cannot_be_resumed_or_reinterpreted(mutation,monkeypatch,design_rows):
    design,_,_=design_rows;requirements=design['prerequisite']['P03_terminal']
    docs={key:json.loads((m.STAGE/requirements[key]['path']).read_bytes()) for key in ('ledger','summary','diagnosis')}
    docs['diagnosis']['inputs']=[]  # synthetic mutation fixture, no large historical raw reads
    if mutation=='counts':docs['ledger']['retired_units']=0
    if mutation=='reordered_retirement':docs['ledger']['units'][-1]['budget_status']='consumed'
    if mutation=='promoted':docs['summary']['treatment_promoted']=True
    if mutation=='diagnosis_missing':docs['diagnosis']['runs'].pop('m12_physics_p03_s009_kd6')
    if mutation=='replay_false':docs['diagnosis']['runs']['m12_physics_p03_s009_kd4']['original_replay_exact']=False
    rebound={str(m.STAGE/requirements[key]['path']):json.dumps(value).encode() for key,value in docs.items()}
    monkeypatch.setattr(m,'read_bound',lambda path,digest:rebound[str(path)])
    with pytest.raises(ValueError):m.p03_terminal(design)


def test_p05_protocol_exactly_inherits_numeric_gates_and_five_second_budget(design_rows):
    design,parent,rows=design_rows
    assert len(design['prerequisite']['inherited_check_names'])==120
    assert set(design['prerequisite']['full_check_roster'])-set(design['prerequisite']['inherited_check_names'])=={'final_paused_source_watermark_drained','physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain'}
    assert design['terminal_collection_protocol']['post_pause_drain_wall_s']==parent['recorder_protocol']['post_pause_drain_wall_s']==5.
    for row in rows:
        contract=yaml.safe_load((m.STAGE/row['contract_path']).read_bytes())
        assert contract['gates']==parent['gates']
        assert contract['phase_schedule']==parent['phase_schedule']
        assert contract['terminal_collection_protocol']==design['terminal_collection_protocol']


def test_existing_source_runtime_closure_cannot_be_rebound_to_new_bytes(tmp_path,monkeypatch):
    source=tmp_path/'shared_runtime.so';source.write_text('original')
    old_digest=m.common.sha(source)
    parent=tmp_path/'p03_release.json';parent.write_text(json.dumps({'resources':[{'path':str(source),'sha256':old_digest}]}))
    design={'stage':'astra_m12_physics_p05','runtime_resource_parent':{'path':str(parent),'sha256':m.common.sha(parent)}}
    source.write_text('silently changed')
    prior={'check_names':['fixture']};monkeypatch.setattr(m,'required_resources',lambda *a:[source]);monkeypatch.setattr(m,'predecessor',lambda *a:prior)
    release={'stage':'astra_m12_physics_p05','status':'root_reviewed_release','execution_authorized':True,'maximum_episodes':6,
             'resources':[{'path':str(source),'sha256':m.common.sha(source)}],'predecessor_measurement_qualified':True,'predecessor':prior}
    path=tmp_path/'release.json';path.write_text(json.dumps(release))
    with pytest.raises(ValueError,match='frozen source/runtime closure changed'):m.verify_release(path,m.common.sha(path),design,[])


def test_inherited_roster_is_explicit_and_excludes_future_route_modules(design_rows):
    design,_,rows=design_rows;paths=m.required_resources(design,rows)
    assert len(paths)>=420
    assert not any(p.name in ('common_route.py','astra_common_route_provider.py') for p in paths)
    assert m.PROBE.resolve() in paths and m.SOURCE.resolve() in paths
    assert (m.STAGE/design['prerequisite']['ingress_repair_review']['path']).resolve() in paths
