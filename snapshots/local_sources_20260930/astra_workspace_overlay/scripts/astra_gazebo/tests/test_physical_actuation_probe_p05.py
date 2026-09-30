"""Terminal capture fault tests; synthetic packets never become episode evidence."""
import copy
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'probe_p05', ROOT/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p05.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


@pytest.fixture
def fixture():
    raw = {'schedule_start_ns': 100000000}
    periods = {key: (2000000, 20000000) for key in ('models', 'links')}
    periods.update({key: (20000000, 20000000) for key in (
        'odom', 'joints', 'targets', 'backend_status', 'commanded_efforts')})
    periods.update({key: (2000000, 2000000) for key in (
        'dense_models', 'dense_links', 'contacts')})
    for key, (first, period) in periods.items():
        raw[key] = [{'ns': ns} for ns in range(first, 144000001, period)]
    contract = {'world_fixture': {'max_step_s': .002, 'fixed_world_models': ['ground_plane']},
                'model_name': 'm2', 'total_schedule_s': .04}
    physics = {'success': True, 'pause': True, 'time_step': .002, 'status_message': ''}
    world = {'success': True, 'sim_time': .144, 'model_names': ['ground_plane', 'm2'], 'status_message': ''}
    raw['final_drain'] = dict(m.final_stream_watermarks(raw, 144000000),
        physics=physics, world_queries=[copy.deepcopy(world), copy.deepcopy(world)],
        source_watermark_ns=144000000, stable_world_query_ns=[144000000, 144000000], wall_elapsed_s=.1)
    return raw, contract


def test_exact_different_publication_phases_are_required(fixture):
    raw, contract = fixture
    assert m.final_drain_integrity(raw, contract)
    tail = m.final_stream_watermarks(raw, 144000000)
    assert tail['required_last_ns']['models'] == 142000000
    assert tail['required_last_ns']['odom'] == 140000000
    assert tail['required_last_ns']['dense_links'] == 144000000


@pytest.mark.parametrize('stream', ['models', 'links', 'odom', 'joints', 'targets',
    'backend_status', 'commanded_efforts', 'dense_models', 'dense_links', 'contacts'])
def test_each_missing_tail_fails_even_when_formal_window_complete(stream, fixture):
    raw, contract = fixture
    raw[stream].pop()
    assert not m.final_drain_integrity(raw, contract)


@pytest.mark.parametrize('mutation', ['interior_loss', 'duplicate', 'off_grid', 'future', 'boolean'])
def test_dense_stream_count_endpoints_cannot_hide_missing_or_invalid_slot(mutation, fixture):
    raw, contract = fixture
    rows = raw['dense_links']
    if mutation == 'interior_loss': rows.pop(10)
    if mutation == 'duplicate': rows[10] = rows[9].copy()
    if mutation == 'off_grid': rows[10]['ns'] += 1
    if mutation == 'future': rows.append({'ns': 146000000})
    if mutation == 'boolean': rows[10]['ns'] = True
    assert not m.final_drain_integrity(raw, contract)


@pytest.mark.parametrize('mutation', ['unpaused', 'physics_failure', 'world_failure', 'changed_step',
    'missing_model', 'duplicate_model', 'nonfinite', 'boolean_time', 'offgrid_time', 'before_end',
    'changed_second_time', 'query_absent', 'pause_error', 'timeout', 'declared_tail_changed', 'absent'])
def test_invalid_service_or_closure_record_fails_closed(mutation, fixture):
    raw, contract = fixture
    drain = raw['final_drain']; physics = drain['physics']; world = drain['world_queries'][1]
    if mutation == 'unpaused': physics['pause'] = False
    if mutation == 'physics_failure': physics['success'] = False
    if mutation == 'world_failure': world['success'] = False
    if mutation == 'changed_step': physics['time_step'] = .001
    if mutation == 'missing_model': world['model_names'] = ['m2']
    if mutation == 'duplicate_model': world['model_names'].append('m2')
    if mutation == 'nonfinite': world['sim_time'] = float('nan')
    if mutation == 'boolean_time': world['sim_time'] = True
    if mutation == 'offgrid_time': world['sim_time'] = .143
    if mutation == 'before_end': world['sim_time'] = .138
    if mutation == 'changed_second_time': world['sim_time'] = .146
    if mutation == 'query_absent': drain['world_queries'].pop()
    if mutation == 'pause_error': raw['pause_error'] = 'timeout'
    if mutation == 'timeout': drain['wall_elapsed_s'] = 5.001
    if mutation == 'declared_tail_changed': drain['observed_last_ns']['dense_links'] = 140000000
    if mutation == 'absent': raw.pop('final_drain')
    assert not m.final_drain_integrity(raw, contract)


def test_actual_double_representation_on_grid_accepted(fixture):
    raw, contract = fixture
    world = raw['final_drain']['world_queries'][0]
    world['sim_time'] = 37.144000000000005
    assert m.paused_world_source_ns(raw['final_drain']['physics'], world, contract, 37000000000) == 37144000000


def ingress_row(ns,seq,v,w):
    values={'input_topic':'cmd_vel','requested_linear_x':str(v),'requested_angular_z':str(w),
            'accepted':'true','disposition':'accepted_queued','backend_fault':'false','activation_time_s':str(ns/1e9)}
    return {'ns':ns,'t':ns/1e9,'seq':seq,'frame':'world','statuses':[{
        'name':'astra_physical_command_ingress','hardware_id':'simulation-only','level':0,
        'message':'accepted_queued','value_count':7,'values':values}]}


@pytest.fixture
def ingress_fixture():
    raw={'schedule_start_ns':1108000000,'commands_sent':[
        {'ns':1108000000,'v':1.2,'w':.1},{'ns':1158000000,'v':1.2,'w':.1},
        {'ns':37108000000,'v':0.,'w':0.}],
        'command_ingress':[ingress_row(1110000000,11,1.2,.1),ingress_row(1160000000,12,1.2,.1),ingress_row(37110000000,13,0.,0.)]}
    raw['final_drain']={'source_watermark_ns':37144000000}
    return raw,{'total_schedule_s':36,'world_fixture':{'max_step_s':.002}}


def test_ordered_source_ingress_contract(ingress_fixture):
    raw,contract=ingress_fixture
    assert all(m.command_ingress_integrity(raw,contract).values())


@pytest.mark.parametrize('mutation',['missing','duplicate','sequence_gap','payload','early','clock_regress',
    'wrong_topic','rejected','fault','wrong_frame','nan','bad_schema','second_terminal'])
def test_driver_ingress_faults_fail_closed(mutation,ingress_fixture):
    raw,contract=ingress_fixture;rows=raw['command_ingress'];values=rows[1]['statuses'][0]['values']
    if mutation=='missing':rows.pop(1)
    if mutation=='duplicate':rows.insert(1,copy.deepcopy(rows[0]))
    if mutation=='sequence_gap':rows[1]['seq']+=10
    if mutation=='payload':values['requested_linear_x']='.5'
    if mutation=='early':rows[0]['ns']=1107000000
    if mutation=='clock_regress':rows[1]['ns']=rows[0]['ns']-1
    if mutation=='wrong_topic':values['input_topic']='ackerman_vel'
    if mutation=='rejected':values['accepted']='false'
    if mutation=='fault':values['backend_fault']='true'
    if mutation=='wrong_frame':rows[1]['frame']='odom'
    if mutation=='nan':values['requested_linear_x']='nan'
    if mutation=='bad_schema':values.pop('activation_time_s')
    if mutation=='second_terminal':rows.append(ingress_row(37120000000,14,0.,0.))
    assert not all(m.command_ingress_integrity(raw,contract).values())


def test_terminal_zero_uses_driver_source_time_not_lagging_probe_clock(ingress_fixture):
    raw,contract=ingress_fixture;raw['commands_observed']=[{'t':37.106,'v':0.,'w':0.}]
    lower=raw['schedule_start_ns']+32000000000+120000000;upper=raw['schedule_start_ns']+36000000000
    assert m.in_ns_window(raw['commands_observed'][0],lower,upper)
    assert not m.in_ns_window(m.parse_command_ingress(raw)[-1],lower,upper)
    assert all(m.command_ingress_integrity(raw,contract).values())


@pytest.mark.parametrize('kind',['future_watermark','off_physics_grid'])
def test_ingress_cannot_claim_impossible_physics_epoch(kind,ingress_fixture):
    raw,contract=ingress_fixture
    ns=raw['final_drain']['source_watermark_ns']+2000000 if kind=='future_watermark' else 37110000001
    raw['command_ingress'][-1]=ingress_row(ns,13,0.,0.)
    assert not m.command_ingress_integrity(raw,contract)['physical_command_ingress_source_schema_and_order']


def test_protected_callback_ignores_errors_after_atomic_close():
    import ast,threading,traceback
    tree=ast.parse(Path(m.__file__).read_text())
    runtime=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='runtime')
    function=next(n for n in runtime.body if isinstance(n,ast.FunctionDef) and n.name=='protected')
    namespace={'lock':threading.RLock(),'active':False,'raw':{'callback_errors':[]},'traceback':traceback}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),'actual_protected','exec'),namespace)
    invoked=[]
    def error(_):invoked.append(True);raise ValueError('synthetic decoder failure')
    callback=namespace['protected'](error)
    callback(None)
    assert not invoked and not namespace['raw']['callback_errors']
    namespace['active']=True;callback(None)
    assert len(invoked)==len(namespace['raw']['callback_errors'])==1


def test_integer_phase_intervals_match_exact_arithmetic_at_all_2ms_starts():
    for start in range(1000000000,1200000001,2000000):
        for phase_ns in (0,2000000000,6000000000,10000000000,18000000000,32000000000,36000000000):
            end=start+phase_ns;begin=end-750000000
            for ns in (begin-1,begin,begin+1,end-1,end,end+1):
                assert m.in_ns_window({'ns':ns,'t':ns/1e9},begin,end)==(begin<=ns<end)
                assert m.in_ns_window({'t':ns/1e9},begin,end)==(begin<=ns<end)


def test_full_analytic_fixture_122_checks_and_old_numeric_thresholds():
    import types
    path=ROOT/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py'
    prefix=path.read_text().split('\nCASES=[]',1)[0]
    scope={'__file__':str(path),'__name__':'synthetic_fixture_only'}
    exec(compile(prefix,str(path),'exec'),scope)
    raw=copy.deepcopy(scope['base']);contract=scope['contract']
    watermark=raw['dense_links'][-1]['ns'];tail=m.final_stream_watermarks(raw,watermark)
    physics={'success':True,'pause':True,'time_step':.002};world={'success':True,'sim_time':watermark/1e9,
        'model_names':contract['world_fixture']['fixed_world_models']+[contract['model_name']]}
    raw['final_drain']=dict(tail,physics=physics,world_queries=[world,copy.deepcopy(world)],
        source_watermark_ns=watermark,stable_world_query_ns=[watermark,watermark],wall_elapsed_s=.1)
    raw['command_ingress']=[ingress_row(r['ns'],i,r['v'],r['w']) for i,r in enumerate(raw['commands_sent'])]
    actual=m.assess(raw,contract);old=scope['probe'].assess(raw,contract)
    assert actual['pass'] and len(actual['checks'])==122 and old['pass']
    added={'final_paused_source_watermark_drained','physical_command_ingress_source_schema_and_order',
           'physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain'}
    assert {k:v for k,v in actual['checks'].items() if k not in added}=={
        k:v for k,v in old['checks'].items() if k not in ('command_loss_no_relay_publication_after_drain','command_relay_observed')}
    for row in raw['commands_observed']:row['t']+=60.
    diagnostic_clock=m.assess(raw,contract)
    assert diagnostic_clock['pass']
    assert diagnostic_clock['metrics']['legacy_probe_relay_match_pass_diagnostic_only'] is False
