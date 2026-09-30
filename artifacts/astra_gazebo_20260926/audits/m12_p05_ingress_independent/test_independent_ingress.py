"""Independent synthetic assessor fixtures; no ROS/Gazebo or episode data creation."""
import copy,hashlib,importlib.util,json
from pathlib import Path
import pytest,yaml
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p05.py'
spec=importlib.util.spec_from_file_location('p05_independent_assessor',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def ingress_row(ns,seq,v,w):
 return {'ns':ns,'t':ns/1e9,'seq':seq,'frame':'world','statuses':[{'name':'astra_physical_command_ingress','hardware_id':'simulation-only','level':0,'message':'accepted_queued','value_count':7,'values':{'input_topic':'cmd_vel','requested_linear_x':str(v),'requested_angular_z':str(w),'accepted':'true','disposition':'accepted_queued','backend_fault':'false','activation_time_s':str(ns/1e9)}}]}

@pytest.fixture(scope='module')
def synthetic():
 T=W/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py';scope={'__file__':str(T),'__name__':'synthetic_fixture_only'};exec(compile(T.read_text().split('\nCASES=[]',1)[0],str(T),'exec'),scope)
 raw=copy.deepcopy(scope['base']);c=scope['contract'];watermark=raw['dense_links'][-1]['ns'];tail=m.final_stream_watermarks(raw,watermark)
 physics={'success':True,'pause':True,'time_step':.002};world={'success':True,'sim_time':watermark/1e9,'model_names':c['world_fixture']['fixed_world_models']+[c['model_name']]}
 raw['final_drain']=dict(tail,physics=physics,world_queries=[world,copy.deepcopy(world)],source_watermark_ns=watermark,stable_world_query_ns=[watermark,watermark],wall_elapsed_s=.1)
 raw['command_ingress']=[ingress_row(e['ns'],i,e['v'],e['w']) for i,e in enumerate(raw['commands_sent'])]
 return raw,c,scope['probe']

def test_independent_full_analytic_baseline_exact_roster_and_numeric_gates(synthetic):
 raw,c,old=synthetic;new=m.assess(raw,c);prior=old.assess(raw,c)
 design=yaml.safe_load((S/'contracts/m12_physics_p05_design.yaml').read_bytes())
 assert new['pass'] is True and len(new['checks'])==122 and sorted(new['checks'])==sorted(design['prerequisite']['full_check_roster'])
 assert prior['pass'] is True and len(prior['checks'])==120
 added={'final_paused_source_watermark_drained','physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands','command_loss_no_driver_receipt_after_drain'}
 removed={'command_loss_no_relay_publication_after_drain','command_relay_observed'}
 assert {k:v for k,v in new['checks'].items() if k not in added}=={k:v for k,v in prior['checks'].items() if k not in removed}
 physical_metrics=[k for k in prior['metrics'] if k!='phases' and 'relay' not in k]
 assert all(new['metrics'][k]==prior['metrics'][k] for k in physical_metrics)
 report={'synthetic_only':True,'source_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'new_pass':True,'old_pass':True,'new_count':122,'old_count':120,'check_names':sorted(new['checks']),'inherited_nonphase_physical_metric_keys_exact':physical_metrics,'phase_metrics_old':prior['metrics']['phases'],'phase_metrics_new':new['metrics']['phases']}
 (D/'analytic_numeric_identity.json').write_text(json.dumps(report,indent=2)+'\n')

@pytest.mark.parametrize('delta',[-.002,.14,.25])
def test_headerless_callback_clock_is_diagnostic_only(synthetic,delta):
 raw,c,_=synthetic;r=dict(raw);r['commands_observed']=[dict(x,t=x['t']+delta) for x in raw['commands_observed']]
 # Source-stamped events/ingress and all actual physical streams are unchanged.
 a=m.assess(r,c);assert a['pass'] is True and 'command_relay_observed' not in a['checks']
 assert a['checks']['physical_command_ingress_matches_source_commands'] is True


def test_actual_driver_acceptance_inside_loss_interval_is_rejected(synthetic):
 raw,c,_=synthetic;r=dict(raw);r['command_ingress']=copy.deepcopy(raw['command_ingress'])
 start=r['schedule_start_ns'];index=max(i for i,e in enumerate(r['commands_sent']) if e['ns']<start+32000000000)
 e=r['command_ingress'][index];ns=start+32000000000+int(round(c['gates']['command_loss_relay_drain_grace_s']*1e9));e.update(ns=ns,t=ns/1e9);e['statuses'][0]['values']['activation_time_s']=str(ns/1e9)
 a=m.assess(r,c)
 assert a['assessment_incomplete'] is False and a['pass'] is False
 assert a['checks']['command_loss_no_driver_receipt_after_drain'] is False
 assert a['checks']['physical_command_ingress_matches_source_commands'] is True
 assert [k for k,v in a['checks'].items() if not v]==['command_loss_no_driver_receipt_after_drain']

@pytest.mark.parametrize('kind',['future_paused_world','offgrid','missing','duplicate','early','wrong_count','extra_bootstrap_after_start','two_terminal','wrong_payload','reverse_source_order'])
def test_ingress_source_fault_never_reaches_true_assessment(synthetic,kind):
 raw,c,_=synthetic;r=dict(raw);r['command_ingress']=copy.deepcopy(raw['command_ingress']);rows=r['command_ingress'];start=r['schedule_start_ns'];formal=next(i for i,e in enumerate(rows) if e['ns']>=start)
 if kind in ('future_paused_world','offgrid'):
  ns=r['final_drain']['source_watermark_ns']+(2000000 if kind=='future_paused_world' else -1);rows[-1].update(ns=ns,t=ns/1e9);rows[-1]['statuses'][0]['values']['activation_time_s']=str(ns/1e9)
 if kind=='missing':rows.pop(formal+1)
 if kind=='duplicate':rows.insert(formal,copy.deepcopy(rows[formal]))
 if kind=='early':
  ns=rows[formal+1]['ns']-2000000;rows[formal+1].update(ns=ns,t=ns/1e9);rows[formal+1]['statuses'][0]['values']['activation_time_s']=str(ns/1e9)
 if kind=='wrong_count':rows.pop(-2);rows[-1]['seq']-=1
 if kind=='extra_bootstrap_after_start':rows.insert(formal+1,ingress_row(start+2000000,rows[formal]['seq']+1,0.,0.));[x.update(seq=i) for i,x in enumerate(rows)]
 if kind=='two_terminal':rows.append(ingress_row(rows[-1]['ns'],rows[-1]['seq']+1,0.,0.))
 if kind=='wrong_payload':rows[formal+5]['statuses'][0]['values']['requested_linear_x']='.123'
 if kind=='reverse_source_order':rows[formal+1],rows[formal+2]=rows[formal+2],rows[formal+1]
 a=m.assess(r,c)
 assert a['pass'] is False and a['assessment_incomplete'] is True
 assert not all(a['checks'][k] for k in ('physical_command_ingress_source_schema_and_order','physical_command_ingress_matches_source_commands'))


def test_all_2ms_start_phase_boundaries_and_grace_remain_exact_integer_arithmetic():
 for start in range(1000000000,1200000001,2000000):
  for offset in (0,2000000000,6000000000,10000000000,18000000000,32000000000,36000000000):
   upper=start+offset;lower=upper-750000000
   for ns in (lower-1,lower,lower+1,upper-1,upper,upper+1):
    assert m.in_ns_window({'ns':ns,'t':ns/1e9},lower,upper)==(lower<=ns<upper)
    assert m.in_ns_window({'t':ns/1e9},lower,upper,inclusive_end=True)==(lower<=ns<=upper)
  lower=start+32000000000+120000000;upper=start+36000000000
  assert not m.in_ns_window({'ns':lower-1},lower,upper)
  assert m.in_ns_window({'ns':lower},lower,upper)
  assert not m.in_ns_window({'ns':upper},lower,upper)
