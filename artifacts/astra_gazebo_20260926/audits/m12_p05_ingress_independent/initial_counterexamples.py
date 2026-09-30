import ast,copy,hashlib,importlib.util,json,threading
from pathlib import Path
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';D=S/'audits/m12_p05_ingress_independent';P=D/'probe_review_initial_snapshot.py'
spec=importlib.util.spec_from_file_location('p05_snapshot',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
T=W/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py';scope={'__file__':str(T),'__name__':'synthetic_fixture_only'};exec(compile(T.read_text().split('\nCASES=[]',1)[0],str(T),'exec'),scope)
raw=copy.deepcopy(scope['base']);contract=scope['contract'];watermark=raw['dense_links'][-1]['ns'];tail=m.final_stream_watermarks(raw,watermark);physics={'success':True,'pause':True,'time_step':.002};world={'success':True,'sim_time':watermark/1e9,'model_names':contract['world_fixture']['fixed_world_models']+[contract['model_name']]};raw['final_drain']=dict(tail,physics=physics,world_queries=[world,copy.deepcopy(world)],source_watermark_ns=watermark,stable_world_query_ns=[watermark,watermark],wall_elapsed_s=.1)
def row(e,i):
 ns=e['ns'];return {'ns':ns,'t':ns/1e9,'seq':i,'frame':'world','statuses':[{'name':'astra_physical_command_ingress','hardware_id':'simulation-only','level':0,'message':'accepted_queued','value_count':7,'values':{'input_topic':'cmd_vel','requested_linear_x':str(e['v']),'requested_angular_z':str(e['w']),'accepted':'true','disposition':'accepted_queued','backend_fault':'false','activation_time_s':str(ns/1e9)}}]}
raw['command_ingress']=[row(e,i) for i,e in enumerate(raw['commands_sent'])]
baseline=m.assess(raw,contract);assert baseline['pass'] and len(baseline['checks'])==123
out={'synthetic_only':True,'assessor_snapshot':{'path':str(P),'sha256':hashlib.sha256(P.read_bytes()).hexdigest()},'baseline':{'pass':True,'checks':len(baseline['checks'])},'cases':{}}
# Keep every source/command/motion row intact: only final driver time impossible after paused source watermark.
r=copy.deepcopy(raw);bad=r['command_ingress'][-1];bad['ns']=watermark+2000000;bad['t']=bad['ns']/1e9;bad['statuses'][0]['values']['activation_time_s']=str(bad['t']);a=m.assess(r,contract);out['cases']['driver_receipt_after_paused_watermark']={'pass':a['pass'],'failed':[k for k,v in a['checks'].items() if not v],'driver_ns':bad['ns'],'paused_watermark_ns':watermark}
# All true source streams intact, receiver /clock alone old by 2 ms (observed in P04), or delay 140 ms (P01 observed mechanism).
for delta in (-.002,.14,.25):
 r=copy.deepcopy(raw)
 for q in r['commands_observed']:q['t']+=delta
 a=m.assess(r,contract);out['cases']['headerless_callback_clock_shift_'+str(delta)]={'pass':a['pass'],'failed':[k for k,v in a['checks'].items() if not v],'relay_match_fraction':a.get('metrics',{}).get('relay_match_fraction')}
# Exact protected callback source mutates callback_errors even after active=False.
rt=next(n for n in ast.parse(P.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='runtime');fn=next(n for n in rt.body if isinstance(n,ast.FunctionDef) and n.name=='protected');ctx={'lock':threading.RLock(),'raw':{'callback_errors':[]},'active':False,'traceback':__import__('traceback')};exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(fn)],type_ignores=[])),str(P)+':exact_protected','exec'),ctx)
def fail(_):raise ValueError('synthetic callback decode error after atomic capture close')
ctx['protected'](fail)(None);out['cases']['callback_decode_failure_after_capture_close']={'active':False,'callback_errors_appended_after_close':len(ctx['raw']['callback_errors'])}
(D/'initial_counterexamples.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
