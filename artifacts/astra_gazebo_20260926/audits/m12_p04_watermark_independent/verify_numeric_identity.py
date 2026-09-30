#!/usr/bin/env python3
"""Analytic toy baseline and original negative replay; no new simulation evidence."""
import ast,copy,hashlib,json,time,types
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'src/simulation/m2_gazebo/scripts';T=W/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
source=T.read_text();prefix=source[:source.index('\nCASES=[]')];ns={'__file__':str(T),'__name__':'synthetic_baseline_construction'};exec(compile(prefix,str(T)+':baseline_prefix_only','exec'),ns)
old=ns['probe'];new=types.ModuleType('p04_independent');new.__file__=str(P/'astra_physical_actuation_probe_p04.py');exec(compile(Path(new.__file__).read_bytes(),new.__file__,'exec'),vars(new))
raw=copy.deepcopy(ns['base']);contract=ns['contract'];watermark=raw['dense_links'][-1]['ns'];tail=new.final_stream_watermarks(raw,watermark);assert tail['complete']
physics={'success':True,'pause':True,'time_step':contract['world_fixture']['max_step_s'],'status_message':'synthetic fixture, no service invoked'};world={'success':True,'sim_time':watermark/1e9,'model_names':contract['world_fixture']['fixed_world_models']+[contract['model_name']],'status_message':'synthetic fixture, no service invoked'}
raw['final_drain']=dict(tail,source_watermark_ns=watermark,physics=physics,world_queries=[copy.deepcopy(world),copy.deepcopy(world)],stable_world_query_ns=[watermark,watermark],wall_elapsed_s=.1,wall_budget_s=5.)
a=old.assess(raw,contract);begin=time.monotonic();b=new.assess(raw,contract);elapsed=time.monotonic()-begin
assert a['pass'] is True and b['pass'] is True and len(a['checks'])==120 and len(b['checks'])==121
assert {k:v for k,v in b['checks'].items() if k!='final_paused_source_watermark_drained'}==a['checks'];assert b['metrics']==a['metrics']
# This is only a toy failure: actual motion-negative gates must remain negative.
raw['links'][100]['poses'][1][2]=float('nan');bad=new.assess(raw,contract);assert bad['pass'] is False
# Unmodified old raw never qualifies for P04: historical service metadata absent.
rid='m12_physics_p03_s009_kd6';oldraw=json.loads((S/'runs'/rid/'probe_raw.json').read_text());expected=json.loads((S/'runs'/rid/'probe_report.json').read_text());original=old.assess(oldraw,oldraw['contract']);assert all(expected[k]==v for k,v in original.items());rerun=new.assess(oldraw,oldraw['contract']);assert rerun['pass'] is False and rerun['assessment_incomplete'] is True and rerun['checks']['final_paused_source_watermark_drained'] is False
assert {k:v for k,v in rerun['checks'].items() if k!='final_paused_source_watermark_drained'}==original['checks']
# Recorded final clock is used only as a diagnostic hypothetical watermark, not a service observation.
stream_result=new.final_stream_watermarks(oldraw,37144000000);assert stream_result['complete'] is False
out={'synthetic_only':True,'no_ROS_or_Gazebo':True,'analytic_baseline_old_120_true':True,'analytic_baseline_new_121_true':True,'old_check_values_exactly_unchanged':True,'all_numerical_metrics_exactly_unchanged':True,'new_finite_failure_still_rejected':bad['pass'] is False,'full_new_assessor_elapsed_wall_s_in_offline_toy':elapsed,'check_names':sorted(b['checks']),'historical_unmodified_kd6':{'original_replay_exact':True,'new_gate_false_due_missing_service_metadata':True,'all_old_54_checks_unchanged':True,'diagnostic_hypothetical_watermark_only':stream_result},'inputs':[{'path':str(p),'sha256':sha(p)} for p in [T,Path(old.__file__),Path(new.__file__),S/'contracts/m12_physics_p02.yaml',S/'runs'/rid/'probe_raw.json',S/'runs'/rid/'probe_report.json']],'script_sha256':sha(__file__),'command':'OPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+str(Path(__file__).resolve())}
(D/'numeric_identity.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ('inputs','check_names')},indent=2))
