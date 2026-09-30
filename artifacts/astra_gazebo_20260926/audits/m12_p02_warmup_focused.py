#!/usr/bin/env python3
"""Final warmup-only regression after 67-case snapshot; no ROS/Gazebo."""
from pathlib import Path
import ast
import copy
import hashlib
import json
import sys
import difflib

A=Path(__file__).resolve().parent;S=A.parent;T=S/'workspace/scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py'
P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p02.py'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
payload=T.read_text();prefix=payload[:payload.index('CASES=[]')];env={'__file__':str(T),'__name__':'focused_synthetic_loader'}
exec(compile(prefix,str(T),'exec'),env)
probe=env['probe'];base=env['base'];contract=env['contract']
oldpath=A/'m12_p02_probe_suite67_bcbe_snapshot.py'
old=ast.parse(oldpath.read_bytes());new=ast.parse(P.read_bytes())
oldfn=next(x for x in old.body if isinstance(x,ast.FunctionDef) and x.name=='source_packet_integrity')
newfn=next(x for x in new.body if isinstance(x,ast.FunctionDef) and x.name=='source_packet_integrity')
def scrub(fn):
    fn.body=[x for x in fn.body if not (isinstance(x,ast.Assign) and (
        any(isinstance(t,ast.Name) and t.id in ['warmup_start','warmup_end','running_stamps'] for t in x.targets)
        or any(isinstance(t,ast.Subscript) and isinstance(t.slice,ast.Index) and isinstance(t.slice.value,ast.Str) and t.slice.value.s=='recorder_warmup_before_start' for t in x.targets)))]
scrub(oldfn);scrub(newfn)
assert ast.dump(old)==ast.dump(new),'Unexpected final change beyond warmup metadata gate'
cases=[]
mutations=[('valid_realistic_warmup',lambda r:None,True),
 ('warmup_after_schedule',lambda r:r.update(warmup_start_ns=100000000000,warmup_end_ns=101000000000),False),
 ('warmup_float_ns',lambda r:r.update(warmup_start_ns=0.),False),
 ('warmup_negative_ns',lambda r:r.update(warmup_start_ns=-1),False),
 ('warmup_short',lambda r:r.update(warmup_end_ns=999999999),False),
 ('warmup_after_running',lambda r:r.update(warmup_end_ns=1000000001),False),
 ('warmup_reversed',lambda r:r.update(warmup_start_ns=2000000000),False),
 ('missing_running_status',lambda r:r['source_status'].pop(1),False)]
for name,mutate,expected in mutations:
    raw=copy.deepcopy(base);mutate(raw)
    result=probe.assess(raw,contract) if expected else {'checks':probe.source_packet_integrity(raw,contract)}
    passed=result['checks']['recorder_warmup_before_start']
    ok=passed is expected and (not expected or result['pass'] is True)
    cases.append({'name':name,'expected_warmup_gate':expected,'actual_warmup_gate':passed,'fixture_pass':ok,'full_assessor_baseline_pass':result.get('pass'),'false_checks':[k for k,v in result['checks'].items() if not v]})
report={'status':'focused_final_warmup_pass' if all(r['fixture_pass'] for r in cases) else 'fail',
        'synthetic_only':True,'no_ROS_started':True,'cases':cases,'all_other_AST_equal':True,
        'probe_sha256':sha(P),'probe_source_text':P.read_text(),'source_unchanged_during_test':sha(P)==env['SNAPSHOTS'][str(P)]['sha256'],
        'previous_67_report_sha256':sha(A/'m12_p02_probe_independent_review.json'),'previous_executed_probe_sha256':sha(oldpath),
        'fixture_source_sha256':sha(T),'fixture_source_text':T.read_text(),'contract_sha256':sha(S/'contracts/m12_physics_p02.yaml'),
        'diff_from_67_probe':''.join(difflib.unified_diff(oldpath.read_text().splitlines(True),P.read_text().splitlines(True))),
        'command':str(S/'workspace/.venv/bin/python')+' '+str(Path(__file__).resolve()),'script_sha256':sha(__file__)}
(A/'m12_p02_warmup_focused_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':report['status'],'cases':cases,'final_probe_sha256':sha(P),'report_sha256':sha(A/'m12_p02_warmup_focused_review.json')},indent=2))
sys.exit(0 if all(r['fixture_pass'] for r in cases) else 1)
