#!/usr/bin/env python3
"""Offline-only diagnostic continuation; cannot qualify or change P01 evidence."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import types

HERE=Path(__file__).resolve().parent
STAGE=HERE.parents[1]
RUN=STAGE/'runs/m12_physics_p01'
PROBE=STAGE/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p01.py'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    frozen=PROBE.read_bytes()
    assert sha(PROBE)=='2ded4d98e2c6191c50bfa3a55792b211f4be93500fbb76f0a473f2b848dc6f4f'
    raw=json.loads((RUN/'probe_raw.json').read_text())
    contract=raw['contract']
    original=json.loads((RUN/'probe_report.json').read_text())
    receipt=json.loads((RUN/'receipt.json').read_text())
    assert original['raw_sha256']==sha(RUN/'probe_raw.json')==receipt['probe_raw.json_sha256']
    assert receipt['status']=='completed' and receipt['pass'] is False
    tree=ast.parse(frozen,filename=str(PROBE))
    mod=types.ModuleType('frozen_original');mod.__file__=str(PROBE)
    exec(compile(tree,str(PROBE),'exec'),vars(mod))
    replay=mod.assess(raw,contract)
    assert all(original[k]==v for k,v in replay.items())
    assert [k for k,v in replay['checks'].items() if not v]==['physical_backend_fault_observation_complete']
    derived=copy.deepcopy(tree)
    assessor=next(n for n in derived.body if isinstance(n,ast.FunctionDef) and n.name=='assess')
    candidates=[n for n in ast.walk(assessor) if isinstance(n,ast.If) and ast.dump(n.test)==ast.dump(ast.parse('not all(checks.values())',mode='eval').body)]
    assert len(candidates)==1
    node=candidates[0]
    before=ast.dump(node.test)
    expression="not all(checks.values()) and not (checks.get('physical_backend_fault_observation_complete') is False and sum(not value for value in checks.values()) == 1)"
    replacement=ast.parse(expression,mode='eval').body
    node.test=ast.copy_location(replacement,node.test)
    after=ast.dump(node.test)
    ast.fix_missing_locations(derived)
    # Prove every AST field except this single predicate is unchanged.
    restoration=copy.deepcopy(derived)
    ar=next(n for n in restoration.body if isinstance(n,ast.FunctionDef) and n.name=='assess')
    nr=next(n for n in ast.walk(ar) if isinstance(n,ast.If) and ast.dump(n.test)==after)
    nr.test=copy.deepcopy(candidates[0].test)
    nr.test=ast.parse('not all(checks.values())',mode='eval').body
    assert ast.dump(restoration)==ast.dump(tree)
    diag=types.ModuleType('derived_diagnostic');diag.__file__=str(PROBE)
    exec(compile(derived,str(PROBE),'exec'),vars(diag))
    result=diag.assess(raw,contract)
    assert result['checks']['physical_backend_fault_observation_complete'] is False
    assert result['pass'] is False
    assert result['assessment_incomplete'] is False
    output={
      'status':'offline_diagnostic_only_original_P01_remains_failed',
      'run_id':raw['run_id'],'seed':raw['seed'],
      'original_report_unchanged_replay_exact':True,
      'sole_allowed_precondition_failure':'physical_backend_fault_observation_complete',
      'transformation':{'original_predicate_ast':before,'diagnostic_predicate_ast':after,'expression':expression,'line':node.lineno,'all_other_ast_fields_equal':True},
      'purpose':'Continue original frozen motion gates after the sole Bool receipt-clock completeness failure; never change that check or total pass=false.',
      'canonical_original_result':{'pass':original['pass'],'assessment_incomplete':original['assessment_incomplete'],'failed_checks':[k for k,v in original['checks'].items() if not v]},
      'diagnostic_result':result,
      'diagnostic_failed_checks':[k for k,v in result['checks'].items() if not v],
      'inputs':[{'path':str(p),'sha256':sha(p)} for p in [PROBE,RUN/'probe_raw.json',RUN/'probe_report.json',RUN/'receipt.json',RUN/'actuation_contract.yaml']],
      'script_sha256':sha(__file__),
      'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),
      'no_ROS_started':True,'no_run_files_modified':True,
    }
    (HERE/'frozen_assessor_continuation.json').write_text(json.dumps(output,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'failed_checks':output['diagnostic_failed_checks'],'check_count':len(result['checks']),'metrics':{k:v for k,v in result['metrics'].items() if k in ['actual_center_rate_radps','actual_front_curvature_disagreement_inv_m','internal_target_acceleration_mps2','internal_target_center_slew_radps']},'sha256':sha(HERE/'frozen_assessor_continuation.json')},indent=2))

if __name__=='__main__':main()
