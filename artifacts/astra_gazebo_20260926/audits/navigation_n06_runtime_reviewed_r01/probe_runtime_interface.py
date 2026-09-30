#!/usr/bin/python3
"""Actual r05 verifier runtime-branch test, explicit non-runtime stand-ins.

Creates temporary synthetic documents only; never a canonical release or seed.
"""
import argparse,copy,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r05 as author
q=author.q

def probe(runtime_path,mutation=None):
    case=author.SyntheticClosureTests('test_complete_actual_prepared_structural_roster');case.setUp()
    try:
        document=q.strict_json(Path(runtime_path).read_bytes())
        for item in document['resources']:case.add(Path(item['path']))
        if mutation is None:
            case.documents.pop('runtime_closure');case.base['runtime_closure']=case.add(runtime_path)
        else:
            mutation(document);case.documents['runtime_closure']=document
        path,sha=case.document()
        try:actual=q.verify_release(path,sha);error=None
        except BaseException as exc:actual=None;error=type(exc).__name__+': '+str(exc)
        return {'accepted':actual is not None,'error':error,'actual_runtime_document':q.binding(runtime_path),
          'temporary_document_removed_on_return':True,'runtime_assertions_real':True,
          'non_runtime_native_and_converter_authentication_explicit_test_standins':True,
          'not_a_full_execution_readiness_verdict':True,'canonical_release_created':False,'seed_or_run_allocation':False,
          'resource_count_in_temporary_fixture':len(case.resources),'actual_q_module':q.binding(q.__file__),
          'exact_eight_rows_when_accepted':actual is not None and q.same_typed(actual['rows'],q.prepared_api.schedule())}
    finally:case.tearDown()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--runtime',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=probe(args.runtime)
    with args.output.open('x') as f:json.dump(result,f,indent=2,sort_keys=True);f.write('\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
