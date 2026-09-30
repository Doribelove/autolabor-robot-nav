"""Actual nested archived_source function on frozen r02 roster; no run or ROS."""
import ast,hashlib,json,tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace'
def bind(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
source=W/'scripts/astra_gazebo/navigation_evidence_n06.py'
tree=ast.parse(source.read_text());outer=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='closure')
node=next(n for n in outer.body if isinstance(n,ast.FunctionDef) and n.name=='archived_source')
module=ast.Module(body=[node],type_ignores=[]);ast.fix_missing_locations(module)
roster=json.loads((S/'audits/navigation_n06_candidate_roster_r05_r02/resources.json').read_text());resources={r['path']:r for r in roster['resources']}
results=[]
with tempfile.TemporaryDirectory() as tmp:
 archive=Mock(side_effect=AssertionError('missing check should fail before archive'))
 namespace={'WORKSPACE':W,'Path':Path,'resources':resources,'reviewed':resources,'inherited':SimpleNamespace(archive=archive),'root':Path(tmp)}
 exec(compile(module,str(source),'exec'),namespace)
 for relative in ('src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp','src/simulation/m2_gazebo/src/astra_physics_metadata_p03_plugin.cpp'):
  try:namespace['archived_source'](relative);raise AssertionError('missing actual source unexpectedly accepted')
  except ValueError as error:results.append({'relative':relative,'error':str(error)})
 archive.assert_not_called();assert not list(Path(tmp).iterdir())
out={'schema':'astra_n06_candidate_r02_actual_closure_counterexample_v1','source':bind(source),'nested_function':'closure.archived_source','lines':[node.lineno,node.end_lineno],'frozen_roster':bind(S/'audits/navigation_n06_candidate_roster_r05_r02/resources.json'),'cases':results,'both_fail_before_archive':True,'actual_function_AST_used':True,'whole_runtime_closure_executed':False,'temporary_files_created':False,'ROS_started':False}
(A/'candidate_r02_counterexample.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
