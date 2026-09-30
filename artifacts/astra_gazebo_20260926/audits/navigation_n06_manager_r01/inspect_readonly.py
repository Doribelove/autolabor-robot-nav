#!/usr/bin/env python3
"""Read-only N06 manager dependency/authority audit; writes only this audit dir."""
import ast
import hashlib
import json
from pathlib import Path
import sys
S=Path(__file__).resolve().parents[2]
W=S/'workspace';P=W/'scripts/astra_gazebo';A=Path(__file__).resolve().parent
sys.path.insert(0,str(P))
import navigation_n06_integrity as q
import navigation_n06_process
import navigation_n06_admin_plan
import run_navigation_n06_manager
import run_smoke_a02
import run_passive_sweep_q05
import dense_sweep_n05
import evaluate_episode_n05

def bind(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
prep=S/'audits/navigation_n06_preparation/source_evidence_manifest.json'
pv=json.loads(prep.read_text());preproster={i['path']:i['sha256'] for g in pv.values() if isinstance(g,list) for i in g}
owned=q.required_manager_paths()[:4]
roster={str(p.resolve()):bind(p)['sha256'] for p in q.required_manager_paths()};roster.update(preproster)
visited=set();todo=list(owned);edges=[]
while todo:
 p=Path(todo.pop()).resolve()
 if p in visited:continue
 visited.add(p)
 for node in ast.walk(ast.parse(p.read_text())):
  names=[a.name.split('.')[0] for a in node.names] if isinstance(node,ast.Import) else [node.module.split('.')[0]] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   candidate=P/(name+'.py')
   if candidate.is_file():
    edges.append({'importer':str(p),'module':name,'file':str(candidate.resolve())});todo.append(candidate)
missing=[str(p) for p in sorted(visited) if str(p) not in roster]
changed=[path for path,sha in preproster.items() if bind(path)['sha256']!=sha]
observed=sorted({str(Path(m.__file__).resolve()) for m in list(sys.modules.values()) if getattr(m,'__file__',None) and str(Path(m.__file__).resolve()).startswith(str(W)) and str(m.__file__).endswith('.py')})
observed_unbound=[p for p in observed if p not in roster]
registry=(S/'seed_registry.csv').read_bytes();rows=q.csv_rows(registry,q.REGISTRY_FIELDS)
proposals=q.prepared_api.schedule();ids={r['run_id'] for r in proposals};seeds={str(r['seed']) for r in proposals}
found=[r for r in rows if r['run_id'] in ids or r['seed'] in seeds]
future_paths=[S/'queue/navigation_n06_ledger.json',S/'queue/navigation_n06_events',S/'queue/navigation_n06_supervisor_start.json',S/'queue/navigation_n06_manager_return.json',S/'contracts/navigation_n06_manager_release_r01.json']+[S/'runs'/r['run_id'] for r in proposals]
report={'schema':'astra_navigation_n06_manager_readonly_audit_v1','preparation_manifest':bind(prep),
 'preparation_resources':len(preproster),'preparation_changed':changed,
 'local_import_edges':edges,'local_import_resources':[bind(p) for p in sorted(visited)],'local_imports_outside_release_required_roster':missing,
 'manager_required_resources':[bind(p) for p in q.required_manager_paths()],
 'observed_workspace_python_imports':observed,'observed_workspace_imports_outside_roster':observed_unbound,
 'python':str(Path(sys.executable).resolve()),'python_version':sys.version,
 'proposed_run_ids':sorted(ids),'proposed_seeds':sorted(seeds),'actual_registry_matches':found,
 'original_csv_observations':[bind(S/name) for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')],
 'execution_artifacts_already_present':[str(p) for p in future_paths if p.exists()],
 'execution_authorized':False,'allocated_budget':0,'ros_gazebo_started':False,'seed_registration_performed':False,
 'scope':'Source/import and file observations only; not future runtime ELF/native/launch qualification.'}
if changed or missing or observed_unbound or found or report['execution_artifacts_already_present']:raise ValueError(json.dumps(report,indent=2))
q.publish(A/'readonly_dependency_authority_audit.json',report)
print(json.dumps({'status':'pass','frozen_preparation_resources':len(preproster),'local_python_modules':len(visited),'observed_workspace_python_imports':len(observed),'zero_authority':True}))
