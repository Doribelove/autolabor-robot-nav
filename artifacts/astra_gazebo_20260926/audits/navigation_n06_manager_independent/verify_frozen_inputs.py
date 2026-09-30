import ast,hashlib,json,sys
from pathlib import Path
S=Path(__file__).resolve().parents[2];W=S/'workspace';H=W/'scripts/astra_gazebo';A=Path(__file__).resolve().parent
report=S/'audits/navigation_n06_manager_r01/implementation_report.json';manifest=report.parent/'source_evidence_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def bound(p):return {'path':str(p.resolve()),'sha256':sha(p)}
assert sha(report)=='2a9587125804a7fac73f0a7558490196616a3c98f11702701b5980a8b6757288'
assert sha(manifest)=='50a5af06ff0be1ddd7d94ae17af3b108c41a213a855be51a7d4656cf9b3f8c8b'
m=json.loads(manifest.read_text());roster={x['path']:x for k in ('owned_sources','read_only_dependencies','author_evidence') for x in m[k]}
fail=[]
for p,row in roster.items():
 if sha(p)!=row['sha256']:fail.append(p)
assert len(roster)==352 and not fail
queue=[H/name for name in ('run_navigation_n06_manager.py','navigation_n06_integrity.py','navigation_n06_process.py','navigation_n06_admin_plan.py')];seen=set();edges=[]
while queue:
 p=queue.pop()
 if p in seen:continue
 seen.add(p);tree=ast.parse(p.read_text())
 for node in ast.walk(tree):
  names=[x.name for x in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   local=H/(name.split('.')[0]+'.py')
   if local.is_file():edges.append({'from':str(p),'to':str(local),'line':node.lineno});queue.append(local)
missing=[str(p) for p in seen if str(p) not in roster];assert not missing
sys.path.insert(0,str(H));import run_navigation_n06_manager,navigation_n06_admin_plan
observed=sorted({str(Path(x.__file__).resolve()) for x in sys.modules.values() if getattr(x,'__file__',None) and str(Path(x.__file__).resolve()).startswith(str(W))})
observed_missing=[p for p in observed if p not in roster];assert not observed_missing
snapshot=[]
for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv'):
 p=S/name;st=p.stat();snapshot.append(dict(bound(p),inode=st.st_ino,mtime_ns=st.st_mtime_ns))
result={'status':'352_bindings_and_recursive_workspace_imports_pass','author_report':bound(report),'author_manifest':bound(manifest),'unique_resources':len(roster),'mismatches':fail,'static_recursive_modules':[bound(p) for p in sorted(seen)],'edges':edges,'observed_workspace_modules':observed,'missing_observed':observed_missing,'global_csv_readonly_baseline':snapshot,'zero_authority':True}
(A/'frozen_input_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'resources':len(roster),'static_modules':len(seen),'observed_modules':len(observed),'report':bound(A/'frozen_input_verification.json')}))
