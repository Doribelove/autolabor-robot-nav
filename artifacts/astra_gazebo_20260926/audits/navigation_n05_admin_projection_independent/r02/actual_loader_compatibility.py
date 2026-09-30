"""Read-only loader/header/first-closed-pair compatibility; never close()/main()."""
from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;S=A.parents[2];p=S/'queue/close_navigation_n05_r02.py';env={'__file__':str(p),'__name__':'offline_loader_compatibility'};exec(compile(p.read_bytes(),str(p),'exec'),env)
manager,validator,deps=env['load_dependencies'](S);ledger=json.loads((S/'queue/navigation_n05_ledger.json').read_bytes());checks=[]
for method in ('astra_analytic_route_v1','astra_analytic_preview_eligible_n05'):
 u=next(u for u in ledger['units'] if u['method']==method and u.get('receipt'));r=validator(u,ledger['release_sha256']);root=Path(u['receipt']['path']).parent;art={x['path']:x['sha256'] for x in r['artifacts']};fields=[]
 for name in ('allocated.json','process.json','contract.json','collector_raw.json'):
  q=root/name
  if q.exists():
   assert env['binding'](q)['sha256']==art[str(q)];data=json.loads(q.read_bytes());fields.append(name)
   if name=='process.json':assert all(type(data[k]) is int and data[k]>0 for k in ('pid','pgid'))
 checks.append({'run_id':u['run_id'],'receipt':u['receipt'],'metadata_bound':fields,'pass':True})
for name,fields in zip(env['TABLES'],env['HEADERS']):
 with (S/name).open() as f:assert f.readline().strip().split(',')==list(fields)
r={'status':'actual_loader_and_first_closed_pair_schema_compatible','manager_name':manager.NAME,'dependencies':deps,'checks':checks,'actual_table_headers_match':True,'full_release_verifier_called':False,'real_close_or_main_called':False,'real_csv_written':False,'bag_opened':False}
print(json.dumps(r,indent=2))
