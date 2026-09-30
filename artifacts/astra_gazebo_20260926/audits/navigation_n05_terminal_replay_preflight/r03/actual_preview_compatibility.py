"""Inspect only first three formal candidates from each of two closed bags; no replay metrics."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,copy,hashlib,json,sys
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import rosbag
A=Path(__file__).resolve().parent;S=A.parents[2];source=A/'source_snapshot.py'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
tree=ast.parse(source.read_text());tree.body=[n for n in tree.body if not(isinstance(n,ast.Import) and any(a.name in ('rosbag','run_navigation_n05') for a in n.names))]
env={'__file__':str(source),'__name__':'offline','rosbag':rosbag,'manager':NS(NAME='astra_navigation_n05_preview_selection')};exec(compile(tree,str(source),'exec'),env);env['S']=S
replay=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='replay_one')
block=next(n for n in ast.walk(replay) if isinstance(n,ast.If) and ast.dump(n.test)==ast.dump(ast.parse("unit['method']==METHODS[1]",mode='eval').body))
fun=ast.parse("def check_preview(data,unit):\n eligible=Counter()\n audit=data.get('preview_selection_audit')\n").body[0];fun.body += [copy.deepcopy(block),ast.Return(value=ast.Constant(value=True))];ast.fix_missing_locations(fun);exec(compile(ast.Module(body=[fun],type_ignores=[]),str(source)+':preview','exec'),env)
ledger=json.loads((S/'queue/navigation_n05_ledger.json').read_bytes());results=[]
for method in env['METHODS']:
 u=next(u for u in ledger['units'] if u['method']==method and u.get('receipt'));receipt=env['validate_consumed_receipt'](u,ledger['release_sha256']);root=Path(u['receipt']['path']).parent
 rawpath=root/'collector_raw.json';expected={x['path']:x['sha256'] for x in receipt['artifacts']};assert sha(rawpath)==expected[str(rawpath)];raw=json.loads(rawpath.read_bytes());samples=[]
 with rosbag.Bag(str(root/'fixture.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=['/astra/route_runtime/candidates']):
   data=json.loads(msg.data)
   if raw['start_ns']<=data['stamp_ns']<raw['end_ns']:
    assert env['check_preview'](data,u);samples.append(data)
    if len(samples)==3:break
 assert len(samples)==3
 dest=A/('actual_first3_'+method+'.json');dest.write_text(json.dumps(samples,indent=2)+'\n')
 results.append(dict(run_id=u['run_id'],method=method,receipt=u['receipt'],raw_sha256=sha(rawpath),bag_expected_sha256_from_receipt=expected[str(root/'fixture.bag')],bag_current_whole_hash_recomputed=False,samples=3,pass_=True,sample_file=dict(path=str(dest),sha256=sha(dest))))
report=dict(status='six_actual_closed_candidate_preview_payloads_compatible',source_sha256=sha(source),rows=results,scope='First3 in-formal candidates per method from first closed pair; no full bag replay, no fullmatrix conclusion, no bag metrics',no_ROS=True)
(A/'actual_preview_compatibility.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'status':report['status'],'samples':6},indent=2))
