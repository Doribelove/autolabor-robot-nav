import pathlib,json,hashlib,os,time
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');A=pathlib.Path(__file__).parent;D=S/'audits/navigation_n06_runtime_closure_r01'
def digest(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  start=os.fstat(f.fileno())
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
  end=os.fstat(f.fileno())
 assert (start.st_dev,start.st_ino,start.st_size,start.st_mtime_ns,start.st_ctime_ns)==(end.st_dev,end.st_ino,end.st_size,end.st_mtime_ns,end.st_ctime_ns)
 return h.hexdigest(),end.st_size
expected={'implementation_report.json':'d310d6b1e49c2d8090df7247fa9ef9d62c64989f77d52ad6cf3571d3af665fab','source_evidence_manifest.json':'429575f85cb336923a64ebc3b6a23656e655c8135909599f71462f8666a18b31','closure.json':'25f882db92b529ee5a16b1350807d56d4ed053383f9ccc41d2ff93fba0280c93'}
for name,sha in expected.items():assert digest(D/name)[0]==sha
m=json.loads((D/'source_evidence_manifest.json').read_text());rows={}
for group in ('owned_sources','runtime_resources','author_evidence','manager_and_risk_bindings'):
 for r in m[group]:
  if r['path'] in rows:assert rows[r['path']]==r['sha256']
  rows[r['path']]=r['sha256']
checked=[];failures=[];start=time.monotonic()
for path,expected_sha in sorted(rows.items()):
 try:
  sha,size=digest(path)
  assert sha==expected_sha, 'SHA mismatch'
  checked.append({'path':path,'sha256':sha,'size_bytes':size})
 except Exception as e:failures.append({'path':path,'error':type(e).__name__+': '+str(e)})
out={'expected_freeze_hashes':expected,'unique_count':len(rows),'verified_count':len(checked),'failures':failures,'total_bytes':sum(r['size_bytes'] for r in checked),'wall_s':time.monotonic()-start,'resources':checked}
(A/'rehash_results.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
print(json.dumps({k:v for k,v in out.items() if k!='resources'}))
assert not failures
