#!/usr/bin/env python3
import json,hashlib,datetime
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
r=S/'contracts/m12_physics_p04_release.json';assert sha(r)=='cb256ee6105b81d2935ae37d6794df3621b282947c6a28214be53aa05a55f5f5';release=json.loads(r.read_text());old=json.loads((S/'contracts/m12_physics_p03_release.json').read_text());resources=[]
for x in release['resources']:resources.append(dict(x,current_matches=sha(x['path'])==x['sha256']))
review=json.loads((S/'audits/m12_physics_p04_watermark_repair_review.json').read_text());reviewinputs=[dict(x,current_matches=sha(x['path'])==x['sha256']) for x in review['sources']]
checks={'450_release_resources_unique':len(resources)==len({x['path'] for x in resources})==450,'all450_current_hashes_match':all(x['current_matches'] for x in resources),'all420_P03_path_sha_pairs_inherited':{(x['path'],x['sha256']) for x in old['resources']}<={(x['path'],x['sha256']) for x in resources},'all_independent_probe_review_source_hashes_match':all(x['current_matches'] for x in reviewinputs)}
out={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'read-only release verification while simulation may run; no active run evidence opened','checks':checks,'pass':all(checks.values()),'release_sha256':sha(r),'resources':resources,'review_sources':reviewinputs,'script_sha256':sha(__file__),'command':'/usr/bin/python3 '+str(Path(__file__).resolve())};(D/'release_pre_result_check.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(checks))
