#!/usr/bin/env python3
import datetime,hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
p=S/'contracts/m12_physics_p06_release.json';assert sha(p)=='b1ee6e965074b712c66d257cf652c5caae59e074abd5ab730fbec5ed88b51adc';r=json.loads(p.read_bytes());prior=json.loads((S/'contracts/m12_physics_p05_release.json').read_bytes());resources=[dict(x,current_matches=sha(x['path'])==x['sha256']) for x in r['resources']]
old={(x['path'],x['sha256']) for x in prior['resources']};new={(x['path'],x['sha256']) for x in r['resources']};checks={'573_unique_resources':len(resources)==len({x['path'] for x in resources})==573,'all_current_bytes_match':all(x['current_matches'] for x in resources),'all527_prior_path_and_sha_pairs_inherited':len(old)==527 and old<=new,'PHY04_preregistration_bound_exact':(str(D/'figure_selection_preregistration.json'),sha(D/'figure_selection_preregistration.json'))in new,'independent_startup_review_bound_exact':(str(S/'audits/m12_physics_p06_startup_integration_review.json'),sha(S/'audits/m12_physics_p06_startup_integration_review.json'))in new}
out={'status':'independent_pre_result_release_check','pass':all(checks.values()),'checks':checks,'resources':resources,'release_sha256':sha(p),'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'active_run_raw_or_bag_opened':False,'offline_only':True,'script_sha256':sha(__file__)};(D/'release_pre_result_check.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'pass':out['pass'],'checks':checks}))
