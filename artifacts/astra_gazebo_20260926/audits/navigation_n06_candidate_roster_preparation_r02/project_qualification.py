#!/usr/bin/python3
"""Only normalize the one historical Q05 evidence path; preserve qualification."""
import copy,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];REPO=S.parents[1]
def bind(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def put(name,d):
 with (A/name).open('x')as f:json.dump(d,f,indent=2,sort_keys=True);f.write('\n')
def main():
 source=S/'contracts/passive_q05_navigation_evidence_qualification.json';old=json.loads(source.read_text());new=copy.deepcopy(old)
 first=old['evidence_bindings'][0]
 if first['path']!='artifacts/astra_gazebo_20260926/audits/passive_q05_qualify.py':raise ValueError('exact historical relative path changed')
 canonical=bind(REPO/first['path'])
 if canonical['sha256']!=first['sha256']:raise ValueError('historical source bytes changed')
 new['evidence_bindings'][0]=canonical
 for item in new['evidence_bindings']:
  if bind(item['path'])!=item:raise ValueError('qualification evidence changed')
 check=copy.deepcopy(new);check['evidence_bindings'][0]=first
 if check!=old:raise AssertionError('non-path qualification change')
 put('qualification_canonical.json',new)
 put('qualification_projection.json',{'schema':'astra_n06_qualification_canonical_projection_v1','original':bind(source),'projected':bind(A/'qualification_canonical.json'),
  'producer':bind(__file__),'sole_delta':{'json_pointer':'/evidence_bindings/0/path','before':first['path'],'after':canonical['path']},
  'sha256_unchanged':True,'all_other_fields_equal':True,'qualification_scope_extended':False,'execution_authorized':False})
 print(json.dumps(bind(A/'qualification_canonical.json'),indent=2))
if __name__=='__main__':main()
