#!/usr/bin/python3
"""Small actual-verifier mutations; non-runtime stand-ins explicitly inherited."""
import json
from pathlib import Path
from probe_runtime_interface import probe
A=Path(__file__).resolve().parent
def main():
 cases=[('wrong_canonical_python',lambda d:d.update(python_executable='/usr/bin/false'),'runtime interpreter differs'),
  ('wrong_virtualenv_prefix',lambda d:d.update(python_prefix='/tmp/other_venv'),'runtime virtualenv identity differs'),
  ('user_site_enabled',lambda d:d.update(python_no_user_site=False),'runtime virtualenv identity differs'),
  ('canonical_gzserver_missing',lambda d:d.update(resources=[x for x in d['resources'] if Path(x['path']).name!='gzserver-11.15.1']),'canonical gzserver target')]
 result={}
 for name,mutation,error in cases:
  row=probe(A/'closure.json',mutation)
  row['expected_rejection_substring']=error;row['pass']=not row['accepted'] and error in (row['error']or'');result[name]=row
 if not all(row['pass']for row in result.values()):raise AssertionError(result)
 with (A/'runtime_boundary_cases.json').open('x')as f:json.dump(result,f,indent=2,sort_keys=True);f.write('\n')
 print(json.dumps({'passed':len(result),'runtime_only':True}))
if __name__=='__main__':main()
