#!/usr/bin/python3
"""Derive a new reviewed disk envelope only from the exact independent pass."""
import copy, hashlib, json
from pathlib import Path
A=Path(__file__).resolve().parent; S=A.parents[1]
def bind(p):
 p=Path(p).resolve(); return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def read(p,sha):
 b=bind(p)
 if b['sha256']!=sha:raise ValueError('frozen input drift '+str(p))
 return json.loads(Path(p).read_text()),b
def main():
 author,ab=read(S/'audits/navigation_n06_runtime_closure_r02/closure.json','af6c053e0efcd78493b889f40b873c172ff94ec5cd9140274dd17babb568d6de')
 review,rb=read(S/'audits/navigation_n06_runtime_disk_closure_independent/r02_export_supplement/review.json','9154b5d8a145d747b5b15d15505a9c693ad14833637c05a1424ff9cb1e5660e7')
 if review['status']!='independent_reviewed_disk_dependency_closure_pass' or review['disk_dependency_closure_qualified']is not True or review['author_closure']!=ab:raise ValueError('independent pass does not qualify exact author')
 if len(author['resources'])!=5217 or author['independent_review']is not False:raise ValueError('author identity differs')
 for item in author['resources']:
  if bind(item['path'])!=item:raise ValueError('runtime bytes changed '+item['path'])
 for key in ('independent_resources','supplement_author_report','supplement_author_manifest'):
  item=review[key]
  if bind(item['path'])!=item:raise ValueError('review lineage changed')
 result=copy.deepcopy(author)
 result.update(status='reviewed_disk_dependency_closure',independent_review=True,
  author_closure=ab,independent_review_report=rb,independent_review_manifest=review['independent_resources'],
  author_report=review['supplement_author_report'],author_manifest=review['supplement_author_manifest'],
  disk_dependency_closure_qualified=True,runtime_ready=False,execution_authorized=False,
  reviewed_envelope_limits=review['limitations'],
  future_live_check_boundary={'gazeborc_currently_absent':True,'gazeborc_immutable_absence_guarantee':False,
   'gazeborc_path':'/opt/ros/noetic/share/gazebo_ros/.do_not_use_gazeborc',
   'native_SystemPaths_plugin_executed':False,'live_mapped_elf_observed':False,
   'current_manager_dedicated_gazeborc_absence_gate':False,
   'scope':'Disk observation only; future source/native/mapped-ELF gates remain required. No new live precheck is implemented by this envelope.'})
 if result['resources']!=author['resources'] or result['execution_environment']!=author['execution_environment']:raise AssertionError('runtime identity changed')
 with (A/'closure.json').open('x') as f:json.dump(result,f,indent=2,sort_keys=True);f.write('\n')
 print(json.dumps(bind(A/'closure.json'),indent=2))
if __name__=='__main__':main()
