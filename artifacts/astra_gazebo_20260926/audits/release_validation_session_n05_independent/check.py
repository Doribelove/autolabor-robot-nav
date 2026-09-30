import copy, hashlib, json, tempfile, sys
from pathlib import Path
from types import SimpleNamespace
D=Path(__file__).resolve().parent; S=D.parents[1]; W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import release_validation_session_n05 as c
rows=[]
def bind(p):return dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def read(b):
 data=Path(b['path']).read_bytes()
 if hashlib.sha256(data).hexdigest()!=b['sha256']:raise ValueError('changed bytes')
 return data

def fixture(directory,case=None):
 def doc(name,value):
  p=directory/name;p.write_text(json.dumps(value));return bind(p)
 source=doc('manager.py','manager'); native=doc('native.json',{})
 hidden_qualification=doc('qualification_evidence.json',{'ok':True})
 hidden_conversion=doc('conversion_detail.json',{'ok':True})
 hidden_review=doc('source_review_detail.json',{'ok':True})
 prepared=dict(units=[dict(files=[native],conversion_manifest=native,converted_sdf=native,conversion_resources=[hidden_conversion])])
 review=dict(resources=[hidden_review],engine_module_bindings=[native],engine_source_bindings=[native],native_engine_report=native,native_engine_records=native,upstream_source_manifest=native)
 helper=bind(Path(c.__file__))
 release=dict(resources=[source,helper,native],prepared=doc('prepared.json',prepared),source_review=doc('review.json',review),passive_geometry_qualification=doc('qualification.json',dict(evidence_bindings=[hidden_qualification])))
 if case=='conflicting':release['resources'].append(dict(source,sha256='0'*64))
 if case=='noncanonical':release['resources'][0]=dict(source,path=str(directory/'sub/../manager.py'))
 if case=='missing_helper':release['resources'].remove(helper)
 release_binding=doc('release.json',release); roster=[Path(source['path']),Path(helper['path'])]
 if case=='duplicate_roster':roster.append(Path(source['path']))
 if case=='unbound_roster':roster.append(directory/'unbound.py')
 schedule=[('one',1,'analytic')]; calls=[]; pure_state={'allowed':True}
 def full(path,digest):
  calls.append(1);read(dict(path=str(path),sha256=digest))
  if not pure_state['allowed']:raise ValueError('non-pure external validation condition')
  return release,prepared,review
 manager=SimpleNamespace(__file__=source['path'],evidence=SimpleNamespace(read_bound=read),verify_release=full,required_resources=lambda:roster,identities=lambda:schedule)
 return SimpleNamespace(manager=manager,release=release,prepared=prepared,review=review,binding=release_binding,roster=roster,schedule=schedule,calls=calls,pure_state=pure_state,hidden=dict(qualification=hidden_qualification,conversion=hidden_conversion,review=hidden_review))

def expect_failure(fn):
 try:fn()
 except (ValueError,RuntimeError,OSError):return
 raise AssertionError('expected failure absent')

for label in ('conflicting','noncanonical','missing_helper','duplicate_roster','unbound_roster'):
 with tempfile.TemporaryDirectory(dir=str(D)) as td:
  f=fixture(Path(td),label)
  expect_failure(lambda:c.UnchangedReleaseSession(f.manager,f.binding['path'],f.binding['sha256']))
  rows.append(dict(name=label,passed=True))
for role in ('qualification','conversion','review'):
 with tempfile.TemporaryDirectory(dir=str(D)) as td:
  f=fixture(Path(td));session=c.UnchangedReleaseSession(f.manager,f.binding['path'],f.binding['sha256'])
  p=Path(f.hidden[role]['path']);old=p.read_bytes();p.write_text('changed')
  expect_failure(session.verify_unchanged);p.write_bytes(old);expect_failure(session.verify_unchanged)
  rows.append(dict(name='off_top_roster_'+role+'_mutated_permanent_close',passed=True))
with tempfile.TemporaryDirectory(dir=str(D)) as td:
 f=fixture(Path(td));session=c.UnchangedReleaseSession(f.manager,f.binding['path'],f.binding['sha256'])
 f.release['resources'].clear();f.prepared['units'].clear();f.review['resources'].clear()
 value=session.verify_unchanged();assert value[0]['resources'] and value[1]['units'] and value[2]['resources'] and len(f.calls)==1
 rows.append(dict(name='full_verifier_return_alias_cannot_mutate_internal_cache',passed=True))
with tempfile.TemporaryDirectory(dir=str(D)) as td:
 f=fixture(Path(td));session=c.UnchangedReleaseSession(f.manager,f.binding['path'],f.binding['sha256'])
 f.pure_state['allowed']=False
 session.verify_unchanged() # Expected: caller state is outside this byte cache.
 expect_failure(lambda:f.manager.verify_release(f.binding['path'],f.binding['sha256']))
 rows.append(dict(name='nonpure_verifier_outside_contract_counterexample',passed=True,cache_accepted=True,full_verifier_rejected=True,
   meaning='Generic manager correctness requires an independently reviewed pure deterministic full verifier; not suitable to cache wall/process/env/ledger/authorization-liveness conditions. Current inherited N04 full verifier has no such liveness read.'))
release=json.loads((S/'contracts/navigation_n04_release.json').read_text())
assert str(Path(c.__file__).resolve()) not in {str(Path(r['path']).resolve()) for r in release['resources']}
rows.append(dict(name='frozen_N04_release_does_not_bind_new_helper',passed=True,no_live_full_verifier_invoked=True))
result=dict(schema='astra_release_validation_n05_independent_small_checks_v1',passed=len(rows),total=len(rows),checks=rows,source=bind(Path(c.__file__)),no_ros=True,no_release_modified=True,no_benchmark=True)
(D/'checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
