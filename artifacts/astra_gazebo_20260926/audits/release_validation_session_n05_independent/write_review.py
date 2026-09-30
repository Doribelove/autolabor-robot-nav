import hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
def b(path):
 p=Path(path).resolve();raw=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest())
checks=json.loads((D/'checks.json').read_text());assert checks['passed']==checks['total']==11
resources=[b(P/n) for n in ('release_validation_session_n05.py','tests/test_release_validation_session_n05.py','run_navigation_n04.py','navigation_evidence_n04.py','run_passive_sweep_q05.py','passive_sweep_q05.py','passive_sweep_q04.py','dense_sweep_p04.py')]
resources += [b(S/'audits/release_validation_scaling_n05/implementation_report.json'),b(S/'contracts/navigation_n04_release.json'),b(D/'check.py'),b(D/'checks.json'),b(D/'checks.log'),b(Path(__file__))]
report=dict(schema='astra_release_validation_session_n05_independent_review_v1',status='component_pass_with_pure_manager_and_new_release_integration_scope',
 new_source=b(P/'release_validation_session_n05.py'),author_report=b(S/'audits/release_validation_scaling_n05/implementation_report.json'),
 author_tests_independently_rerun=dict(command='PYTHONDONTWRITEBYTECODE=1 W/.venv/bin/python -m pytest -q -p no:cacheprovider W/scripts/astra_gazebo/tests/test_release_validation_session_n05.py',passed=8,observed_result='8 passed in 0.02s; not a performance benchmark'),
 independent_checks=b(D/'checks.json'),independent_checks_passed=11,
 source_review=dict(
 constructor='Runs the original full verifier once; caches a deep copy; consolidates canonical exact bindings and rejects conflicts, duplicate required roster, missing helper/manager source or unbound active paths; immediately validates unchanged again.',
 preserved_closure='Every release resource plus release/prepared/review/qualification documents, prerequisite evidence, native/source review resources, every prepared file and conversion resource is re-opened and hashed at each call. Current required_resources roster and identities schedule rebuilt and compared.',
 inherited_full_verifier='Reviewed N04 verify_release and its make_contract/validate_contract/native_review_facts/conversion_resources helpers. Its repeated semantics depend on bound files, source-defined constants and returned schedule/rosters; no clock/process/ledger/service check is cached by this helper.',
 no_unclosed_current_file_dependency_found=True,
 permanent_failure='All BaseException from rehash/roster/schedule closes the session; restoring bytes cannot reopen. Caller mutation of returned nested objects or original verifier aliases cannot modify cached deep copy.',
 liveness='Ledger allocation, attempt existence, process ownership/ELF/roster, service gateways, runtime records and final evidence remain separate mandatory checks; this helper neither enforces nor replaces them.',
 dependency_roster='Directory/glob changes that affect active required_resources are detected, rather than inferred from unchanged old files.',
 old_stage='Actual frozen N04 release does not contain this new helper. Construction would reject missing binding after full verifier; no actual full verifier or current run was invoked for this check.'),
 counterexample=dict(type='scope_boundary_not_current_N04_code_defect',
  fixture='External allowed flag switches false without bound-byte/roster changes. Cache still returns earlier semantics; the full verifier rejects.',
  required_precondition='Only a separately reviewed deterministic verifier may be cached. Any future verifier reading time, environment, process state, ledger, directory facts outside the roster, or mutable authorization state must keep those validations uncached or include them in a freshly checked explicit state contract.'),
 integration_requirements=['New stage release and independent source review bind helper and exact manager source; frozen N04 must stay unchanged.',
 'Verify the new manager still satisfies the pure bound-file semantics assumption.',
 'Construct one session after initial release verification in the one-shot manager process; retain unchanged verification at original preallocation/prelaunch boundaries.',
 'Never catch a closed-session failure and start a replacement session to continue the same matrix.',
 'Do not move ledger/runtime/terminal validations into cached semantic work.'],
 performance=dict(measured=False,claim='Repeated all-matrix semantic reconstruction can be avoided; repeated full-resource hashing still occurs. No runtime/speedup or end-to-end complexity measurement was performed during active N04.'),
 execution=dict(integrated=False,authorized=False,ros_started=False,seed_or_release_changes=False),resources=resources)
(D/'review.json').write_text(json.dumps(report,indent=2)+'\n')
(D/'review.md').write_text('''# N05 release-validation cache: independent review

The unintegrated component passes this scoped source review. Eight author tests independently reran successfully; eleven additional small checks passed. Nested qualification, review and conversion evidence remains hashed even when omitted from the fixture’s top-level resource list. Conflicting/noncanonical paths, unbound dependencies, missing helper, changed rosters and restored-after-failure bytes are rejected. Returned data cannot mutate the cached copy.

The inherited N04 full verifier was inspected as a bound-file semantic validator. No additional current file dependency or stateful runtime check was found that this helper silently omits. The new helper still hashes the complete release and explicit nested evidence on every call and rebuilds current dependency/schedule rosters.

This is not a generic cache for arbitrary validators. A deliberate external-state fixture demonstrates that changing an unbound authorization flag can make a fresh full verifier reject while the cache returns its prior value. Future integration must preserve the pure, deterministic manager contract and leave ledger, process, service, environment/liveness and terminal checks outside cached work. A permanently closed session must not be replaced to resume its failed matrix.

The actual frozen N04 release does not bind this helper, so it cannot be used there. A new stage must bind both helper and manager and independently review integration at the original validation boundaries. No release, ledger, source or live run was changed. No performance benchmark or speedup claim was made.
''')
for r in resources:assert b(r['path'])['sha256']==r['sha256']
print(json.dumps(dict(review=b(D/'review.json'),markdown=b(D/'review.md'),resources=len(resources)),indent=2))
