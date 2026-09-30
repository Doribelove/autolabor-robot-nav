import hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
def bind(path):
 p=Path(path).resolve();data=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))
author_path=S/'audits/navigation_n05_preview_selection/implementation_report.json';author=json.loads(author_path.read_text())
resources=[]
for row in author['source_bindings']:
 now=bind(row['path']);assert now['sha256']==row['sha256'];resources.append(now)
derivation_path=S/'audits/navigation_n05_metadata_derivation/derivation.json';derivation=json.loads(derivation_path.read_text())
for row in derivation['files']:
 for prefix in ('original','new'):
  b=bind(row[prefix+'_path']);assert b['sha256']==row[prefix+'_sha256'];resources.append(b)
for p in (author_path,derivation_path,S/'audits/navigation_n04_actor_independent/review.json',S/'audits/navigation_n04_release_source_review/source_review.json',D/'selector_author_tests.log',D/'checks.json',D/'checks.log',D/'check_components.py',Path(__file__)):
 resources.append(bind(p))
checks=json.loads((D/'checks.json').read_text());assert checks['passed']==checks['total']==9
report=dict(schema='astra_navigation_n05_components_independent_review_v1',status='selection_and_actor_metadata_increment_review_pass_native_and_integration_pending',
 review_roles=dict(selector='environment_audit independently reviews design_audit component',
 actor='environment_audit independently reviews root N04-to-N05 delta only; original N04 actor authored by environment_audit and independently reviewed by historical_audit',
 prior_actor_review=bind(S/'audits/navigation_n04_actor_independent/review.json')),
 selector=dict(source=author['source_bindings'][0],design=bind(S/'contracts/navigation_n05_preview_selection_design.json'),
  author_tests_independently_rerun=19,log=bind(D/'selector_author_tests.log'),
  independent_reference_cases=36,
  conclusion='Exact declared single selection factor: moving eligibility = original backup mask AND original unrounded full-preview clearance>=0. Index0 stop keeps its original backup eligibility. Rank/lowest-index ties use original analytic prior.',
  original_mask_prior_feature_mutation=False,original_class_monkeypatched=False,
  rollout='Wrapper calls original rollout once per requested preview/backup, then verifies original returned object. Capture checks the16 alternating action/backup pairs and inherited backup values, with finally cleanup; no second geometry query.',
  full_preview='float(original_verify_return), captured before clipping/normalization/float32 feature storage. Negative−1e−50 remains ineligible although candidate feature rounds to−0.',
  command='state/step/predict/command/goal_reached remain exact inherited function objects. command revalidates backup on current scan/state; full-preview eligibility is a selection factor only, not a new control-time shield.',
  fallbacks='No original backup pass retains original reason. Some moving backups but no eligible preview/stop returns index0/proposedNone with no_full_preview_eligible_action. It does not set stop backup mask true or certify a physical stop.',
  actor='Any actor or checkpoint metadata rejected at construction; an actor installed later is rejected by select. No learned behavior in this treatment.',
  serialization='Caller must retain original build/command lock; subclass explicitly rejects recursive capture and checks exact protocol. This helper does not supply a new ROS node or concurrent transaction owner.',
  rounded_input_boundary='The public pure function checks shape/type/finiteness, not provenance. Passing an already-rounded float32 full array can change eligibility; independent fixture chooses7 for rounded−0 vs6 for original−1e−50. Current integration subclass supplies original values correctly. Future node must use build output without reconstructing from candidate features.',
  qualification='Synthetic verifier fixtures establish factor isolation; no new full planner geometry benchmark, runtime latency or live closed-loop qualification.'),
 actor_metadata=dict(derivation=bind(derivation_path),files=derivation['files'],
  exact_mechanical_reconstruction=True,
  permitted_changes=['N04/N05 C++ type/class/header names and logs','versioned plugin filename and status wire schema','exact one allowed method: astra_analytic_preview_eligible_n05'],
  methods_added=['astra_analytic_preview_eligible_n05'],methods_removed=[],
  event_topic_unchanged='/astra/navigation/gate',status_topics_unchanged='/astra/actors/<model>/gate_status',transport_limit_ns=50_000_000,
  callback_begin_dt_lifecycle_policy_math_unchanged=True,
  actual_parent_world_parser='Saved N04 prepared actor world, transformed only new plugin filename and new method identity, accepted by N05 parser and rejected by N04 parser.',
  boundaries='Wrong method/seed/run/template/topic/transport/delay, duplicate and old spatial activation fields rejected; old status schema, stale/missing/future active latch, fault and identity errors rejected. All status timeline logic is byte-derived unchanged.',
  original_algorithm_newly_independently_reviewed=False,original_independent_review_inherited=True,
  no_compiler_or_native_tests_run=True,native_library_qualified=False,
  pending=['new targeted native build after N04 terminal','exact built ELF/source binding','new namespace consistent collector/evaluator/launch integration and live evidence']),
 independently_executed_checks=bind(D/'checks.json'),
 integration_requirements=['Use original RouteAwareRuntime for control and PreviewEligibleRuntime for treatment; no M08 tracker/actor factor.',
  'Record true backup mask separately from selection_eligibility, original_analytic, chosen/proposed/reason and original unrounded full values.',
  'Retain common command/terminal/deadline gates and physical evaluator; zero fallback is not a stop certificate.',
  'Both paired methods use the same new actor metadata namespace/build and unchanged geometry/policy math.',
  'Bind new module/source/native outputs in a fresh independently reviewed release; never mutate N04.'],
 execution=dict(ros_started=False,build_started=False,benchmark_run=False,training=False,seed_allocation=False,source_modified=False,release_authority=False),
 resources=resources)
(D/'review.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
(D/'review.md').write_text('''# N05 selection and actor-metadata delta review

Both scoped components pass this independent offline review. The19 selector author tests were independently rerun. Nine additional check groups include36 selector cases against a separate Python filter/max reference, exact derivation of all three actor files, and actual saved-world parser/status negative cases. No source was edited; no native compilation, ROS, benchmark or seed allocation ran.

The selector preserves the genuine backup mask and all prior/features. Moving candidates additionally require the original full-preview verifier result>=0; stop retains backup-only eligibility and can win the unchanged prior. The subclass captures the existing16 original verifier returns before float32 feature conversion, with no extra rollout or geometry call. State, dynamics, prediction, command backup revalidation and goal functions stay inherited. Empty eligibility returns zero with an explicit reason; it never fabricates a passed stop mask. Actor/checkpoint input is rejected.

A preserved API boundary matters for integration: the pure selector cannot recover precision lost before its call. An independent−1e−50 fixture picks action6 with the original float64 value, but picks7 when a caller first rounds it to float32 signed zero. The current subclass supplies the correct original values. Future nodes must keep these separate raw fields and must not reconstruct them from normalized candidate features.

The actor header/C++/Python contract match exact N04-to-N05 text derivation plus one allowed method string. Callback/Begin ordering, first-step zero dt,50ms transport limit, lifecycle guard, status timing and reactive policy/physical updates are unchanged. New plugin/status names prevent accidental acceptance as N04. This independently reviews root’s metadata delta only; the original actor algorithm inherits historical_audit’s prior independent review and is not claimed as newly independently reviewed by its original author.

Native build, ELF binding, node/collector/evaluator integration and fresh live qualification remain pending. The N05 study remains a bounded selection-factor diagnostic, not evidence of performance or safety. N04 bytes and release stay frozen.
''')
for r in resources:assert bind(r['path'])['sha256']==r['sha256']
print(json.dumps(dict(review=bind(D/'review.json'),markdown=bind(D/'review.md'),resources=len(resources)),indent=2))
