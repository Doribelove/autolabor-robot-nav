import hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1]
def bind(p):
 p=Path(p).resolve(); b=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(b).hexdigest(),bytes=len(b))
author_path=S/'audits/route_tracks_runtime_m08/implementation_report.json'
author=json.loads(author_path.read_text());manifest_path=author_path.with_name('source_manifest.json');manifest=json.loads(manifest_path.read_text())
resources=[]
for group in ('sources','tests','frozen_dependencies'):
 for row in author[group]:
  actual=bind(row['path']);assert actual['sha256']==row['sha256'];resources.append(actual)
for name in ('implementation_report.json','source_manifest.json','benchmark_frozen.json','final_tests.log','compatible_environment_import_only.json'):
 resources.append(bind(author_path.with_name(name)))
resources.append(bind(S/'audits/sensor_tracks_v2_independent/review.json'))
for p in sorted(D.glob('*')):
 if p.is_file() and p.name not in ('review.json','review.md','release_required_resources.json'):
  resources.append(bind(p))
checks=json.loads((D/'small_fixtures.json').read_text());assert checks['passed']==checks['total']==10
review=dict(schema='astra_route_tracks_m08_independent_review_v1',
 status='offline_component_review_pass_existing_runtime_environment_blocked_separate_diagnostic_candidate',
 review_role='environment_audit independently reviewed design_audit sources; no source modifications',
 source_manifest=bind(manifest_path),author_report=bind(author_path),sources=author['sources'],
 independently_executed_checks=dict(small_fixtures=bind(D/'small_fixtures.json'),passed=10,total=10,
  environment_matrix=bind(D/'environment_import_diagnosis.json'),compatible_absolute_interpreter_import=bind(D/'compatible_absolute_interpreter_import.json'),
  no_author_full_test_rerun=True,no_benchmark_rerun=True,no_ros=True,no_training=True,no_gpu=True),
 findings=[dict(id='M08-ENV-01',severity='blocks_existing_venv_deployment',
  trigger='source astra_env.sh; /usr/bin/env python3 imports route_candidate_core_m08',
  observed='NumPy1.24 in existing .venv plus system SciPy1.3.3 fails at scipy.sparse.sputils np.typeDict',
  actual_node_shebang='#!/usr/bin/env python3',existing_route_launch_prefix='$(env THESIS_WS)/.venv/bin/python',
  source_or_environment_modified=False,
  author_environment='unsourced /usr/bin/python3, Python3.8.10, user-site NumPy1.21.0, system SciPy1.3.3',
  other_successful_import='sourced /usr/bin/python3 uses system NumPy1.17.4/SciPy1.3.3; not the author test environment',
  candidate_resolution='explicit new launch-prefix W/.venv_train_cuda/bin/python with source astra_env.sh; import-only pass NumPy1.24.4/SciPy1.10.1, no user-site, rospy/rosbag/cv_bridge/core/worker imported, torch not imported',
  still_required='Bind the absolute interpreter and all Python/native dependencies, then geometry/parity and complete-chain timing in this specific environment after active N04 completes; do not alter frozen .venv or reuse old timing as new-environment evidence'),
 dict(id='M08-TIME-01',severity='retained_negative_not_new_source_defect',
  measured_maximum_ms=69.239986,transaction_deadline_ms=50.,full_deadline_gate=False,
  scope='Author full-core scan acceptance+8preview+8backup+current command recheck; excludes ROS scheduling, trained actor and OS WCET',
  disposition='Retain complete sparse reobservation negative. Late computed transaction publishes zero, but a synchronous computation cannot retroactively replace the previous command during its blocked interval. Future live diagnostic must record deadline drops and actual command ingress/staleness; no hard real-time claim.')],
 reviewed_invariants=dict(
  causal_availability='Exact timestamp TF; actual odom at-or-before scan within200ms; exact odom/joint source join; accepted_now lower bound plus original scan-source expiry; no latest TF, interpolation or fabricated history.',
  commit_scope='The frozen tracker accept body is byte-equal except local clustering dependency and commits only at end. TrackSensorBuffer first commits raw frame before tracker acceptance; failure is detected by health mismatch and propagates to actual node permanent fault/zero. The standalone IO helper is not an atomic rollback abstraction.',
  history='Six distinct real200ms bucket rows inherited; current scan used separately for verifier. Bootstrap zero stamps are discarded and never substituted as history.',
  clustering='Independent all-pairs transitive-closure reference matches ten small geometries including duplicate and nextafter threshold at +/-9999m. KD search is broad phase only; original scalar norm decides near threshold. Finite tested-domain numerical claim, not a universal floating-point proof.',
  dominance='Every persistent track/cell retained. Only current point terms strictly inside a retained circle with>1e-9 interior slack omitted from query work; 1-Lipschitz distance and identical1.35m/s slopes make their terms redundant. Independent GEOS queries with actual full-body hull agree with unpruned supports within the existing1e-10 subtraction; no physical safety inference.',
  full_body='Uses inherited nine-component16-plane FullFootprintEnvelope vertices without radius substitution. Prior geometry inclusion tests retained by source hash; not rerun as a new stress matrix.',
  control='Main holds RLock before sampling source time, includes exact-TF drain in started_wall, resamples source after ingestion and before publish, rechecks health/200ms age/50ms wall budget/deadline, zeros invalid or late command. No moving output after local terminal/fault.',
  actor_worker='One daemon worker, no growing request queue; per-job deepcopy/result/event, busy immediate fallback, late result never becomes later request result. 20ms inference fallback is analytic, final50ms transaction guard remains. Worker/thread scheduling is not WCET proof.',
  namespace='New method IDs/observation schema/actor format and whole semantics SHA required despite unchanged array dimensions; old R2/route metadata rejected. No compatible newly trained M08 checkpoint exists.',
  observation_roles='Policy subscribes scan/odom/joints/map/shared route plus shared gate/status only. No Gazebo state/actor oracle input or Gazebo service write. Shared coordinator owns task outcome; local faults preserved separately even after shared complete.',
  deployment='No new coordinator enum/launch/collector/release is implemented by this review; neither direct shebang nor old launch automatically selects compatible environment.'),
 author_evidence_not_independently_rerun=dict(test_count=36,node_fake_transport_cases=6,test_log=author['author_tests']['log'],
  benchmark=bind(author_path.with_name('benchmark_frozen.json')),why='Active48-unit N04 matrix: root limited review to light source reads and small fixtures.'),
 conclusion=dict(component_semantics_acceptable_for_next_bounded_analytic_diagnostic_design=True,
  current_frozen_env_launchable=False,compatible_absolute_interpreter_import_only=True,
  future_live_release_still_requires=['explicit compatible interpreter and native/Python closure','new coordinator/launch/collector/recorder method bindings','specific-environment small parity+full-chain timing after N04','fresh separately approved identity/budget/release','deadline zero and actual command-source evidence'],
  pct_method_usable_without_new_checkpoint=False,runtime_ready=False,full_50ms_qualified=False,physical_safety_certified=False,
  learned_candidate_conditioned_prediction_implemented=False,execution_authorized=False),resources=resources)
(D/'review.json').write_text(json.dumps(review,indent=2,allow_nan=False)+'\n')
(D/'review.md').write_text('''# M08 independent component review

The six frozen new sources pass this focused offline component review. Ten independent small fixtures passed: causal acceptance/expiry, reset fault, exact odom/joint join, six real history rows, byte-equal tracker acceptance and inherited rollouts, KD connected components against all-pairs transitive closure, strict circle dominance against GEOS with the full robot hull, deadline zeroing, expired worker isolation, and namespace/terminal separation.

**Existing runtime environment is blocked.** After sourcing `astra_env.sh`, the new node’s `env python3` resolves to `.venv`, where SciPy1.3.3 requires the removed NumPy1.24 `typeDict`. The actual import failure is saved. Author36tests and timing used unsourced `/usr/bin/python3` with user-site NumPy1.21.0; sourced explicit system Python instead uses NumPy1.17.4. These are distinct environments.

A separately bound absolute `.venv_train_cuda/bin/python` is an available candidate: sourced-environment imports of rospy, rosbag, cv_bridge, M08 core and worker pass with NumPy1.24.4/SciPy1.10.1 and user-site disabled. Torch was not imported. This is import-only evidence, not cross-environment geometry, timing or runtime qualification.

The full-core author negative remains69.239986ms versus50ms, even before ROS scheduling or learned inference. No benchmark was repeated during N04. The code zeros an expired transaction at its eventual prepublish boundary; it cannot replace the last command retroactively while synchronous work blocks. A future research diagnostic must measure actual ingress and drop frequency. No worst-case real-time or physical safety claim follows.

The pure input buffer commits its raw frame before tracker acceptance; a failed tracker commit is detected as a freshness mismatch and the actual node faults/zeros. Do not reuse that helper as an atomic rollback transaction. Shared task outcomes remain separate from preserved local input faults. The policy does not subscribe to Gazebo truth.

This supports preparing a new bounded **analytic M08 diagnostic** after compatible environment closure and new method/launch/collector release integration. Runtime readiness remains false. The PCT entry additionally needs a newly trained matching namespace checkpoint; no such artifact exists. No source, old environment, N04 binding, seed registry, ROS process or GPU state was changed.
''')
for row in resources:assert bind(row['path'])['sha256']==row['sha256']
print(json.dumps(dict(review=bind(D/'review.json'),markdown=bind(D/'review.md'),resources=len(resources)),indent=2))
