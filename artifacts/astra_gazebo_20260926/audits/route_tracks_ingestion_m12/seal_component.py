"""Freeze M12's failed-factor author result; never create execution authority."""
from pathlib import Path
import hashlib
import json

A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
def bind(path):
    path=Path(path).resolve();raw=path.read_bytes()
    return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
def write(name,value):
    path=A/name
    if path.exists():raise ValueError('existing freeze output '+str(path))
    path.write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n')

comparison=json.loads((A/'comparison.json').read_text());environment=json.loads((A/'environment.json').read_text())
protocol=json.loads((A/'protocol.json').read_text());protected=json.loads((A/'postmeasurement_bindings.json').read_text())
assert comparison['semantic_parity'] and comparison['factor_gate']=={'primary_ge5percent':True,'dense_regression_le5percent':False}
assert not comparison['all_m12_observed_whole_under50ms']
assert bind(P/'sensor_tracks_ingestion_m12.py')['sha256']==protocol['source_candidate']['sha256']
owned=[P/'sensor_tracks_ingestion_m12.py',P/'tests/test_sensor_tracks_ingestion_m12.py']
frozen=A/'frozen_sources';frozen.mkdir(exist_ok=False)
for p in owned:(frozen/p.name).write_bytes(p.read_bytes())
resources={}
def add(path):
    row=bind(path);resources[row['path']]=row
for row in protected['resources']+environment['mapped_libraries']+environment['source_modules']+environment['packages']+[environment['interpreter']]:
    assert bind(row['path'])['sha256']==row['sha256'],row['path'];add(row['path'])
for p in owned+[W/'scripts/astra_env.sh',W/'scripts/astra_train_env.sh',W/'.venv_train_cuda/pyvenv.cfg',
               S/'audits/route_tracks_m08_compatible_latency_diagnosis/report.json',
               S/'audits/route_tracks_m08_independent/review.json',
               S/'audits/route_tracks_runtime_m08/benchmark_frozen.json']:
    add(p)
for p in sorted((W/'requirements').glob('*')):
    if p.is_file():add(p)
for p in sorted(A.rglob('*')):
    if p.is_file() and p.name not in ('component_report.json','review_resources.json','hash_validation.json'):add(p)
manifest=dict(schema='astra_m12_author_review_resources_v1',resources=sorted(resources.values(),key=lambda r:r['path']),
              scope='component author source/tests/observed diagnostic dependency evidence; not a deployment release')
write('review_resources.json',manifest)
rows=[]
for c in comparison['cases']:
    rows.append({k:c[k] for k in ('name','timing','paired_median_relative_reduction','ratio_of_medians_relative_reduction','all_m12_observed_whole_under50ms','m12_over50ms_count')})
report=dict(schema='astra_m12_lazy_tiled_kd_author_component_v1',
    status='semantic_parity_pass_latency_factor_failed_dense_regression',
    independent_review=False,independent_review_pending=True,author_source_frozen=True,
    execution_ready=False,live_runtime_qualified=False,WCET_qualified=False,
    candidate_promotion=False,no_integrated_ROS_runtime=True,
    source=bind(owned[0]),tests=bind(owned[1]),resource_manifest=bind(A/'review_resources.json'),
    protocol=bind(A/'protocol.json'),comparison=bind(A/'comparison.json'),environment=bind(A/'environment.json'),
    test_result=dict(passed=20,failed=0,log=bind(A/'tests_r01.log'),tests_include_independent_all_pairs_graph=True,
                     global_FP_or_backend_proof=False),
    measured_trial_count=40,paired_count=20,case_count=2,all_pairs_all_semantic_fields_equal=True,
    excluded_from_semantic_equality=['descriptive Snapshot.statistics.processing_ns only'],
    identity_scope='Per-instance offline harness injection; explicit implementation identity; no M08 globals changed, actor absent',
    single_factor=dict(tile_rows=32,one_cached_tile=True,workers=1,eps=0.,return_sorted=True,
      radius='unchanged gap+1e-9',predicate='unchanged hypot + original np.linalg.norm in1e-12 boundary band',
      original_accept_method_bytes_equal=True,normalized_clustering_traversal_AST_equal=True,
      maximum_cached_indices_at1440_points=46080,cache_bound_excludes_python_byte_and_native_temporary_size=True),
    factor_gate=dict(primary_pass=True,dense_regression_pass=False,overall_pass=False,
                    primary_threshold=0.05,maximum_dense_median_paired_regression=0.05),
    fifty_ms_gate=dict(all_measured_M12_pass=False,sparse_failures=2,sparse_trials=10,dense_failures=0,dense_trials=10,
                      finite_sample_not_live_qualification=True),
    results=rows,
    retrieval_mechanism=dict(sparse=dict(original_scalar_native_queries=1439,tiled_native_queries=45,tiled_prefetched_rows=1440),
                            dense=dict(original_scalar_native_queries=123,tiled_native_queries=45,tiled_prefetched_rows=1440),
                            separate_profile_not_paired_deadline_timing=True),
    negative_outcomes_retained=['dense median paired regression5.576585454695434percent exceeds5percent threshold',
                               'sparse M12 two of ten whole-chain observations exceed50ms',
                               'both stress fixtures retain allfalse backup mask/zero command; not navigation-success inputs',
                               'old M08 historical69.239986ms and compatible57.912548ms negatives retained unchanged',
                               'original M08 runtime venv NumPy/SciPy import failure retained separately'],
    harness_failures_this_candidate=[],
    actions=dict(ROS_started=False,Gazebo_started=False,training_started=False,GPU_used=False,Torch_imported=False,
                 old_M08_files_modified=False,N05_release_resources_modified=False,timing_reruns_or_outlier_removal=False,
                 post_result_factor_edits=False),
    commands=[
      'source '+str(W/'scripts/astra_env.sh')+'; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+' -B -m pytest -q -p no:cacheprovider '+str(owned[1]),
      'source '+str(W/'scripts/astra_env.sh')+'; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+' -B '+str(A/'paired_benchmark.py')],
    limitations=['two fixed synthetic inputs,10 pairs each; no worst-case or population performance inference',
                 'fresh core construction and retained-input setup excluded as in frozen benchmark',
                 'analytic-only; no actor, ROS, TF callbacks or live timing/load proof',
                 'SciPy1.10.1/NumPy1.24.4 single-thread compatible environment specifically; old author environment not interchangeable',
                 'future optimization must use a new factor/protocol; this failed candidate is not to be tuned or promoted retroactively'],
    design_and_result=bind(A/'design_and_result.md'))
write('component_report.json',report)
for row in manifest['resources']:assert bind(row['path'])==row,row['path']
validation=dict(all_resources_match=True,resources_verified=len(manifest['resources']),report=bind(A/'component_report.json'),
                manifest=bind(A/'review_resources.json'),source=bind(owned[0]),tests=bind(owned[1]))
write('hash_validation.json',validation)
print(json.dumps(validation,sort_keys=True))
