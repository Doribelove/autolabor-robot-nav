#!/usr/bin/env python3
"""One-shot author evidence seal; no allocation or simulation authority."""
import collections
import hashlib
import json
import shutil
from pathlib import Path

A = Path(__file__).resolve().parent
S = A.parent.parent
P = S/'workspace/scripts/astra_gazebo'
O = S/'scenes/development_d04_geometry_r05'

def read(path): return json.loads(Path(path).read_text())
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def bind(path):
    path = Path(path).resolve()
    return {'path':str(path),'sha256':sha(path),'bytes':path.stat().st_size}
def write(path,data):
    with Path(path).open('x') as f:
        json.dump(data,f,sort_keys=True,indent=2,allow_nan=False)
        f.write('\n')

def main():
    if (A/'source_evidence_manifest.json').exists() or (A/'implementation_report.json').exists():
        raise FileExistsError('author outputs already frozen')
    g=read(A/'generation_report.json');r=read(A/'closed_input_recheck.json')
    assert r['status']=='author_closed_input_replay_pass_independent_review_pending'
    source=P/'prepare_development_geometry_d04_r05.py'
    tests=P/'tests/test_prepare_development_geometry_d04_r05.py'
    assert sha(source)==g['source']['sha256']=='8d795b97fece3657459ae9de228962af5f086a92347d6a57bd32faf6cccb9105'
    assert 'Ran 15 tests' in (A/'author_tests_final.log').read_text() and (A/'author_tests_final.log').read_text().rstrip().endswith('OK')
    snap=A/'frozen_sources';snap.mkdir()
    for p in (source,tests,A/'recheck_closed_inputs.py',Path(__file__)):
        shutil.copyfile(p,snap/p.name)
        assert sha(p)==sha(snap/p.name)
    old_manifest=S/'audits/development_geometry_d04/r04/source_evidence_manifest.json'
    deps={row['path']:row for row in read(old_manifest)['read_only_dependencies']+r['direct_imported_workspace_sources']+g['retained_prior_bindings']}
    ref=S/'contracts/navigation_n02_prepared_r01/astra_navigation_n02_frozen_r2_v060_sensor_lifecycle_v1'
    for p in (ref/'robot_description.xml',ref/'conversion/conversion.sdf',ref/'conversion/manifest.json',old_manifest,old_manifest.parent/'implementation_report.json'):
        deps[str(p)]=bind(p)
    for row in deps.values():assert sha(row['path'])==row['sha256'],row['path']
    manifest={'schema':'astra_d04_r05_offline_source_evidence_manifest_v1',
        'scope':'offline input geometry/allocation author seal; not an executable ROS/runtime release or complete historical registry audit',
        'owned_generator_and_tests':[bind(source),bind(tests)],
        'final_geometry_inputs':[bind(p) for p in sorted(O.rglob('*')) if p.is_file()],
        'retained_author_audits':[bind(p) for p in sorted(A.rglob('*')) if p.is_file() and '__pycache__' not in p.parts],
        'read_only_dependencies':[bind(p) for p in sorted(deps)],
        'frozen_split_contract':bind(S/'contracts/splits.yaml'),
        'design_preregistered_before_selection':g['design']}
    write(A/'source_evidence_manifest.json',manifest)
    inputs=read(O/'layouts.json');static=[x for x in inputs if x['family']=='static_avoidance']
    failures=read(A/'rejected_geometry_and_raster.json')
    risks=[
        'Independent split/raster review pending; no N06 release, seed allocation or runtime qualification.',
        'Only 4 parents/family, not full development 24/family, validation or final-test population.',
        'Ownership audit covers bound A01/A02 and N04/N05 inputs plus r04 (48 prior parents); future train/validation/final allocation must consume final_ownership.json and perform its own complete registry check.',
        'Static slot001 original native witness is raster-negative; use the bound raster_witness.csv and common_bundle, not the original native witness, for any future derivation.',
        'Exported A* guide has conservative whole-footprint geometric clearance; heading corners do not prove Ackermann transitions or controller tracking.',
        'Raster/guide qualification currently covers only the four static_avoidance parents; other20 have native geometry checks only.',
        'Static planar witness, nominal actor arrivals, two-pose ray visibility and bounded wheel/tilt envelope do not qualify reactive temporal behavior, actual sensor reveal, physical tracking, elapsed22s goals or dynamic safety.',
        'Actual runtime worlds/plugin parameters, SE2 spawn/route bridge, sensor noise and fresh execution identities remain future independently reviewed work.',
        'Finite deterministic geometry/cell enumeration provides development inputs, not representative random sampling or a performance effect estimate.'
    ]
    report={'schema':'astra_d04_r05_author_closure_v1',
        'status':'24_joint_development_combinations_and4_static_raster_guides_offline_author_pass_independent_review_pending',
        'source':bind(source),'tests':bind(tests),'source_evidence_manifest':bind(A/'source_evidence_manifest.json'),
        'design':g['design'],'readme':bind(A/'README.md'),'generation_report':bind(A/'generation_report.json'),
        'closed_input_recheck':bind(A/'closed_input_recheck.json'),'output_root':str(O),
        'counts':{'selected_parent_cell_combinations':24,'per_family':r['per_family'],'retained_r04_geometry':17,'new_replacement_geometry':7,'actual_new_atomic_actor_cells':20,'prior_parent_groups_checked':len(read(A/'initial_ownership.json')['parent_groups']),'joint_rejections':150,'geometry_rejections':12,'geometry_rejection_reasons':dict(collections.Counter(x['reason'] for x in failures))},
        'tests_result':{'author_unit_tests_passed':15,'closed_input_replay_pass':True,'old_r04_manifest_resources_unchanged':r['unchanged_r04_manifest_resources'],'current_bound_resources_rechecked':r['authenticated_current_resources'],'initial_test_compatibility_failure_retained':True},
        'four_static_layout_ids':[s['layout_id'] for s in static],
        'static_raster_results':r['static_raster_rechecks'],
        'minimum_selected_raster_witness_conservative_m':min(x['selected_raster_witness_lower_m'] for x in r['static_raster_rechecks']),
        'minimum_exported_guide_full_envelope_conservative_m':min(x['exported_guide_sweep_lower_m'] for x in r['static_raster_rechecks']),
        'unchanged_margin_m':.33,'native_raster_failure_retained':True,
        'selection_uses_N05_method_outcomes':False,'selection_completed_before_parent_N05_terminal_summary':True,
        'remaining_risks_and_prerequisites':risks,'offline_environment':r['offline_environment'],
        'execution_authorized':False,'execution_release':False,'N06_release':False,'runtime_ready':False,
        'independent_review':False,'ROS_started':False,'new_geometry_or_execution_seeds_allocated':0,'global_seed_registry_modified':False,'final_outcomes_read':False}
    write(A/'implementation_report.json',report)
    unique={row['path']:row for key in ('owned_generator_and_tests','final_geometry_inputs','retained_author_audits','read_only_dependencies') for row in manifest[key]}
    assert all(sha(p)==row['sha256'] for p,row in unique.items())
    receipt={'status':'author_freeze_hash_closure_pass','report':bind(A/'implementation_report.json'),
        'source_evidence_manifest':bind(A/'source_evidence_manifest.json'),'source':bind(source),'tests':bind(tests),
        'unique_bound_files_postverified':len(unique),'output_root':str(O),'four_static_layout_ids':report['four_static_layout_ids'],
        'independent_review':False,'execution_release':False}
    write(A/'freeze_receipt.json',receipt)
    print(json.dumps(receipt,indent=2))

if __name__=='__main__':main()
