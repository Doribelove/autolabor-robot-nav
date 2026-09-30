"""Close N05 immutable resources after independent source review; no ROS/Gazebo."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

S=Path(__file__).resolve().parents[1]
W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import navigation_evidence_n05 as evidence
import run_navigation_n05 as manager

OUT=S/'audits/navigation_n05_root_release'
PREPARED=S/'contracts/navigation_n05_prepared_r01/prepared.json'
QUAL=S/'contracts/passive_q05_navigation_evidence_qualification_r1.json'
INDEPENDENT=S/'audits/navigation_n05_runtime_independent/review.json'
OLD_REVIEW=S/'audits/navigation_n04_release_source_review/source_review.json'
SOURCE_REVIEW=OUT/'source_review.json'
CANDIDATE=S/'contracts/navigation_n05_release_candidate_r01.json'
RELEASE=S/'contracts/navigation_n05_release.json'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def bound(path):return {'path':str(Path(path).resolve()),'sha256':sha(path)}

def close_paths():
    prepared=json.loads(PREPARED.read_text())
    qualification=json.loads(QUAL.read_text())
    prior=json.loads(OLD_REVIEW.read_text())
    independent=json.loads(INDEPENDENT.read_text())
    if (independent.get('independent_review') is not True
            or independent.get('pass_') is not True
            or independent.get('status') != 'independent_N05_r02_source_and_prepared_input_review_pass'
            or independent.get('full_runtime_release_review') is not False):
        raise ValueError('N05 independent runtime review did not pass')
    if len(prepared['units'])!=24 or prepared['stage']!=manager.NAME:
        raise ValueError('N05 exact 24-unit prepared roster required')
    if qualification.get('qualified_for_navigation_evidence') is not True:
        raise ValueError('inherited P05 Kd6 navigation geometry prerequisite absent')
    oldledger=json.loads((S/'queue/navigation_n04_ledger.json').read_text())
    if oldledger['status']!='terminal' or len(oldledger['units'])!=48:
        raise ValueError('N04 predecessor must be terminal')
    paths=set(manager.required_resources())
    for unit in prepared['units']:
        paths.update(Path(r['path']) for r in unit['files']+unit['conversion_resources']+
                     [unit['conversion_manifest'],unit['converted_sdf']])
    paths.update(Path(r['path']) for r in prior['engine_module_bindings']+prior['engine_source_bindings'])
    paths.update(Path(r['path']) for r in independent['resources'])
    paths.update(Path(prior[k]['path']) for k in ('native_engine_report','native_engine_records','upstream_source_manifest'))
    paths.update(Path(r['path']) for r in qualification['evidence_bindings'])
    paths.update((PREPARED,QUAL,OLD_REVIEW,INDEPENDENT,
        Path(__file__),
        S/'queue/navigation_n04_ledger.json',
        S/'queue/navigation_n04_admin_closure.json',
        S/'audits/navigation_n05_root_scene_closure_r01.py',
        S/'audits/navigation_n05_root_scene_closure_r01/report.json',
        S/'audits/navigation_n05_root_launch_preflight_r01.py',
        S/'audits/navigation_n05_root_launch_preflight_r01/report.json',
        S/'audits/navigation_n05_runtime_derivation/source_snapshot_r02.json',
        S/'audits/navigation_n05_native_build/prebuild_elf_manifest.json',
        S/'audits/navigation_n05_native_build/build_result_r01.json',
        S/'audits/navigation_n05_native_build/build_result_r02.json',
        S/'audits/navigation_n05_native_build/inherited_cmake_pre_n05.txt',
        S/'logs/navigation_n05_plugin_ldd_r01.log',
        S/'logs/navigation_n05_pipeline_tests_r02.log',
        S/'audits/navigation_n05_components_independent/review.json',
        S/'audits/navigation_n05_scene_derivation/implementation_report.json',
        S/'scenes/navigation_n05_development_r01/preparation_report.json',
        W/'scripts/astra_gazebo/tests/test_navigation_n05_pipeline.py',
        W/'scripts/astra_gazebo/tests/test_navigation_process_cleanup_n05.py',
        W/'scripts/astra_gazebo/tests/test_coordinated_route_preview_n05.py',
        W/'scripts/astra_gazebo/tests/test_route_preview_selection_n05.py',
        W/'scripts/astra_gazebo/tests/test_event_actor_contract_n05.py',
        W/'src/simulation/m2_gazebo/CMakeLists.txt'))
    paths={p.resolve() for p in paths}
    missing=[str(p) for p in paths if not p.is_file()]
    if missing:raise FileNotFoundError(missing[:5])
    # The role map is copied from the frozen N04 independent engine inventory;
    # only the separately built event actor is a new binary identity.
    engine=[]
    for item in prior['engine_module_bindings']:
        if item['role']=='actor_binaries':
            engine.append(dict(role='actor_binaries',**bound(W/'devel/lib/libastra_event_actor_n05_plugin.so')))
        else:
            if sha(item['path'])!=item['sha256']:raise ValueError('inherited native engine changed: '+item['role'])
            engine.append(item)
    if len(engine)!=len({e['role'] for e in engine}):raise ValueError('duplicate native engine roles')
    paths.update(Path(r['path']) for r in engine)
    paths.add(W/'devel/lib/libastra_event_actor_n05_plugin.so')
    return paths,prepared,qualification,prior,independent,engine

def main():
    if OUT.exists() or CANDIDATE.exists() or RELEASE.exists():
        raise FileExistsError('N05 release identity already exists; preserve it')
    paths,prepared,qualification,prior,independent,engine=close_paths()
    OUT.mkdir()
    # The independent N05 review's actual checks and its preserved negative
    # r01 are the source review authority. Root merely binds all active bytes.
    resources=[bound(p) for p in sorted(paths)]
    source_review=dict(schema='astra_navigation_source_review_v1',stage=manager.NAME,
        status='independently_reviewed',review_revision='N05_r02_after_port_repair',
        reviewer='historical_audit N05 r02 independent source review; root assembled immutable roster',
        navigation_lifecycle_authority='collector_only_spawn_unpause_pause_fixed_gateway',
        robot_pose_velocity_writes=[],actor_updates='reviewed_bounded_Begin_updates',
        native_engine_report=prior['native_engine_report'],
        native_engine_records=prior['native_engine_records'],
        upstream_source_manifest=prior['upstream_source_manifest'],
        engine_source_bindings=prior['engine_source_bindings'],
        gazebo_callback_order_review=prior['gazebo_callback_order_review'],
        engine_module_bindings=engine,resources=resources,
        independent_N05_review=bound(INDEPENDENT),
        original_N04_native_review=bound(OLD_REVIEW),
        N05_actor_selector_independent_review=bound(S/'audits/navigation_n05_components_independent/review.json'),
        N05_scene_root_closure=bound(S/'audits/navigation_n05_root_scene_closure_r01/report.json'),
        N05_launch_preflight=bound(S/'audits/navigation_n05_root_launch_preflight_r01/report.json'),
        N05_native_build=bound(S/'audits/navigation_n05_native_build/build_result_r02.json'),
        scope='N05 source/launch/actor/static geometry and inherited native facts; development-only, no outcome claim',
        retained_limits=['N04 result matrix ended 24 consumed/24 retired after user pause; N05 uses new seeds.',
                         'N05 six parent geometries are previously used development layouts, not twelve new independent geometries.',
                         'Original occlusion sin4 witness failed; inherited quintic static feasibility does not prove tracking.',
                         'M08 full-chain sparse stress69.239986ms negative; it is not used here.',
                         'No training, formal evaluation, hardware, or general safety claim.'],
        source_review_does_not_grant_authority=True,execution_authorized=False,
        seed_allocation_authorized=False)
    SOURCE_REVIEW.write_text(json.dumps(source_review,indent=2)+'\n')
    release_resources=resources+[bound(SOURCE_REVIEW)]
    release_resources=sorted(release_resources,key=lambda x:x['path'])
    release=dict(stage=manager.NAME,status='root_reviewed_release',
        created_utc=datetime.now(timezone.utc).isoformat(),execution_authorized=True,
        authority='User CODEX_ASTRA_LONG_RUN_20260926.md long-run instruction resumed after pause; new isolated N05 development identity',
        maximum_episodes=24,prepared=bound(PREPARED),source_review=bound(SOURCE_REVIEW),
        passive_geometry_qualification=bound(QUAL),resources=release_resources,
        retry_allowed=False,resume_allowed=False,training_authorized=False,
        real_vehicle_authorized=False,scope='Six reused development parent geometries, two fresh-seed repetitions, analytic control/treatment only',
        image_policy='supplementary_native_frames_do_not_suppress_physical_failure_evidence',
        algorithm_failures_retained=True,infrastructure_failure_retires_remaining=True,
        formal_result=False,limitations=source_review['retained_limits'])
    CANDIDATE.write_text(json.dumps(release,indent=2)+'\n')
    candidate_sha=sha(CANDIDATE)
    try:
        manager.verify_release(CANDIDATE,candidate_sha)
    except BaseException as error:
        (OUT/'failed_preflight.json').write_text(json.dumps(dict(candidate_sha256=candidate_sha,error=repr(error)),indent=2)+'\n')
        raise
    # Hard link retains the exact reviewed bytes. Reverify canonical path.
    os.link(CANDIDATE,RELEASE)
    release_sha=sha(RELEASE)
    manager.verify_release(RELEASE,release_sha)
    seal=dict(status='N05_full_prejournal_validation_pass_no_seed_allocation_or_ROS',
        release=bound(RELEASE),candidate=bound(CANDIDATE),source_review=bound(SOURCE_REVIEW),
        resource_count=len(release_resources),prepared_units=24,independent_review=bound(INDEPENDENT),
        execution_started=False,seed_registered=False,ROS_started=False)
    (OUT/'seal.json').write_text(json.dumps(seal,indent=2)+'\n')
    print(json.dumps(dict(status=seal['status'],resource_count=len(release_resources),release_sha256=release_sha)))

if __name__=='__main__':main()
