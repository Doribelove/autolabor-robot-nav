#!/usr/bin/env python3
"""Seal prepared inputs only. Deliberately no execution or seed registry API."""
import json
from pathlib import Path
import shutil
import sys
A=Path(__file__).resolve().parent;S=A.parent.parent;W=S/'workspace';P=W/'scripts/astra_gazebo'
sys.path.insert(0,str(P))
import prepare_navigation_n06_static as p

def file_binding(path):
    r=p.binding(path);r['bytes']=Path(path).stat().st_size;return r

def main():
    if (A/'source_evidence_manifest.json').exists():raise FileExistsError('freeze already exists')
    prepared=p.read(p.PREPARED/'prepared.json');launch=p.read(A/'launch_dump_report.json');checks=p.read(A/'source_input_audit.json')
    assert len(prepared['units'])==len(launch['units'])==8
    assert 'Ran 15 tests' in (A/'author_tests_final.log').read_text() and (A/'author_tests_final.log').read_text().rstrip().endswith('OK')
    assert all(not (S/'runs'/u['run_id']).exists() for u in prepared['units'])
    p.verify_d04();p.treatment_equivalence()
    owned=[P/'prepare_navigation_n06_static.py',P/'navigation_evidence_n06.py',P/'navigation_coordinator_n06.py',
        P/'tests/test_prepare_navigation_n06_static.py',p.G/'scripts/astra_navigation_coordinator_n06.py',
        p.G/'scripts/astra_navigation_collector_n06.py',p.LAUNCH]
    borrowed=[p.G/'scripts/astra_route_lookahead_runtime_n06.py',p.G/'scripts/astra_route_coordinated_runtime.py']
    dependencies={item['path']:item for item in checks['static_local_source_graph']}
    for deriv in p.read(A/'identity_clone_derivations.json'):
        assert p.sha(deriv['parent'])==deriv['parent_sha256'] and p.sha(deriv['child'])==deriv['child_sha256']
        dependencies[deriv['parent']]=p.binding(deriv['parent'])
    extra=[p.DESIGN,p.D04_REPORT,Path(p.read(p.D04_REPORT)['source_evidence_manifest']['path']),p.SHELL,
        p.G/'launch/astra_common_navigation_n05.launch',p.G/'urdf/m2_physical_p05.urdf.xacro',
        p.G/'urdf/m2_physical_p05_gazebo.xacro',p.G/'package.xml',W/'scripts/astra_env.sh',
        Path('/opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch')]
    for block in prepared['blocks']:
        parent=p.D04/block['layout_id']
        extra.extend(x for x in parent.rglob('*') if x.is_file())
    for path in extra:dependencies[str(path)]=p.binding(path)
    for item in dependencies.values():assert p.sha(item['path'])==item['sha256']
    snapshot=A/'frozen_sources';snapshot.mkdir()
    for path in owned+borrowed:
        shutil.copyfile(path,snapshot/path.name)
        assert p.sha(path)==p.sha(snapshot/path.name)
    manifest={'schema':'astra_navigation_n06_offline_preparation_source_manifest_v1',
        'scope':'exact preparation/source/launch/input bytes; no release, actual mapped-ELF closure or runtime authorization',
        'owned_sources':[file_binding(x) for x in owned],
        'root_owned_and_frozen_runtime_nodes':[file_binding(x) for x in borrowed],
        'prepared_inputs':[file_binding(x) for base in (p.SCENES,p.PREPARED) for x in sorted(base.rglob('*')) if x.is_file()],
        'author_audits_including_failures':[file_binding(x) for x in sorted(A.rglob('*')) if x.is_file() and '__pycache__' not in x.parts],
        'read_only_dependencies':[file_binding(x) for x in sorted(dependencies)],
        'execution_authorized':False,'seeds_registered':False}
    p.write_new(A/'source_evidence_manifest.json',manifest)
    report={'schema':'astra_navigation_n06_offline_preparation_author_report_v1',
        'status':'four_block_eight_unit_offline_preparation_pass_independent_review_pending',
        'source':p.binding(P/'prepare_navigation_n06_static.py'),'source_evidence_manifest':p.binding(A/'source_evidence_manifest.json'),
        'readme':p.binding(A/'README.md'),'prepared':p.binding(p.PREPARED/'prepared.json'),'schedule':p.binding(p.PREPARED/'schedule.json'),
        'frozen_scene_root':str(p.SCENES),'frozen_prepared_root':str(p.PREPARED),
        'source_equivalence':p.binding(p.PREPARED/'factor_equivalence.json'),'source_input_audit':p.binding(A/'source_input_audit.json'),
        'launch_dump_report':p.binding(A/'launch_dump_report.json'),
        'author_tests':{'passed':15,'log':p.binding(A/'author_tests_final.log')},
        'exact_launches_expanded':8,'independent_parent_geometries':4,'source_D04_review_pending_separate_binding':True,
        'proposed_seeds':[263003001,263003002,263003003,263003004],
        'methods':list(p.METHODS),'single_behavioral_factor':{'field':'RouteConfig.lookahead_m','control_m':.7,'treatment_m':1.2},
        'unchanged':{'physical_backend':'P05 Kd6','margin_m':.33,'action_count':8,'control_period_s':.05,'decision_period_s':.25,
            'common_gate_source':'actual base odometry','activation_s':5.,'warmup_speed_mps':.8,'episode_cap_s':22.,
            'base_goal_xy_tolerance_m':.25,'base_goal_yaw_tolerance_rad':.25,'original_backup_only_analytic_selector':True},
        'N05_preview_selector_used':False,'training_or_learned_checkpoint_loaded':False,
        'paired_world_URDF_map_route_identical':True,'fullworld_reexported_raster_identical_to_D04':True,
        'retained_failure':{'path':str(A/'launch_dump_attempt01.log'),'reason':'author check compared roslaunch list remap to tuple; parser output correct; r02 normalizes comparison only',
            'source_or_prepared_input_changed_to_fix':False,'first_script_preserved':p.binding(A/'check_launch_inputs.py')},
        'remaining_prerequisites':['independent D04 split/raster review bound separately','independent N06 source/input/launch review',
            'root exact matrix manager and runtime/conversion/ELF closure','global seed collision audit/reservation and independent root execution release'],
        'limitations':['A* guide geometric clearance does not establish Ackermann corner feasibility or physical tracking',
            'D04 static001 original native witness raster-negative retained; qualified alternate raster witness copied',
            'Four static pairs are descriptive development diagnosis, not six-family promotion or confirmatory performance',
            'Prepared launch output paths do not create run directories; no actual gate timestamp or physical outcome exists'],
        'execution_authorized':False,'execution_release':False,'allocated_budget':0,'seeds_registered':False,'run_directories_created':0,
        'ROS_started':False,'training_authorized':False,'formal_result':False,'independent_review':False,'runtime_ready':False}
    p.write_new(A/'implementation_report.json',report)
    records={r['path']:r for key in ('owned_sources','root_owned_and_frozen_runtime_nodes','prepared_inputs','author_audits_including_failures','read_only_dependencies') for r in manifest[key]}
    assert all(p.sha(path)==r['sha256'] for path,r in records.items())
    receipt={'status':'offline_author_freeze_pass','report':p.binding(A/'implementation_report.json'),
        'source_manifest':p.binding(A/'source_evidence_manifest.json'),'source':report['source'],
        'root_treatment_source':p.binding(borrowed[0]),'prepared':report['prepared'],'schedule':report['schedule'],
        'unique_bound_files_postverified':len(records),'execution_authorized':False,'allocated_budget':0,'seeds_registered':False}
    p.write_new(A/'freeze_receipt.json',receipt)
    print(json.dumps(receipt,indent=2))

if __name__=='__main__':main()
