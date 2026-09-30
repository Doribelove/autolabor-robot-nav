"""Build this independent source-review artifact, not an execution release."""
from pathlib import Path
import hashlib
import json

A = Path(__file__).resolve().parent
S = A.parent.parent
W = S/'workspace'
author = A.parent/'baseline_rear_runtime_b03'
manifest = json.loads((author/'source_manifest.json').read_text())
stat = json.loads((A/'static_checks.json').read_text())
def bound(path):
    path = Path(path).resolve(); raw = path.read_bytes()
    return {'path':str(path),'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}
sources = sum((manifest[key] for key in ('owned_new_sources','read_only_referenced_dependencies','tests_and_reports')), [])
extra = [author/'source_manifest.json', author/'implementation_report.json',
    W/'src/simulation/m2_gazebo/src/astra_physical_ackermann_p05_plugin.cpp',
    W/'src/navigation_arena/forks/navigation/local_planner/teb/src/teb_local_planner_ros.cpp',
    S/'audits/navigation_n01_dwa_diagnosis/upstream_navigation_1.17.3/base_local_planner/src/latched_stop_rotate_controller.cpp',
    S/'audits/navigation_n01_dwa_diagnosis/upstream_navigation_1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp',
    Path('/usr/bin/python3.8')]
resources = {row['path']:row for row in sources}
for path in extra + sorted(A.iterdir()):
    if path.is_file() and path.name not in ('review.json', 'hash_validation.json'):
        row = bound(path); resources[row['path']] = row
for row in resources.values():
    assert bound(row['path']) == row, row['path']
report = {
 'schema':'astra_baseline_rear_b03_independent_review_v1',
 'status':'bounded_offline_source_and_fake_transport_review_pass_live_pending',
 'independent_review':True,
 'author_report':bound(author/'implementation_report.json'),
 'author_source_manifest':bound(author/'source_manifest.json'),
 'source_bytes_unchanged':True,
 'execution_ready':False,'live_qualification':False,'formal_safety':False,
 'scope':{'ROS_master_started':False,'Gazebo_started':False,'build_performed':False,'training_performed':False,
          'seed_or_release_allocated':False,'real_roslaunch_preflight_performed':False,
          'fake_transport_uses_actual_node_main':True,'independent_of_author_assertions':True,
          'fake_message_containers_reused_from_author_tests':True,
          'prior_B02_independent_review_inherited_not_reclaimed':True},
 'validation':{'author_replay_passed':27,'independent_pytest_passed':9,'combined_test_count':36,
   'independent_log':bound(A/'independent_tests_r01.log'),'author_replay_log':bound(A/'author_replay.log'),
   'static_checks':bound(A/'static_checks.json'),'author_manifest_resources_verified':39,
   'config_translation_max_abs_error_m':stat['footprint_translation_max_abs_error_m'],
   'coordinator_classes_AST_equal_to_N02':True,
   'independent_cases':['three serialized full3D odom/body-twist transforms and nonidentity map commutation',
       'future source queue keeps exact timestamps',
       'per-publication expiry after first batch output faults permanently',
       'oriented map route and explicit final heading preserved; wrong frames rejected',
       'internal goal bound tested at heading/tilt/position boundaries',
       'actual coordinator main for both methods with rotated map, delayed source and premature private success'],
   'commands':[
     'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+" -B -m unittest discover -s "+str(W/'scripts/astra_gazebo/tests')+" -p 'test_baseline_rear_*b03.py' -v",
     'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+' -B -m pytest -q -p no:cacheprovider '+str(A/'test_independent.py'),
     'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+' -B '+str(A/'static_review.py')]},
 'environment':stat['environment'],
 'technical_findings':{
   'actual_odom_reference':'P05 relative-origin odom, full orientation and body twist; rear offset -0.325 m, no original topic mutation',
   'clock':'source stamps preserved; future queue, 200ms bounds, permanent faults and fresh prepublication check',
   'tf':'one static base_link-to-rear edge, monitored owner/transform and periodic publisher roster',
   'route':'full map-frame oriented fixed guide; inverse start/goal checks; request stamps not sensor evidence',
   'goal':'base source arrival remains sole success; internal rear current-pose goal is a strict subset only geometrically',
   'base_goal_bound_m':0.18289262539757686,
   'fairness':'new method identities and explicit multi-change reference/goal engineering package; no old-result reinterpretation',
   'footprint':'exact planar local-coordinate translation; tilted 3D equivalence remains unqualified'},
 'retained_limitations':[
   {'id':'partial_batch_receipt','severity':'future capture-integrity failure; no source change requested',
    'evidence':'test_independent.py expiry fixture; publish_pending returns only after entire batch; record(rows) follows return',
    'description':'An earlier published row may lack its diagnostic receipt if a later row expires and raises. Permanent fault preserved; collector must fail completeness and never fabricate receipts.',
    'author_acknowledged':True},
   {'id':'tilted_footprint','description':'yaw-only costmap is not exactly full tilted physical geometry; retained 0.0004061653716359581 m example requires engineering-margin review'},
   {'id':'route_kinks','description':'same oriented configurations, but chord geometry/curvature may differ; no Ackermann feasibility proof'},
   {'id':'callback_order','description':'goal-set containment does not order asynchronous action versus actual-odom callbacks'},
   {'id':'DWA_behavior','description':'rear lateral velocity preserved, unchanged inherited acc_lim_y=2.5 and stop/rotate limits; no maneuverability qualification'},
   {'id':'authority','description':'periodic graph and trusted-process TF checks, not instantaneous transport-security proof'}],
 'integration_prerequisites':[
   'fresh bounded stage, method identities, compatible collector/contract/actor metadata and independent release',
   'actual launch parameter expansion and complete interpreter/source/native library bindings',
   'record rear odom, original odom, TF/staticTF, source receipts, status, identity, served path and private action results; retain partial failures',
   'collector independently audits base source terminal and does not treat move_base private success as task success',
   'confirm actual TEB/DWA consume rear TF/odom/footprints and full map path under measured timing',
   'qualify tilted footprint margin and physical stopping/steering behavior without hiding DWA failures'],
 'resources':sorted(resources.values(),key=lambda x:x['path']),
 'resource_roster_scope':'independent source/test evidence only, not complete runtime execution closure',
 'review_text':bound(A/'review.md')}
(A/'review.json').write_text(json.dumps(report,sort_keys=True,indent=2,allow_nan=False)+'\n')
for row in report['resources']:
    assert bound(row['path']) == row, row['path']
validation={'schema':'b03_independent_postwrite_hash_check_v1','all_resources_match':True,
            'resources_verified':len(report['resources']),'review':bound(A/'review.json'),'text':bound(A/'review.md')}
(A/'hash_validation.json').write_text(json.dumps(validation,sort_keys=True,indent=2)+'\n')
print(json.dumps(validation,sort_keys=True))
