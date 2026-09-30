import hashlib,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import frozen_r2_route_frame_bridge_n05 as bridge
from frozen_r2_sensor_lifecycle import authenticate_frozen,FROZEN_CHECKPOINT_SHA256

def bind(path):
 p=Path(path).resolve();raw=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
frozen=authenticate_frozen(W);checkpoint=W/'artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt'
assert bind(checkpoint)['sha256']==FROZEN_CHECKPOINT_SHA256
resources=[bind(P/'frozen_r2_route_frame_bridge_n05.py'),bind(P/'tests/test_frozen_r2_route_frame_bridge_n05.py')]
resources.extend(bind(r['path']) for r in frozen)
resources.extend(bind(p) for p in (checkpoint,P/'frozen_r2_sensor_lifecycle.py',P/'route_world_adapter.py',P/'prepare_navigation_n01_scene.py',P/'common_route.py',P/'static_map_export.py',W/'src/simulation/m2_gazebo/scripts/astra_frozen_r2_sensor_lifecycle.py',D/'tests_r01.log',D/'tests_r02.log',Path(__file__)))
report=dict(schema='astra_r2_route_frame_bridge_n05_component_v1',status='pure_component_author_tests_pass_independent_review_and_runtime_integration_pending',
 method_identity=bridge.METHOD,source=bind(P/'frozen_r2_route_frame_bridge_n05.py'),tests=bind(P/'tests/test_frozen_r2_route_frame_bridge_n05.py'),
 tests_passed=23,final_log=bind(D/'tests_r02.log'),runtime_interpreter=str(W/'.venv/bin/python'),
 scope='Rigid world translation/rotation of the same canonical frozen R2 task. No new ROS node/launch; old files unchanged.',
 actual_legacy_call_review=dict(
  source=bind(W/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py'),
  odometry='policy_pose reads odom.position.x+spawn_world_x, y and quaternion yaw',
  scan='_scan resamples unchanged local LaserScan, looks up odom<-laser at scan header stamp, then adds spawn_world_x to TF.x before append_scan_with_pose',
  required_private_view='route_pose shifted by +5.4m in x; old +(-5.4) restores canonical route x. Both odom and laser TF use the same transform.',
  health='Original latest TF health query can use health_transform only; scan conversion still requires exact requested source transform.',
  actual_code_fixture='Frozen policy_pose/_scan and original resample_scan AST bodies execute on transformed private views with fake transport. Identical model scan arrays, equivalent pose and identical original source seconds verified; no ROS/torch import.',
  commit_guard='Future wrapper must retain sensor-lifecycle r1 real last_runtime_scan_stamp/failure-count append proof. First TF precheck alone is insufficient; this bridge does not claim an observation was committed.'),
 coordinates=dict(world_from_route='Explicit SE2 R->W, no interpolation or fitting',world_from_odom='Explicit real odom O->W, independent of route origin',
  input_real_parent_frames=['configured world/map','configured actual odom'],robot_child='base_link',laser_child='laser_link',
  private_parent_alias='odom',private_scope=bridge.PRIVATE_SCOPE,
  physical_ROS_frame_mutation=False,public_TF_broadcast=False,static_odom_relabel_allowed=False,
  full_pose='left multiply world yaw quaternion; preserve z/roll-pitch content, no normalization repair',
  body_twist='same immutable object and six components, never world-rotated',laser='same immutable object, ranges including+inf, angles/intensities/time fields unchanged',
  command='same BodyTwist object; bridge performs no command projection or physical capability change'),
 source_time=dict(exact_int64=True,bool_or_float_coercion=False,source_stamp_preserved=True,receipt_and_availability_preserved=True,
  first_available='decision now>=actual available_source_ns; source and receipt cannot follow availability',
  age_limit_ns=200_000_000,receipt_source_lead_age_limit_ns=200_000_000,scan_tf_exact_source_equality=True,
  sequence_reset_scope='Pure adapter has no clock/sequence state. Future wrapper must retain SensorIngress queues, source ordering and clock-reset fault lifecycle; no zero-stamp bootstrap substitution.'),
 task_support=dict(canonical_spawn=bridge.SPAWN,canonical_gate=bridge.GATE,canonical_goal=bridge.GOAL,
  nominal_reference='Explicit monotonically forward straight nine-metre task reference, independent from common guide.',
  shared_guide='Finite same-endpoint detour polyline accepted, retained as audit_only. No full polyline consumption.',
  actual_existing_guides_checked='Both repeats of N04 static_avoidance and occluded_emergence (4 existing guides) accepted, each arc length>9m.',
  rejected=['curved nominal L/S reference','changed start-goal or spawn offset','changed nominal length/scale','nominal reversal','wrong frame or method identity'],
  gate='New public run/seed/method and all source/activation/deadline/body-twist fields preserved. Only pose_map and declared goal are expressed in canonical route coordinates inside private view.',
  old_node_compatible_without_new_wrapper=False,reason='Frozen old node and SharedLifecycle deliberately hard-code old method/frame checks. A new separately reviewed wrapper/coordinator identity is required; this component does not bypass them.'),
 static_prior=dict(authority='SHA-bound canonical route-frame PGM/YAML, shared across all future methods under declared frame transform',
  raster_rotation_or_resampling=False,world_AABB_input_allowed=False,union='Frozen legacy_static_boxes plus verify_static_union: every blocked/unknown cell exactly once, no overlaps',
  world_output='Oriented four-corner polygons from the exact same canonical rectangles; no AABB expansion.',
  independent_reference='Per-cell independent matrix transforms match transformed rectangle union area and boundary within1e-12; explicit rotated-AABB alternative adds>0.1m2 on fixture.',
  remaining='Future scene/common-map adapter must prove the actual published map/world physical geometry correspond to these canonical bytes. A frame label alone is not evidence.',
  inherited_limitations=['implicit canonical y walls +/-20m unchanged','unknown outside map semantics still differ from common map','shared guide remains audit-only','no full-body or physical safety upgrade to legacy R2']),
 retained_development_failures=[dict(log=bind(D/'tests_r01.log'),passed=21,failed=2,
  cause='Test selected occluded_reveal instead of actual occluded_emergence; independent GEOS symmetric-difference overlay produced4.0246e-10m2 slivers although area delta1.776e-15m2 and Hausdorff1.256e-15m.',
  correction='Correct actual family name; compare independent area and boundary under1e-12, retaining exact raster cell-count test. No runtime/physical threshold changed.')],
 frozen_algorithm=dict(weights=bind(checkpoint),runtime_actor_candidate_normalization_files=frozen,modified=False,
  unchanged_body_twist_and_local_scan=True,numerical_claim='SE2 equivariance to tested floating-point precision, not bitwise full-policy or near-threshold action equivalence.',
  learned_benefit=False),
 qualification=dict(independent_review=False,live_qualified=False,execution_authorized=False,training_steps=0,ros_started=False,new_seeds_registered=0,benchmark_run=False,physical_safety_certified=False),
 resources=resources)
(D/'component_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
(D/'API.md').write_text('''# Frozen R2 SE(2) bridge component

`frozen_r2_route_frame_bridge_n05.py` is a pure component for a future new method, `frozen_r2_v060_sensor_route_se2_n05_v1`. No ROS node or launch is provided. Old R2/lifecycle/checkpoint bytes remain frozen.

Construct `FrozenR2RouteFrameBridge` with explicit `world_from_route=SE2(x,y,yaw)` and `world_from_odom`, the actual world spawn/gate/goal, an explicit `nominal_reference_world`, separate `guide_world`, and the new run/seed identity. The nominal reference must be a rigid copy of spawn(-5.4,0,0), gate(-4,0,0), goal(5,0,0) and the9m forward straight task. A common guide may detour (including existing N04 static/occlusion guides); it remains audit-only and is not consumed by R2. An L/S nominal task or changed nominal distance is rejected.

`SourceMetadata(source_ns, received_source_ns, available_source_ns, receipt_steady_ns)` keeps typed original timing. `OdometrySample`, `TransformSample`, `LaserSample`, `Pose3`, and `BodyTwist` carry only explicit measured values. `odometry(sample,now_ns)` and `laser_transform(transform,scan,now_ns)` return `LegacyPoseView`. A transform must use the exact scan source stamp. `health_transform` is a separate fresh health-only operation; it cannot substitute for exact scan TF. `scan` and `command` return the identical immutable objects, preserving local ranges/angles and body twist. These are consumed-field views, not general ROS message serializers or covariance transformers.

The private alias named `odom` has pose `route_pose - (-5.4,0,0)`. The original `policy_pose` and `_scan` each add spawn_world_x=-5.4, recovering route coordinates. A future in-process TF facade may materialize that private header alias only after validating the real requested frames and source stamp. **Never publish this view as the real `/odom` or a global `odom` TF.** Keep actual raw messages, their frame identity, source and receipt/availability records. Physical commands remain body-frame scalars; no yaw rotation or speed projection is applied.

`gate(event, metadata, now_ns)` preserves the new public identity and source/deadline/activation/body-twist values, expressing only pose and declared goal in canonical route coordinates. The old lifecycle deliberately rejects the new identity; a separate wrapper must validate its own method and preserve the original gate/terminal checks. Retain actual scan append/failure-count verification from sensor-lifecycle r1, six real histories, ordering and clock-reset fault behavior. This pure bridge neither queues TF nor proves observation commit.

`canonical_static_prior(pgm_bytes,yaml_bytes,image_sha256=...,metadata_sha256=...,frame_id='r2_canonical_route')` accepts only SHA-bound canonical map bytes. `legacy_scene_fields()` returns the frozen exact blocked-cell rectangle union; `world_polygons(world_from_route)` returns actual oriented corners for independent scene checking. Do not derive this prior by inverse-rotating world AABBs or by resampling a rotated raster. Future shared-map publication must use the same canonical cells and reviewed rigid frame relation; this component does not qualify that ROS mapping. Implicit legacy y=+/-20m walls and unknown-outside-map differences remain explicit.

23 lightweight tests pass in the existing `.venv`: six rigid transforms with independent real-odom frames, full quaternion tilt, actual frozen consumer/resampler AST, source-age/availability/TF negatives, gate identity/time preservation, nominal-task rejection, four real N04 detour guides, and canonical raster/world polygon checks. No inference, ROS, training, benchmark or seed allocation ran. Independent review and a new bounded integration stage remain required.
''')
for r in resources:assert bind(r['path'])['sha256']==r['sha256']
print(json.dumps(dict(report=bind(D/'component_report.json'),api=bind(D/'API.md'),source=bind(P/'frozen_r2_route_frame_bridge_n05.py'),tests=bind(P/'tests/test_frozen_r2_route_frame_bridge_n05.py')),indent=2))
