from pathlib import Path
import hashlib,json
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;C=S/'audits/r2_route_frame_bridge_n05';P=W/'scripts/astra_gazebo'
def bind(p):p=Path(p);return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size)
author=json.loads((C/'component_report.json').read_bytes());checked=[]
for row in author['resources']:
    got=bind(row['path']);assert got['sha256']==row['sha256'],row['path'];checked.append(got)
assert author['source']['sha256']=='9a4d3f21929964d50b814748bba8b7ac484f07b5b59099290ceecacbbdb3408e'
assert '23 passed' in (A/'author_tests_replay.log').read_text() and '27 passed' in (A/'independent_tests_r02.log').read_text()
checks={
'author16_resources_rehashed_and_legacy_algorithm_checkpoint_unchanged':True,
'author23_tests_replayed':True,
'independent27_actual_code_counterexamples_pass':True,
'private_x_plus5p4_exactly_compensates_both_legacy_consumers':True,
'actual_legacy_policy_pose_scan_resampler_executed_without_ROS_import':True,
'complete_quaternion_left_rotation_preserves_tilt_and_z':True,
'body_twist_scan_and_command_preserved_not_world_rotated':True,
'source_int64_age_availability_and_exact_scan_TF_rejection':True,
'canonical_raster_blocked_unknown_cell_exact_once_and_oriented_corners':True,
'same_task_SE2_only_and_detour_guide_audit_only':True,
'gate_identity_source_deadline_bodytwist_preserved':True,
'old_lifecycle_rejects_new_method_not_silently_bypassed':True,
'actual_second_TF_failure_preserves_no_commit_requires_wrapper_guard':True,
'no_live_or_fullpolicy_or_fullpolyline_qualification_claim':True}
report=dict(schema='astra_r2_route_frame_bridge_n05_independent_review_v1',status='independent_pure_R2_SE2_bridge_component_review_pass_integration_pending',independent_review=True,reviewed_source_sha256=author['source']['sha256'],method_identity=author['method_identity'],check_count=len(checks),checks=checks,author_tests_passed=23,independent_tests_passed=27,
 conclusion='Pure consumed-field SE2 bridge is consistent with the frozen legacy policy_pose/_scan call sites on reviewed fixtures. No blocking source defect found in this bounded component review. A new wrapper/coordinator integration and fresh bounded engineering qualification remain prerequisites; component pass is not runtime readiness.',
 source_findings=[
 {'file':'pct_teb_dwa_benchmark_episode.py','lines':[171,204,232,240],'finding':'Actual old _scan requests odom<-laser_link at original scan header stamp, resamples original local ranges, adds spawn_world_x toTFx and appends original source seconds. policy_pose also adds spawn_world_x. With frozen -5.4, both private views must be route_x+5.4.'},
 {'file':'frozen_r2_route_frame_bridge_n05.py','function':'Pose3.transformed/_route_pose/_private_view','finding':'Left multiply yawquaternion, retain z/rollpitch content. Independent real world_from_odom composition works even when odom androute frames have differentyaw/origin. Fullbodylinear/angular twist unchanged because it is expressed inrobotbody axes.'},
 {'file':'frozen_r2_route_frame_bridge_n05.py','function':'SourceMetadata.require/laser_transform','finding':'Decision must be at/afteravailability; sourceage<=200ms, receipt/sourcelead<=200ms; exact typedint64 andexact sourceTF equality. Originalmetadata objects retained. SeparatehealthTF cannotreplace exactscanTF.'},
 {'file':'frozen_r2_route_frame_bridge_n05.py','function':'canonical_static_prior/CanonicalStaticPrior.world_polygons','finding':'Factory rehashes both canonicalPGM/YAML, reuses frozen rectangle extraction andexactraster verifier. Independent asymmetric9x13map confirms blockedandunknown cells exactlyonce, nofreecells; orientedcorners match independentmatrix. No worldAABB input/rotation-resampling.'},
 {'file':'frozen_r2_route_frame_bridge_n05.py','function':'__init__/gate','finding':'Rejectcurved/reversed/rescaled nominaltask; permit sameendpointdetour onlyas audit. Newidentity andnongeometricgatefields preserved; inheritedactivation5..5.2s,deadline22s tolerances unchanged. OldSharedLifecycle rejectsnewmethod, asrequired.'}],
 wrapper_integration_requirements=[
 'Create separatelyidentifiedwrapper/coordinator allowedmethod; do not monkey-patchold whitelist or mislabel newmethod as oldR2. Bind all fourlegacyfiles andexactcheckpoint.',
 'Use privateinprocess TFfacade only: validateactual real requestedframe/source, retainrawodometry andrawscan identities, materialize privateodom alias afterauth. Neverpublish transformedprivateview onreal /odom/TF.',
 'Derive world_from_odom andworld_from_route fromboundscene/commonlocalization semantics; no actor/model-state truth subscriptions. Actualgraph/rawTF anddynamicstatic-map evidence required; passingcoordinates alone is notsensor-only proof.',
 'Retain SensorIngress sourceorder/clock-reset faults, actualavailability queues and200msfreshness. Nozero-stamp fallback, nofuturebackdating, no synthesized sixhistory frames.',
 'Retain lifecycle r1 last_runtime_scan_stamp andfailurecount append proof. Actualold _scan catchessecondTFerror and leavesoldhistory unchanged; firstTFprecheck alone cannot declarecommittedscan.',
 'Mapconstructor must invoke authenticated canonical_static_prior factory onboundbytes, not reconstruct CanonicalStaticPrior fromuntrustedboxes; showpublishedcommonmap andphysicalworldgeometry agree withsamecanonicalcells+SE2. A frame label alone doesnotprove this.',
 'Preserve actualsharedgate/activation/deadline andfirstterminal outcome; terminalcommands publiczero whileoldpolicyprivate statecannotoverride coordinator. Originalsource-time floatseconds usedinsidelegacy algorithm areunchanged, notclaimed losslessint64 policyinternal time.',
 'Privateviews are consumed-field records, notgeneral ROS serializers orcovariance transforms; futurefacade mustnotsilently reinterpret unrepresentedmessage fields. Preservebodytwist/localrange invarianceandfullorientation, withoutclaimingnew3Dscan tiltcorrection.',
 'Retainno-full-polyline statement fordetourguide, implicitcanonical y±20walls, unknown-outside-map mismatch andlegacyfootprint/physicalscope. No full-policy bitwiseequivariance ornear-thresholdaction identity claim withoutnew evidence.'
 ],
 limits=['No ROS, Gazebo, inference, training, largebag, benchmark or newidentityallocation performed.','Purecomponent hasnoeventqueue/clockregressionstate; integrationmustprovide thosefrozen lifecycle guarantees.','No deployment, physicalsafety, generalizedroute,learnedbenefit orliveperformance qualification.','Authorchecks include4existingN04 detourguides; this is inputgeometry only, notN04outcomes.'],
 retained_failures=[dict(log=bind(A/'independent_tests_r01.log'),source_snapshot=bind(A/'test_bridge_independent_r01_snapshot.py'),result='26pass1failed',cause='Independent firstfixture usedGEOS Hausdorff onfloating differentlypartitioned rotatedcell/rectangle unions. Area difference6.661338147750939e-16m2; internal sliverboundary Hausdorff0.025m. Thismeasure is notan exactunion-equivalence proof.',repair='Kept exactintegercellcoverage assertion andcompared everyorientedrectanglecorner withindependent rigidmatrix(<1e-14), plusareaagreement. Noauthorsource orphysicalthreshold changed.'),dict(kind='runtimewarning',log=bind(A/'independent_tests_r02.log'),detail='numpy.ufunc size changed binarycompatibility warning preserved; all27 fixtures execute. No suppressions.')],
 resources=checked+[bind(C/'component_report.json'),bind(C/'API.md'),bind(A/'author_tests_replay.log'),bind(A/'independent_tests_r01.log'),bind(A/'independent_tests_r02.log'),bind(A/'test_bridge_independent.py'),bind(A/'test_bridge_independent_r01_snapshot.py'),bind(Path(__file__))],
 commands=['workspace/.venv/bin/python -m pytest -q -p no:cacheprovider workspace/scripts/astra_gazebo/tests/test_frozen_r2_route_frame_bridge_n05.py','workspace/.venv/bin/python -m pytest -q -p no:cacheprovider audits/r2_route_frame_bridge_n05_independent/test_bridge_independent.py'],runtime_ready=False,execution_authorized=False)
(A/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(A/'review.md').write_text('R2 SE(2) 纯组件独立复核通过，runtime integration 仍待资格。\n\n作者 23 项复跑通过，独立 27 项实际代码反例通过；16 个作者绑定资源当前 SHA 全匹配，旧算法与 checkpoint 未改。未发现本次有界组件审查中的阻断源码缺陷。\n\n实际执行了旧 policy_pose、_scan 与 resample_scan 的 AST 方法体。两处旧 x+spawn_world_x 都要求私有 x=route_x+5.4；不同里程计坐标系、独立激光外参、完整倾斜四元数均恢复相同 canonical 输入。体坐标速度、局部扫描与原始源时间保留。availability、200 ms 边界、TF源时差1 ns、错误frame、源时类型与gate身份反例均按预期拒绝。\n\nCanonical PGM 的不对称 9×13 栅格经过独立逐cell计数，未知及阻塞格恰好覆盖一次；矩形四角经独立刚体矩阵验证。首轮 GEOS 不同浮点分区的 Hausdorff 反例和 NumPy warning 原样保留，没有修改作者源码或物理门。\n\n只支持原直线名义任务的刚体变换。detour guide 仍是 audit_only，不进入完整路径决策；新method被旧SharedLifecycle拒绝。组件既没有运行期队列，也未提供ROS节点，所以未来wrapper必须保留真实availability与单调源时间、六帧历史、二次TF失败后的实际append证明、首终态锁存和共享zero命令。不得把私有odom别名发布到真实TF，也不得通过重命名绕过旧身份校验。\n\n地图须通过 authenticated factory 读取同一绑定canonical字节，再证明实际发布地图和物理场景的对应关系；构造一个frame标签不足以证明这一点。仍保留±20 m隐式墙、地图外未知语义差异、旧安全范围以及不保证整策略逐位等价等限制。完整未来接线要求、精确hash和命令见 review.json。\n',encoding='utf-8')
print(json.dumps(dict(status=report['status'],report=bind(A/'review.json'),markdown=bind(A/'review.md'),checks=len(checks)),ensure_ascii=False,indent=2))
