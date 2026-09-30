from pathlib import Path
import json,math,hashlib,sys
import numpy as np
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;C=S/'audits/baseline_rear_axle_b02';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import baseline_rear_axle_adapter_b02 as b
def binding(p):p=Path(p);return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
a=json.loads((C/'implementation_report.json').read_bytes());bindings=[]
for expected in a['bindings']+[a['source_addendum']]:
    got=binding(expected['path']);assert got['sha256']==expected['sha256'],expected['path'];bindings.append(got)
source=next(x for x in bindings if x['path'].endswith('/baseline_rear_axle_adapter_b02.py'));assert source['sha256']=='13b72ac9f159d68f3e2f58cab185b5d6620e936d723b03c5946e712d0e819a92'
assert '32 passed' in (A/'author_tests_replay.log').read_text() and '17 passed' in (A/'independent_tests_r01.log').read_text()
pitch=.05;phi=.2;delta=.325*np.array([math.cos(phi)-1,math.sin(phi)]);rear_xy=np.array([4.675,0.])+.24*delta/np.linalg.norm(delta);actual_base=b.planar_pose_to_base((*rear_xy,phi));goal_error=math.dist(actual_base[:2],(5.,0.))
checks={
'16_author_and_diagnostic_bindings_match':True,
'author32_and_independent17_tests_pass':True,
'complete_rigid_pose_point_twist_reference_math':True,
'actual_rear_lateral_residual_not_zeroed':True,
'6x6_parent_smallrotation_pose_covariance_Jacobian_finite_difference':True,
'6x6_body_twist_covariance_correlations_preserved':True,
'wrong_covariance_conventions_nonfinite_nonPSD_rejected':True,
'actual16_64_64_footprints_exact_in_planar_case':True,
'pitch_nonzero_2D_worldpolygon_difference_retained':True,
'source_receipt_availability_preserved_and_future_use_rejected':True,
'route_all_oriented_poses_goal_and_GetPlan_semantics_roundtrip':True,
'orientation_kink_rear_position_jump_not_smoothed_or_qualified':True,
'same_radius_rear_goal_not_false_base_success':True,
'no_ROS_runtime_config_currentN04_changes':True}
r=dict(schema='astra_baseline_rear_axle_b02_independent_review_v1',status='independent_B02_pure_reference_component_review_pass_integration_pending',independent_review=True,reviewed_source_sha256=source['sha256'],checks=checks,check_count=len(checks),author_tests_passed=32,independent_tests_passed=17,
 conclusion='No blocking defect found in the bounded pure reference conversion component. Complete rigid point/twist and declared covariance math pass independent checks. Two-dimensional costmap tilt equivalence and planner-internal stopping remain unqualified, material integration conditions; no live readiness is inferred.',
 verified_math={'offset_body_m':[-.325,0,0],'pose':'p_R=p_B+R_B r; same orientation','twist':'v_R^B=v_B^B+omega^B cross r; angular unchanged','pose_covariance':'First-order J=[I,-skew(Rr);0,I] for parent-axis position/fixed-axis small-rotation perturbations only. Independently differentiated with Rodrigues rotation matrices.','twist_covariance':'Exact linear J=[I,-skew(r);0,I] for body-axis linear/angular velocity; independent correlated noise-column crossproduct propagation.','footprint':'All existing16/64/64 vertices translated x+.325 without hull/padding/omission. Exact sameworldpoint inSE2. Full3D points andsensorlocations exact underfulltilt.'},
 retained_counterexamples={'pitch_rad':pitch,'yaw_only_costmap_shift_m':.325*(1-math.cos(pitch)),'rear_goal_xy_error_m':.24,'rear_goal_yaw_error_rad':phi,'resulting_actual_base_goal_error_m':goal_error,'base_gate_xy_tolerance_m':.25,'interpretation':'These are synthetic mathcounterexamples, notsimulation. Do not erase withrounding ordeclarefullplanarpolygon equivalence underactualpitch. Do notchange originalgoal gate.'},
 boundaries=['Pure adapt_odometry checks availability, not freshness/order/clockreset: stale sample remainsstale andsource unchanged. Caller mustretain unchanged200ms/no-zero/exactTF sourcehealth semantics.','geometry.Pose3 normalizes inputquaternions upstream. Future raw-message adapter mustvalidate rawmeasuredquaternion/finite pose beforeconstructing, retaining originalraw; this component pass is notinputintegrityproof.','Covariance coordinateconvention mustmatch actualsource provenance; a token/default alone is notmeasurement qualification. Physical P05 allzero covariance isunreported uncertainty, notexactstate confidence.','Samephysicalorientedroute configurations do notimply identicalchord interpolation, pathcosts orAckermannfeasibility; headingkinks move rearXY and remainvisible.','Changing odom_topic alone or frame alone isincomplete; changing2Dfootprint alone undertilt cannotprove sameworldgeometry.','Source ageandfull3D covariance arepreserved butpure component doesnotprovide ROS serialization, sensorgraph, TFownership oractualplannerintegration.'],
 required_future_integration=['Newmethodidentity andnewrelease; keep currentN04, oldB01coarseconfig/algorithms, P05physicsdriver and originalresults unchanged.','One staticbase_link->rear_axle_plan_b02 owner, noadditionalodom->rear parent; preserve originalauthority odom->base andsharedcoordinator readingbase.','Both TEB/DWA actual odom_topic andboth costmap robot_base_frame usealignedrear convention; actual sensorTF/scan mustremainunchanged sourcegeometry.','Translate entireexistingfootprints with unchanged.33semantics, then independentlyquantify existingtilt-envelope coverage versus actual2Dcostmapshift; noquietpadding/marginrelax.','Routeprovider shim inverseconverts bothstart andgoal withownyaw, andconverts allreturnedorientedposes; preserve fixedbaseguide andexplicitfinalyaw. Checkactualserved/global/localpaths.','Retain authoritativebasegoal predicate. Explicitly designandqualify plannerinternalgoal stopping so rearball doesnotstopoutsidebaseball; noautomatic adoption oftheoreticalsmaller radius.','Fresh boundedturning/transient qualification records base/rear actualstate andfullcovariance, sourcejoin, DWA sampling/output, TEB reference andactualexecution; noacc_yincrease orclaimreferenceadaptation aloneenforcescarcurvature/slew.'],
 runtime_ready=False,live_qualified=False,execution_authorized=False,simulation_episodes=0,training_steps=0,
 commands=['workspace/.venv/bin/python -m pytest -q -p no:cacheprovider workspace/scripts/astra_gazebo/tests/test_baseline_rear_axle_adapter_b02.py','workspace/.venv/bin/python -m pytest -q -p no:cacheprovider audits/baseline_rear_axle_b02_independent/test_b02_independent.py'],
 resources=bindings+[binding(C/'implementation_report.json'),binding(A/'author_tests_replay.log'),binding(A/'independent_tests_r01.log'),binding(A/'test_b02_independent.py'),binding(Path(__file__))])
(A/'review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
(A/'review.md').write_text('B02 纯参考点组件独立复核通过；运行期集成仍未资格。\n\n作者32项、独立17项全部通过，16个作者与诊断绑定SHA均复核一致。刚体位置、六分量body twist、实际rear横向残差、完整相关协方差的传播正确。pose协方差仅适用于明确的parent固定轴小旋转误差，一阶传播不冒充任意Euler或有限非线性分布的精确传播。\n\n实际16/64/64个footprint顶点在SE2下世界集合相等，完整3D点在tilt下也相等；但pitch=0.05时，rear TF加yaw-only costmap投影相对原base投影仍偏移 %.12f m。这个非零差别没有抹平。rear位置误差0.24 m、yaw误差0.20 rad的合法rear圈内点，可对应base误差%.12f m，大于原0.25 m目标门。原共享base判定必须保持，局部planner提前停止问题需新设计与资格。\n\n逐点oriented route与GetPlan请求已验证可逆，末项yaw保留；朝向折点导致rear位置跳变，组件不声称平滑或Ackermann可达。时间可用性边界正确；stale记录保留原时间，组件并不代替调用方原freshness、clock-reset与exact-TF门。\n\n未来接线须同时处理TF唯一owner、两基线odom_topic、两costmap参考点、完整footprint、source/covariance约定、服务请求与返回path、共享basegoal和实际tilt几何。geometry.Pose3会归一化四元数，raw消息在构造前仍须按现协议验证并保留原始证据。没有ROS、native build、large replay、benchmark、新种子或旧配置修改。\n'%(r['retained_counterexamples']['yaw_only_costmap_shift_m'],goal_error),encoding='utf-8')
print(json.dumps(dict(status=r['status'],report=binding(A/'review.json'),markdown=binding(A/'review.md'),counterexamples=r['retained_counterexamples']),indent=2))
