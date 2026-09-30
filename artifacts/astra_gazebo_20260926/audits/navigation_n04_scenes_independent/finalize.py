#!/usr/bin/env python3
"""Freeze independent offline N04 scene review from completed calculations."""
import hashlib,json
from pathlib import Path
O=Path(__file__).resolve().parent;S=O.parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
a=json.loads((O/'recompute_result.json').read_text());b=json.loads((O/'closure_result.json').read_text())
assert a['status']=='independent_static_inputs_and_witness_recompute_pass' and b['status']=='independent_exact_scene_and_split_closure_pass'
checks={
 'twelve_rows_six_old_development_parents_exact_original_csv_order':True,
 'fresh_proposed_seed_263001001_plus_index_no_registration_by_review':b['proposed_seeds_absent_from_registry_snapshot'],
 'all_24_supplied_parent_ids_geometry_hashes_and_split_buckets_recomputed':len(a['split_audit'])==24,
 'all_10_actor_parameter_cells_recomputed_development_only':len(b['parameter_cells'])==10,
 'supplied_parent_registry_no_near_duplicates':not b['within_supplied_layouts_near_duplicate_pairs'],
 'full_world_geometry_and_non_gate_actor_parameters_preserved':all(x['inheritance'] for x in a['rows']),
 'replicates_have_same_static_geometry_actor_paths_and_shared_map_route':b['all_twelve_replica_static_and_actor_path_geometries_identical_within_parent'],
 'per_run_identity_binding_changes_only_expected_fields':a['identity_positive_cases']==60,
 'invalid_and_duplicate_actor_identity_rejections':a['identity_negative_cases']+b['bad_identity_rejections']==132,
 'seven_actual_converted_bodies_nine_primitives_independent_fk':len(a['robot_radius_audit'])==9,
 'all_48_prepared_converted_physical_models_equal_reviewed_reference':b['runtime_prepared_physical_model_equal_reference_count']==48,
 'all_initial_robot_and_actor_shapes_clear_of_static_and_each_other':b['all_actor_initial_shapes_disjoint'],
 'all_actor_complete_translation_hulls_clear_of_static':all(not x['intersection'] for g in a['geometry'].values() for x in g['actor_sweeps']),
 'actual_actors_absent_from_static_projection_and_export_replays_exactly':all(x['actors_absent_from_static_prior'] for x in a['rows']),
 'legacy_rectangle_prior_matches_every_blocked_static_raster_cell':all(x['blocked_raster_equals_rectangles'] for x in a['rows']),
 'every_family_has_ideal_static_ackermann_witness_after_separate_supplement':a['all_six_families_have_ideal_static_witness'],
 'original_occlusion_sin4_insufficient_margin_negative_preserved':a['old_occlusion_sin4_negative_retained'],
 'quintic_sampling_interval_pad_and_ackermann_bound_independently_recomputed':a['geometry']['occluded_emergence']['supplement_quintic']['static_pass'],
 'occlusion_two_static_ray_poses_recomputed':a['geometry']['occluded_emergence']['ray_visibility']['gate']['visible_count']==1 and a['geometry']['occluded_emergence']['ray_visibility']['reveal']['visible_count']==60,
 'nominal_nine_meters_distinguished_from_measured_common_guide_arc_length':True,
 'twelve_four_method_paired_blocks_and_balanced_order_design_only':b['paired_four_method_blocks']==12,
}
assert all(checks.values())
limits=[
 'Offline static feasibility only. No new ROS/Gazebo run, physical tracking, moving-actor avoidance, contact, scan visibility timing or navigation qualification follows.',
 '12 development input rows are six previously generated parent geometries with two parameter/seed realizations each; they are not 12 independent geometries or a final-test sample.',
 'Split proof covers 24 supplied historical development layouts, six selected parents and ten inherited actor parameter cells. No materialized train/validation/final population was available here; global held-out isolation and full design sample sizes are not certified.',
 'New simulator seeds leave frozen actor policy parameters and response seeds unchanged. The inherited actor is deterministic with zero response jitter; seed labels do not establish independent stochastic behavior.',
 'The static sin4/quintic witnesses are ideal rear-axle bicycle curves, not the common guide polylines, executed trajectories, or time-parameterized actuator-feasible tracking plans.',
 'The common guide is consumed by route-aware methods; frozen R2 still treats the common path as audit-only and retains its global-x route/action semantics. Identical raster rectangles do not prove identical outside-map behavior or method-wide path consumption.',
 'XY conservative projections and analytic Z bounds are not a 3D contact proof. Converted wheel zero-pose lower Z is about -1.47e-7m from six-digit SDF roll; this audit does not requalify wheel contact or floor support.',
 'Cylinder projections use outward 48-gons. Body/root radius and front steering local radius bounds are checked from the actual converted model; the wheel-axis quantization pad is below the retained1e-5m interval pad.',
 'At actor-free templates the identity binder has no expected_* fields to mutate; run identity must still be bound by the separately reviewed runner/contract.',
 'The old geometry_qc.json sin4 failure and old preparation report remain unchanged. The quintic is separately bound supplementary evidence, not a reclassification or overwrite of that negative.',
 'Occlusion visibility is one initial actor pose evaluated from two static observer poses:1/30 and60/60 actor-hit beams. Temporal emergence and all moving interactions remain live gates.',
 'The per-run48 converted physical-model comparison is exact XML geometry/inertia/joint equality; final runtime conversion authentication, loaded ELF closure and actual model metadata remain collector responsibilities.',
]
commands=[
 '/usr/bin/python3 -u '+str(O/'recompute.py')+' > '+str(O/'recompute_r01.log')+' 2>&1',
 '/usr/bin/python3 -u '+str(O/'closure_checks.py')+' > '+str(O/'closure_checks_r01.log')+' 2>&1',
 '/usr/bin/python3 '+str(O/'finalize.py'),
]
(O/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
errors={'execution_test_errors':[], 'initial_read_only_path_discovery_errors':[
 {'operation':'rg requested audit directory before creation','result':'directory absent; then created only reviewer-owned audit directory'},
 {'operation':'sed .../navigation_n04_development_r01/reference_robot.sdf','result':'wrong guessed path absent; actual reference_robot/converted_robot.sdf read'},
 {'operation':'cat .../common_bundle/route.json','result':'wrong guessed filename absent; actual common_route.json read'}],
 'effect':'No source or historical result mutation. Both executable independent audit scripts passed on first invocation; original logs preserved.'}
(O/'tool_errors.json').write_text(json.dumps(errors,indent=2)+'\n')
table=[]
for f,g in a['geometry'].items():
    w=g.get('supplement_quintic',g['original_sin4']);v=w['interval_minus_033_lower_m'];value='无静态障碍' if v is None else '%.9f'%v
    table.append('|%s|%.9f|%s|%s|'%(f,g['actual_common_guide_length_m'],w['curve'],value))
md='''N04 场景独立离线复核通过；没有启动 ROS、Gazebo、登记种子或改变旧证据。

直接从实际转换 SDF 解析7个刚体、9个碰撞部件，独立使用4×4矩阵运动学、盒体8角点和外接48边形圆柱投影；未调用作者的 ReferenceRobot / project_box / project_cylinder 来得到见证数值。对每类901个采样点重算净空，再使用全区间导数上界减去半步位移及1e-5米轮轴量化余量。48份已准备的实际转换 SDF 碰撞、惯性和关节结构全部与参考一致。

|几何族|实际共享guide长度/米|采用的独立静态见证|区间物理净空下界−0.33米|
|---|---:|---|---:|
'''+ '\n'.join(table)+'''

所有场景的名义起终点纵向距离仍为9米。共享guide与见证曲线各自不同，不能将这张表理解为实际行驶轨迹、时间或跟踪性能。

遮挡场景原sin⁴见证的采样物理净空0.241362796米，扣区间余量及0.33米后为−0.117345710米；原失败文件保持不变。独立重算的新quintic为采样0.490048601米、区间下界0.479083801米、扣0.33米后0.149083801米，中心转角上界0.288257126rad，小于0.4964rad。该曲线端点的横向位移、航向、曲率均为零；中点两段连接C²，分段三阶导数界足以支持所用Lipschitz余量。九部件半径界≤1米、前轮转向局部半径≤0.3米，量化轮轴的旋转形状偏差约1.14047e-6米，低于所扣1e-5米。

所有actor起点到终点的恒定朝向平移凸包均与static不相交。有限最小净空分别为head-on1.094735920米、遮挡0.241650000米；crossing和sequential的静态障碍集合为空，结果以null表达，未编造大净空数。机器人spawn/gate/goal九部件与static/初始actor、actor彼此初始形状均不相交。遮挡静态ray复算得到gate可见1/30束，另一静态位置60/60束；不能据此声称真实动态可见时序已通过。

12行严格继承旧CSV顺序、6个旧parent、原碰撞几何与actor路径/策略参数/response_seed。新种子为263001001+原index。24个已提供旧development parent的canonical ID、精确geometry SHA与split桶独立重算，10个actor参数cell同属development，24个parent的近重复检查无冲突。这里没有已物化train/validation/final总集合，故不声称全局held-out隔离证明或完整样本量；6个旧parent和每组两次输入变体不能记作12个独立几何。

调用实际纯identity函数完成60个正常绑定和132个拒绝反例；全XML比较证明正常绑定只改变expected_run_id/expected_seed/expected_method。显式剔除所有declared actor后静态输入XML与保存内容相等，PGM/YAML回放字节相等；另以独立整数cell栅格覆盖逐像素证明R2矩形输入与所有非free栅格一致。每组两次输入的静态图和共享guide也字节相等。旧R2仍不消费共享guide，静态栅格相等不代表全部方法的路线或图外语义等价。

尚无本审计产生的运行授权。12×4配对设计与顺序平衡复核通过，种子未出现于保存的registry快照；这里只读快照，不修改全局注册表。旧碰撞/物理资格与未来actor共同gate运行资格均应使用各自正式证据。

复现命令见commands.json；实际结果见recompute_result.json、closure_result.json及7份逐点CSV。两脚本首次执行通过，纯路径定位错误另存tool_errors.json。输入在脚本结束及封存时均重新计算SHA。
'''
(O/'review.md').write_text(md)
resources={}
for z in (a,b):
    for item in z['source_resources']:
        p=item['path'];h=item['sha256'];assert sha(p)==h,(p,'changed');assert p not in resources or resources[p]==h;resources[p]=h
for p in sorted(O.iterdir()):
    if p.is_file() and p.name not in ('review.json',):resources[str(p.resolve())]=sha(p)
report={'schema':'astra_navigation_n04_scenes_independent_review_v1','status':'independent_N04_static_scene_and_split_review_pass','independent_review':True,'pass':True,'scene_preparation_source_sha256':sha(S/'workspace/scripts/astra_gazebo/prepare_navigation_n04_scenes.py'),'input_rows':12,'distinct_preexisting_parent_geometries':6,'converted_robot_body_count':7,'converted_collision_count':9,'prepared_conversion_model_comparisons':48,'check_count':len(checks),'checks':checks,'identity_positive_cases':60,'identity_negative_cases':132,'static_geometry_results':a['geometry'],'limitations':limits,'global_final_split_qualification':False,'static_feasibility_only':True,'live_actor_qualification':False,'runtime_tracking_qualification':False,'execution_authorized':False,'new_simulation_or_registration':False,'frozen_historical_sources_unchanged':True,'source_resources':[{'path':p,'sha256':h}for p,h in sorted(resources.items())]}
(O/'review.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
assert all(sha(x['path'])==x['sha256'] for x in report['source_resources'])
print(json.dumps({'status':report['status'],'checks':len(checks),'resources':len(resources),'report_sha256':sha(O/'review.json'),'md_sha256':sha(O/'review.md')},indent=2))
