#!/usr/bin/env python3
"""Bind source findings and the reviewable B01 proposal; no runtime checks."""
import csv,hashlib,json,re,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
O=Path(__file__).resolve().parent;S=O.parents[1];W=S/'workspace';C=W/'src/simulation/m2_gazebo/config';T=W/'src/navigation_arena/forks/navigation/local_planner/teb';D=S/'audits/navigation_n01_dwa_diagnosis/upstream_navigation_1.17.3';R=Path('/opt/ros/noetic')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def binding(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def locations(p,needle):
 return [dict(path=str(p),sha256=sha(p),line=i,text=line.strip())for i,line in enumerate(p.read_text().splitlines(),1)if needle in line]
m=json.loads((O/'candidate_manifest.json').read_text());d=yaml.safe_load((O/'design.yaml').read_text());resources={x['path']:x['sha256']for x in m['source_resources']}
for row in m['candidates']:assert sha(row['config_path'])==row['config_sha256'];resources[row['config_path']]=row['config_sha256']
evidence={
 'footprint_load_after_planner':locations(W/'src/simulation/m2_gazebo/launch/astra_common_navigation_n02.launch','rosparam'),
 'shared_footprint_overrides_min_obstacle_dist':locations(C/'astra_navigation_common_footprint.yaml','min_obstacle_dist:'),
 'DWA_stop_time_member_assignment':locations(D/'dwa_local_planner/src/dwa_planner.cpp','stop_time_buffer_'),
 'DWA_stop_time_member_declaration':locations(R/'include/dwa_local_planner/dwa_planner.h','stop_time_buffer_'),
 'TEB_converter_rate_default':locations(T/'include/teb_local_planner/teb_config.h','costmap_converter_rate'),
 'TEB_converter_worker':locations(T/'src/teb_local_planner_ros.cpp','startWorker'),
 'TEB_rate_readers_in_config_cpp':locations(T/'src/teb_config.cpp','costmap_converter_rate'),
 'TEB_lookahead_radial_cutoff':locations(T/'src/teb_local_planner_ros.cpp','dist_threshold'),
 'DWA_path_goal_resolution_scales':locations(D/'dwa_local_planner/src/dwa_planner.cpp','resolution *'),
 'DWA_forward_point_distance_consumers':locations(D/'dwa_local_planner/src/dwa_planner.cpp','forward_point_distance_'),
 'TEB_plugin_fallback':locations(T/'src/teb_local_planner_ros.cpp','cannot be loaded'),
 'TEB_converter_replaces_point_path':locations(T/'src/teb_local_planner_ros.cpp','updateObstacleContainerWithCostmap'),
 'TEB_converter_setCostmap_and_odom':locations(T/'src/teb_local_planner_ros.cpp','setCostmap2D')+locations(T/'src/teb_local_planner_ros.cpp','setOdomTopic'),
}
libs=[]
for p,label in [(W/'devel/lib/libteb_local_planner.so','workspaceTEB_source0.8.4'),(R/'lib/libdwa_local_planner.so','installedDWA1.17.3'),(R/'lib/libcostmap_converter.so','installedconverter0.0.13')]:
 out=subprocess.check_output(['readelf','-n',str(p)],text=True);build=re.search(r'Build ID: ([a-f0-9]+)',out).group(1)
 log=O/(label+'_readelf.txt');log.write_text(out);b=binding(p);b.update(role=label,build_id=build,scope='installed/current file identity only; future actual-process mapping required');libs.append(b);resources[str(p.resolve())]=sha(p)
evidence['ELF_files']=libs
(O/'source_effect_findings.json').write_text(json.dumps(evidence,indent=2)+'\n')
converted=S/'scenes/navigation_n04_development_r01/reference_robot/converted_robot.sdf';model=ET.fromstring(converted.read_bytes()).find('model');plugin=next(p for p in model.findall('plugin')if p.get('filename')=='libastra_physical_ackermann_p05_plugin.so')
actual={k:float(plugin.findtext(k))for k in ('wheelbase','track','wheelRadius','maxSpeed','maxSteer','maxAcceleration','maxDeceleration','maxSteeringRate','commandTimeout','publishRate','commandDelay','commandJitter','wheelDriveTorqueLimit','wheelBrakeTorqueLimit','steeringTorqueLimit','steeringKp','steeringKd')}
assert actual['steeringKd']==6 and actual['maxSpeed']==2.778 and actual['maxSteer']==.4964 and actual['maxAcceleration']==.6 and actual['maxDeceleration']==.9 and actual['maxSteeringRate']==.75
checks={'12_coarse_each_with_current_effective_anchor':m['candidate_counts']=={'teb':12,'dwa':12},'24_configs_exact_current_hash':True,'12_source_confirmed_active_fingerprints_each':all(len({x['confirmed_active_fingerprint_sha256']for x in m['candidates']if x['planner']==p})==12 for p in ('teb','dwa')),'all_contract_levels_included':True,'all144_cfg_axis_types_ranges_checked':m['schema_axis_value_checks']==144,'byte_identical_regeneration_pass':json.loads((O/'regeneration_check_r01.log').read_text().splitlines()[-1])['status']=='deterministic_regeneration_and_source_rehash_pass','fixed_speed_request_caps_compound_footprint_and_kinematics':True,'actual_reference_P05_parameters_preserved':True,'source_inert_or_censored_axes_not_counted_as_unique':True,'converter5Hz_dt_point2_not_false10Hz_claim':True,'converter_component_and_load_order_not_qualified_by_YAML':not m['runtime_ready'],'dev_validation_split_and_equal_budget_proposal':d['equal_budget_proposal_not_authorization']['validation']['paired_blocks']==216,'all_failures_included_no_baseline_suppression':True,'no_seeds_release_ROS_benchmark_or_final_access':not m['seeds_allocated']and m['authorized_episodes']==0 and not m['simulator_started']and not m['final_outcomes_read']}
commands={'generation':'/usr/bin/python3 '+str(O/'generate.py'),'reproduction':'/usr/bin/python3 '+str(O/'generate.py')+' --check-only','report':'/usr/bin/python3 '+str(O/'finish_report.py'),'read_only_package_query':"dpkg-query -W -f='${Package} ${Version}\\n' ros-noetic-teb-local-planner ros-noetic-dwa-local-planner ros-noetic-costmap-converter",'read_only_ELF':['readelf -n '+x['path']for x in libs]}
(O/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
errors={'generator_attempts':[{'log':'generate_r01.log','source':'generate_quota_deadend_r01.py','result':'Strict marginal quotas combined with effective-unique greedy selection exhausted the candidate pool before any config write.'},{'log':'generate_r02.log','source':'generate_uncovered_level_r02.py','result':'Unconstrained maximin omitted interior levels; coverage assertion failed before any config write.'},{'log':'generate_r03.log','source':'generate.py','result':'Explicit level-coverage priority followed by maximin passed; first actual24config materialization.'}],'other_warnings':['Initial read-only discovery guessed absent N04 launch / spawn/header paths; actual paths located and bound.','Initial source search missed ignored nestedTEB files; --hidden --no-ignore located actualworkspace0.8.4.','System Python emitted an existing requests/urllib3 compatibility warning in initial generator logs; generator uses no HTTP requests.'],'no_experiment_failure_or_retry':True,'historical_results_unchanged':True}
(O/'error_history.json').write_text(json.dumps(errors,indent=2)+'\n')
tables=[]
for planner,title in [('teb','TEB'),('dwa','DWA')]:
 rows=list(csv.DictReader((O/(planner+'_coarse.csv')).open()));cols=[k for k in rows[0]if k not in ('anchor','confirmed_active_fingerprint_sha256','config_sha256')]
 tables.append(title+'参数表（00为当前有效anchor）：\n\n|'+'|'.join(cols)+'|\n|'+'|'.join('---'for _ in cols)+'|\n'+'\n'.join('|'+'|'.join(r[k]for k in cols)+'|'for r in rows))
md='''B01初始基线调参方案已生成，共24份新配置：workspace TEB0.8.4和安装版DWA1.17.3各12份，分别包含当前有效anchor。它们只是一份可审查设计，没有分配种子、启动ROS/Gazebo或读取final结果；也没有修改当前N04或任何已有配置。

生成器遍历合同有限网格，先保证每轴层级被覆盖，再对源码确认生效的轴做确定性maximin选择。每方12个有效参数向量均不同；TEB lookahead和DWA stop_time_buffer不参与有效唯一性判断。144项cfg类型/范围检查及逐字重新生成通过。该矩阵偏向边界探索，不是正交表或均衡Latin设计；完整边际计数及每一步距离/哈希在candidate_manifest.json，不能用12次混合调参推断单轴因果效应。后续局部细化保留内部层级。

四个实际接线问题需要在新组件阶段解决：

- 现launch先加载planner YAML，再加载共同footprint；后者会把TEB min_obstacle_dist重新覆盖为0.33。新B01必须在共同footprint之后加载完整planner配置，并实际dump/query核实。原launch完全未改。
- DWA1.17.3的stop_time_buffer_仅出现赋值和成员声明；该版本未把它用于轨迹/评分。保留合同0.3/0.5/0.7表值，但不把它称为已生效制动裕量，也不靠它凑配置数。forward_point_distance已由源码确认影响前向目标和对齐代价，列为后继细化轴。
- TEB当前12×12米local costmap使transformGlobalPlan采用5.1米径向窗口；配置lookahead6/8/10可能在直线段全部被同样截断。它们不是保证不同的有效horizon，不为B01偷偷扩大共用地图。
- workspace TEB0.8.4不读取costmap_converter_rate参数，实际代码用默认5Hz创建worker。B01不写这个无效字段；动态tracker dt按0.2秒设置。系统apt TEB为0.9.1，未混作当前workspace0.8.4依据。精确文件SHA/ELF BuildID及源码行号见source_effect_findings.json。

动态候选使用costmap_converter::CostmapToDynamicObstacles0.0.13，输入限于共同静态prior、真实scan形成的local Costmap2D及/odom自运动补偿。converter、include_dynamic_obstacles、动态权重与动态膨胀距离是显式联动组合；其余converter参数取本机cfg默认值。源码显示插件加载失败会回落到点障碍，成功加载时converter路径替换直接costmap点路径，因此必须在未来组件测试中确认实际加载、频率、frame/速度单位、静止与运动目标、空/陈旧输入和静态障碍保留；仅有YAML不构成已支持动态障碍的证据。未资格的converter-on项不得静默删除后继续宣称12配置已调参。

所有配置保持现有1.2米/秒前向请求上限、同一物理P05驱动和九碰撞部件参考。共同64平面外包footprint+0.33米保持，TEB采用原16平面完整包络，未退回旧底盘矩形。真实驱动绝对参考速度上限是2.778米/秒，中心转角0.4964rad、轴距0.65米，reference加速/减速/转角速率为0.6/0.9/0.75，Kd=6；这些不会被误写成实际ODE状态的严格上界。原TEB后退0.4与DWA前向采样差异、DWA acc_lim_y2.5只覆盖既定±0.125米/秒横向状态窗口，也明确保留。

预算与选取规则在design.yaml，全部为待审方案，当前授权预算为0。建议初筛采用6族×2个新parent×2次执行，共24配对blocks：每方12配置=288回合，双方合计576。后续每个局部batch双方各6个新配置、各144回合；只用development选择，按安全失败数、有效完成率、失败包含限制时间依序排名。局部搜索持续到双方均在两批完整证据中达到所定义plateau或合法邻域耗尽，不把单次失败当停止依据。

validation在每方anchor+最优3个非anchor冻结后才开展，并严格保持合同要求的每族至少12个新独立parent×3次执行，即216配对blocks、每方4配置864回合。训练/开发/validation/final按parent及参数cell分组隔离，不能随机拆episode。无完整性证据不能选winner；算法timeout/fault/碰撞/净空失败仍占原分母，非有效完成记22秒；安全损失不能用速度补偿。分别保留最强安全TEB和DWA，不因它们胜过PCT而丢弃。自适应调参和validation选择本身不是确认性优越性检验。

首次两个纯离线生成尝试分别因过严边际配额死路、maximin漏内部层级而触发断言，均在写配置之前退出，源码和日志完整保留；第三次引入显式层级覆盖优先后生成24份配置。它们不是仿真尝试或预算重用。

'''+ '\n\n'.join(tables)+'\n'
(O/'README.md').write_text(md)
for p in O.iterdir():
 if p.is_file()and p.name!='implementation_report.json':resources[str(p.resolve())]=sha(p)
assert all(sha(p)==h for p,h in resources.items())
report={'schema':'astra_baseline_b01_offline_design_implementation_v1','status':'offline_initial_baseline_tuning_design_ready_for_independent_review','author_role':'historical_audit acting as B01 design/config author; not an independent approval of its own configuration implementation','independent_review':False,'offline_checks_pass':all(checks.values()),'check_count':len(checks),'checks':checks,'configs_per_planner':12,'total_configs':24,'actual_reference_driver_parameters':actual,'ELF_files':libs,'coarse_design':m['matrix_design'],'configuration_manifest':binding(O/'candidate_manifest.json'),'selection_and_budget_proposal':binding(O/'design.yaml'),'source_effect_evidence':binding(O/'source_effect_findings.json'),'execution_authorized':False,'authorized_episodes':0,'seeds_allocated':[],'runtime_ready':False,'converter_runtime_qualified':False,'final_outcomes_read':False,'source_resources':[{'path':p,'sha256':h}for p,h in sorted(resources.items())]}
(O/'implementation_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
assert all(sha(x['path'])==x['sha256']for x in report['source_resources'])
print(json.dumps({'status':report['status'],'checks':len(checks),'resources':len(resources),'report_sha256':sha(O/'implementation_report.json'),'design_sha256':sha(O/'design.yaml'),'candidate_manifest_sha256':sha(O/'candidate_manifest.json')},indent=2))
