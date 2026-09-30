B01初始基线调参方案已生成，共24份新配置：workspace TEB0.8.4和安装版DWA1.17.3各12份，分别包含当前有效anchor。它们只是一份可审查设计，没有分配种子、启动ROS/Gazebo或读取final结果；也没有修改当前N04或任何已有配置。

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

TEB参数表（00为当前有效anchor）：

|candidate_id|dt_ref|min_obstacle_dist|inflation_dist|weight_obstacle|lookahead_m|homotopy|dynamic_scan_converter|
|---|---|---|---|---|---|---|---|
|teb_b01_00_anchor|0.25|0.33|0.6|80.0|8.0|False|False|
|teb_b01_01|0.35|0.25|0.8|160|6|True|True|
|teb_b01_02|0.15|0.4|0.45|40|10|True|True|
|teb_b01_03|0.15|0.4|0.8|160|6|False|True|
|teb_b01_04|0.15|0.25|0.45|160|10|True|False|
|teb_b01_05|0.35|0.4|0.8|40|8|True|True|
|teb_b01_06|0.35|0.25|0.45|160|6|False|True|
|teb_b01_07|0.15|0.4|0.8|160|10|True|False|
|teb_b01_08|0.35|0.25|0.45|40|8|True|True|
|teb_b01_09|0.35|0.4|0.45|160|8|True|False|
|teb_b01_10|0.15|0.25|0.8|40|10|True|True|
|teb_b01_11|0.35|0.25|0.8|40|6|False|True|

DWA参数表（00为当前有效anchor）：

|candidate_id|sim_time|vx_samples|vth_samples|occdist_scale|path_distance_bias|goal_distance_bias|stop_time_buffer|
|---|---|---|---|---|---|---|---|
|dwa_b01_00_anchor|3.0|18|36|0.15|32.0|20.0|0.4|
|dwa_b01_01|2.0|30|60|0.4|16|32|0.7|
|dwa_b01_02|4.0|30|60|0.1|48|16|0.5|
|dwa_b01_03|2.0|30|36|0.2|48|24|0.3|
|dwa_b01_04|4.0|18|60|0.4|48|32|0.7|
|dwa_b01_05|4.0|30|36|0.4|16|16|0.3|
|dwa_b01_06|3.0|18|60|0.4|16|16|0.5|
|dwa_b01_07|2.0|18|60|0.1|16|32|0.3|
|dwa_b01_08|4.0|30|36|0.1|16|32|0.5|
|dwa_b01_09|2.0|30|60|0.1|16|16|0.7|
|dwa_b01_10|4.0|18|36|0.4|16|32|0.5|
|dwa_b01_11|2.0|18|36|0.4|48|32|0.7|
