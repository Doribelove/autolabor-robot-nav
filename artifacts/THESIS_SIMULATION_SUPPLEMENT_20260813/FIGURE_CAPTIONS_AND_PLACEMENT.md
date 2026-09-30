# 图注与放置建议

以下图注可直接复制后修改图号。建议正文优先使用组合图和关键结果图，独立场景图放在需要展开说明的章节或附录中。

## 一、Gazebo 四方法正式对比

| 文件 | 建议位置 | 建议图注 |
|---|---|---|
| `01_gazebo_formal_comparison/figures/png/formal_effects_97_5_ci.png` | “对比实验结果”主图 | **两轮正式 Gazebo 实验中 PCT-SCOPE-RL 相对传统规划器的失败计入时间效应。** 点为配对相对降低，误差线为双侧 97.5% block-bootstrap 区间，红色虚线为预注册的 8% 点效应门限。R1、R2 均无比较通过全部阳性门限。 |
| `01_gazebo_formal_comparison/figures/png/formal_r2_method_outcomes.png` | R2 总体结果 | **R2 正式实验中四种方法的总体结果。** 图中并列给出失败计入平均时间与有效到达率；超时、未到达、接触、负保守净空及系统失败均按冻结规则计入，不删除失败样本。 |
| `01_gazebo_formal_comparison/figures/png/formal_r2_scene_strata.png` | 分场景分析 | **R2 正式实验的六场景分层结果。** 各子图比较 PCT-SCOPE-RL、解析先验、Actual TEB 与 DWA 在冻结场景中的失败计入时间和到达表现，用于识别总体效应的场景异质性。 |
| `01_gazebo_formal_comparison/figures/png/r2_development_ablation.png` | 机制分析或附录 | **R2 开发阶段的候选覆盖单因素消融。** 该图仅用于解释 0.6 m/s 低速空间候选对开发集行为的影响，不属于正式 R2 主结果，也不能替代 fresh-seed 正式评价。 |

如排版系统支持 SVG，可将上述路径中的 `figures/png/*.png` 替换为 `figures/svg/*.svg`。

## 二、M2 机器人与六类场景

| 文件 | 建议位置 | 建议图注 |
|---|---|---|
| `02_m2_scene_gallery/00_six_scene_overview.png` | “仿真环境与测试场景”主图 | **Gazebo 仿真测试场景。**（a）静态障碍避让；（b）单行人横穿；（c）局部遮挡横穿；（d）迎面交互与让行；（e）连续双行人交互；（f）宽度变化走廊。 |
| `02_m2_scene_gallery/01_m2_model_overview.png` | “实验平台”或“机器人模型” | **M2 移动机器人及静态障碍仿真环境。** 蓝色模型为 M2 移动机器人，橙色立方体为静态障碍物，蓝色和绿色圆盘分别表示起点与目标点。 |
| `02_m2_scene_gallery/02_static_avoidance.png` | 静态场景说明 | **静态障碍避让场景。** 机器人需在有限宽度走廊中绕过主行进方向上的静态障碍并到达目标。 |
| `02_m2_scene_gallery/03_single_crossing.png` | 动态交互说明 | **单行人横穿场景。** 动态参与者沿横向路径穿过机器人行进路线，形成典型时空冲突。 |
| `02_m2_scene_gallery/04_partially_occluded_crossing.png` | 遮挡场景说明 | **局部遮挡横穿场景。** 动态参与者从遮挡体后方接近道路，用于展示有限可见条件下的潜在冲突。 |
| `02_m2_scene_gallery/05_head_on_yield.png` | 会车/让行说明 | **迎面交互与让行场景。** 动态参与者与机器人沿相反纵向运动，形成狭窄通道中的速度协调任务。 |
| `02_m2_scene_gallery/06_sequential_interaction.png` | 多阶段交互说明 | **连续双参与者交互场景。** 两个动态参与者在不同位置依次横穿机器人路线，形成连续冲突下的多阶段局部决策任务。 |
| `02_m2_scene_gallery/07_width_change_corridor.png` | 结构约束说明 | **宽度变化走廊场景。** 通道由宽变窄后再次变宽，用于展示空间约束变化对轨迹与速度选择的影响。 |

这组图片是论文可读性优化后的暂停态展示图，不是正式实验 episode 的截图。若正文只需要一幅环境图，优先使用六场景总览。

## 三、Arena 高细节环境

| 文件 | 建议位置 | 建议图注 |
|---|---|---|
| `03_arena_scene_gallery/00_arena_environment_overview.png` | “Arena 仿真平台”主图 | **Arena-Rosnav-3D 集成的多类型三维仿真环境。**（a）住宅；（b）书店；（c）医院；（d）工厂；（e）小型仓库。该图为暂停态平台扩展场景示例，非本文冻结正式实验结果。 |
| `03_arena_scene_gallery/01_residential.png` | 环境案例或附录 | **家具密集型住宅三维仿真环境。** 场景包含客厅、厨房、餐桌、隔断和室内家具。 |
| `03_arena_scene_gallery/02_bookstore.png` | 环境案例或附录 | **包含书架与展台的书店三维仿真环境。** 场景同时具有规则货架通道和开放公共区域。 |
| `03_arena_scene_gallery/03_hospital.png` | Arena Q1 场景介绍 | **包含病房与走廊网络的医院三维仿真环境。** 本图用于展示 hospital 世界的资产与结构，不是 Q1 episode 回放。 |
| `03_arena_scene_gallery/04_factory.png` | 环境案例或附录 | **包含输送线与工业货架的工厂三维仿真环境。** 场景具有工业设备、托盘和狭长作业通道。 |
| `03_arena_scene_gallery/05_small_warehouse.png` | 环境案例或附录 | **包含货架、堆垛区与装卸区的小型仓库三维仿真环境。** |

可配套正文：

> 为展示仿真平台对不同空间结构和视觉资产的兼容能力，本文补充给出了住宅、书店、医院、工厂及小型仓库五类三维环境。它们覆盖家具密集空间、规则货架通道、大型走廊网络和工业作业区，但未全部纳入本文冻结的定量评价集合。

## 四、Arena Hospital 四方法资格实验

| 文件 | 建议位置 | 建议图注 |
|---|---|---|
| `04_arena_q1_qualification/figures/failure_inclusive_time.png` | Arena 对比结果主图 | **Arena Hospital 资格实验的原始到达与安全规范合格结果。** 柱高为 3 个配对块的失败计入平均时间，空心点表示方法原生到达时间，实心点表示统一安全规范口径。Actual TEB 与 DWA 在两个受控横穿块中均到达目标，但因保守净空为负而在安全口径中记为 30 s。该图为描述性资格结果，不包含显著性推断。 |
| `04_arena_q1_qualification/figures/hospital_route_trajectories.png` | 轨迹与交互分析 | **Arena Hospital 受控横穿任务中的执行轨迹。** 实线为四种方法的机器人轨迹，彩色点线为闭环反应式代理在对应方法下的响应轨迹。四方法共享 M2 机器人、传感器、地图、起终点和 30 s 上限，但内部地图使用语义不同，因此该图表示完整导航系统栈比较。 |

推荐紧随结果图使用的正文：

> 在冻结的 Arena Hospital 单走廊资格实验中，四种方法均实现 3/3 原始到达。PCT-SCOPE-RL 与解析先验均实现 3/3 保守规范合格到达，而 Actual TEB 与 DWA 均为 1/3。PCT 相对共享候选集和硬约束的解析先验，其安全口径失败计入均时由 16.067 s 降至 12.011 s，描述性降低 25.25%。但 DWA 的原始到达均时为 10.038 s，快于 PCT 的 12.011 s。该 n=3 结果只提供受限任务中的机制与接口资格证据，不构成显著性或一般优越性证明。

## 五、排版取舍建议

- 正文篇幅有限时，保留 M2 六场景总览、Arena 五环境总览、正式效应图和 Arena Q1 双口径图即可。
- Arena 独立环境图适合放入附录，正文不要同时重复总览和全部五张单图。
- development 消融图必须与正式结果图分开，并在图注中明确“开发阶段”。
- Q1 结果图必须同时解释原始到达和安全规范口径；不要只展示封顶后的百分比。
- 所有“场景展示图”与“量化结果图”应在正文中分段介绍，避免读者误以为算法已在所有展示环境中完成正式验证。

