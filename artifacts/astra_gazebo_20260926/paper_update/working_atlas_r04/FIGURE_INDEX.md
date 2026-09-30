# 论文图册索引（阶段性）

本图册只收录 `figure_manifest.csv` 与 `figure_plan.csv` 中已通过实际视检的独立图组。每图数据范围与限制以原图 QC 记录为准；当前图册不等于最终 30 图交付。

## H01 历史 R2 两次冲突失败

- 问题：历史误选发生在何种几何和运动关系下
- 选择与边界：历史归因图；不得当作本轮新训练或新形式评估
- 质检：pass_actual_agent_view_r03
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/H01.png)；[QC来源](artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/source_manifest.json)

## H02 方向性负全预测失配

- 问题：方向性预测与参与者实际运动如何偏离
- 选择与边界：仅展示历史错误实例与预测失配
- 质检：pass_actual_agent_view_r03
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/H02.png)；[QC来源](artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/source_manifest.json)

## H03 历史超时与 TF 失败分账

- 问题：超时和 TF 失败应如何独立记录
- 选择与边界：按原始故障类别分账；不得合并成规划性能
- 质检：pass_actual_agent_view_r03
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/H03.png)；[QC来源](artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/r03/source_manifest.json)

## Q01 四方法 Gazebo 链路实拍检查

- 问题：四个方法的早期 Gazebo 图像链路是否形成
- 选择与边界：启动链路图，不构成方法成功或性能结果
- 质检：pass_actual_all_four_panel_and_native_views
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/panels/bootstrap_a02/r01/Q01_smoke_overview.png)；[QC来源](artifacts/astra_gazebo_20260926/visuals/panels/bootstrap_a02/r01/manifest.json)

## AQC02 冻结转向门下的驱动比较

- 问题：原驱动与假转速通道在同一转向门下如何表现
- 选择与边界：物理通道诊断；不等于实车验证
- 质检：pass_actual_root_and_agent_view_six_panels
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/actuation_q03_comparison/r02/AQC02.png)；[QC来源](artifacts/astra_gazebo_20260926/visuals/plots/actuation_q03_comparison/source_manifest.json)

## TIM01 源时间与异步采集协议

- 问题：接收时钟和异步采集如何保持时间可追溯
- 选择与边界：协议诊断；不构成导航时延性能比较
- 质检：pass_actual_agent_native_and_root_170mm_six_panels
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/physical_qualification/r02/TIM01.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/physical_qualification/TIM01_visual_qc.json)

## PHY05 有界转向半径的三阶段执行

- 问题：三阶段转向执行能否形成连续物理轨迹
- 选择与边界：仅限冻结仿真物理执行样本；无实车结论
- 质检：pass_agent_and_root_actual_nativePNG_and170mmPDF
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/physical_qualification/PHY05/r03/PHY05.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/physical_qualification/PHY05/PHY05_visual_qc.json)

## F04 物理与保守净空

- 问题：方盒和机器人compound净空如何定义
- 选择与边界：独立几何 fixture 的定义与边界；不表示 Gazebo 导航安全或实车净空
- 质检：pass_independent_and_root_paper_size_and_native_actual_views
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/geometry_contract/r07/F04.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/geometry_contract/r07/F04_manifest.json)

## NAV01 静态绕障失败中的候选可行性与控制时限

- 问题：候选mask耗尽及控制时限如何伴随一次开发超时
- 选择与边界：single pre-existing chronological diagnostic case; not population representative
- 质检：root actual r02 PNG and paper preview pass
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/navigation_n04_failure_trace/r02/NAV01_static_failure_trace.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/visuals/plots/navigation_n04_failure_trace/r02/delivery_manifest.json)

## NAV02 旧静态超时轨迹与固定状态前瞻假设

- 问题：旧超时路径中前瞻距离变化如何影响相同状态下的期望转向
- 选择与边界：单个旧开发失败案例；不得推断N06闭环收益或安全性
- 质检：root actual PNG/PDF render pass; prior r01/r02 rejected
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV02_lookahead_fixed_state_diagnostic_r03/NAV02_lookahead_fixed_state_diagnostic.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV02_lookahead_actual_view_qc.json)

## NAV03 N04五次静态超时中的独立事件率与终态距离

- 问题：五个旧开发回合的停止与退避相关事件是否一致以及终态距目标多远
- 选择与边界：五条旧有效超时完整保留；中断和未运行身份注明；非总体效应
- 质检：root actual PNG and PDF render pass
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV03_n04_five_attempts_r01/NAV03_n04_five_attempts.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV03_n04_five_attempts_r01/actual_view_qc.json)

## NAV04 N05配对终态距离与重叠事件率

- 问题：预览可选性约束改变动作后是否改善旧开发几何的结果
- 选择与边界：完整保留成功/超时和b04/b05安全失败；无性能提升或安全主张
- 质检：root actual PNG and PDF render pass; r01 footnote spacing rejected
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV04_n05_paired_diagnostic_r02/NAV04_n05_paired_diagnostic.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV04_n05_paired_diagnostic_r02/actual_view_qc.json)

## NAV05 N05最大终态差配对的原生仿真截图

- 问题：同一新seed/场景下两方法从相同早期画面到终态如何分化
- 选择与边界：12对中处理减对照终态距离最大的一对，标注选例和非总体推断
- 质检：root each raw original plus full panel PNG/PDF pass; r01 footer overlap rejected
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV05_n05_native_pair_r02/NAV05_n05_native_pair.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV05_n05_native_pair_r02/actual_view_qc.json)

## NAV06 N05保守扫掠净距安全门

- 问题：零违法接触日志能否排除保守工程净距失败
- 选择与边界：全24回合显示，b04/b05四个负值保留；不作形式安全或总体性能主张
- 质检：root actual full PNG and PDF render pass; no overlap/crop
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV06_n05_clearance_gate_r01/NAV06_n05_clearance_gate.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV06_n05_clearance_gate_r01/actual_view_qc.json)

## NAV07 N06四个新静态开发布局与共用几何引导线

- 问题：单因素静态先导试验使用了哪些独立新场景
- 选择与边界：展示全部四个新静态开发布局；几何示意，不冒充Gazebo截图或执行轨迹
- 质检：root full PNG and PDF render pass; r01/r02 panel-title overflow rejected
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV07_d04_static_layouts_r03/NAV07_d04_static_layouts.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV07_d04_static_layouts_r03/actual_view_qc.json)

## NAV08 D04四个新静态场景的离线保守几何净距

- 问题：被替代的负值原见证路径与选定路径是否如实保留
- 选择与边界：全部四布局；明确显示slot001旧负值；几何门不推断导航安全
- 质检：root full PNG and PDF render pass; r01 legend overlap rejected
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV08_d04_static_geometry_margins_r02/NAV08_d04_static_geometry_margins.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/NAV08_d04_static_geometry_margins_r02/actual_view_qc.json)

## LAT01 M13 合成输入构建剖析

- 问题：候选生成和验证的主要代码热点在哪里
- 选择与边界：完整呈现稀疏和密集两种合成输入；累积时间不可相加；不推断运行期限
- 质检：pass_root_actual_full_png_and_170mm_pdf_r06
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/LAT01_m13_synthetic_profile_r06/LAT01.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/LAT01_m13_synthetic_profile_r06/source_manifest.json)

## F01 部署安全接口

- 问题：sensor到cmd_vel_navigation安全链是否完整
- 选择与边界：接口设计示意；现阶段不宣称整条实车链或shadow已验证
- 质检：pass_root_actual_full_png_and_170mm_pdf_r01
- [原图](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/F01_command_boundary_r01/F01.png)；[QC来源](/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926/paper_update/figures/F01_command_boundary_r01/source_manifest.json)
