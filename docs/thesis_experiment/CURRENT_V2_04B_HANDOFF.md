# FAM-TEB V2-04B Handoff

更新时间：2026-07-14 CST

## 当前结论

V2-04B 已完成 simulation-only typed TEB 真实事务门，并完成预注册 calibration screen：
54 个候选、90/90 个 calibration episode 均形成有效证据。结果为 87 SUCCESS、3 ABORTED、
0 collision、0 interface failure；3 个失败都来自 Balanced Anchor 的 Maneuver 配对，均保留为
hard-gate 负样本。

30-episode 首次冻结门和 90-episode 最终冻结评估都已执行。最终结论为 `freeze_blocked`，不是
Anchor 已冻结：Balanced 的跨五族聚合规则没有预注册，9 个 Dynamic episode 都没有观测到
有限 TTC，而且合同声明的 bounded refinement 没有对应预注册规则。为避免查看结果后补规则，
当前 Anchor Bank 没有被修改。

所有部署合同继续保持 `runtime_ready=false`；没有加载或训练 SAC，没有实车写参数。

## 本轮实现

- 完成 18 double + 1 int + 1 bool 的 Gazebo typed 单请求事务、live schema/range/type、
  ack/readback/activation barrier、previous-executed 故障恢复和 startup snapshot 恢复；
- calibration launch 接入无场景标签的 `nav_world_model`，runner 从跟踪消息记录预测 TTC；
- 新增可恢复 batch orchestrator：center-first 顺序、episode 身份/trace/hash 校验、两次基础设施
  重试、三连接口失败熔断、每 episode 原子进度更新和完整仿真清理；
- 新增 fail-closed 离线冻结评估器；只输出单族 screen winner，不允许在阻断项存在时修改
  Anchor Bank。

## 实际结果

- typed Gazebo probe：41 个真实参数事务、6 个 Anchor 段全部收敛，startup snapshot 恢复；
- calibration：90/90 有效，87/90 hard-gate pass，0 collision，0 interface failure；
- Anchor 分布：Balanced 45，Cruise/Static Dense/Corridor/Maneuver Forward/Maneuver Reverse 各 9；
- 族分布：Cruise 18，Dynamic 9，Static Dense 18，Corridor 18，Maneuver 27；
- 最小 footprint clearance（按 Anchor）：Balanced 0.4061 m、Cruise 2.6883 m、Static Dense
  0.4907 m、Corridor 0.4730 m、Maneuver Forward 0.3969 m、Maneuver Reverse 0.3911 m；
- 30 条门点评估：29 SUCCESS、1 ABORTED，结论为继续到 90、不冻结；
- 最终冻结阻断：`balanced_cross_family_aggregation_not_preregistered`、
  `dynamic_primary_ttc_objective_unobserved`、`bounded_refinement_not_preregistered`。

单族临时 screen winner 只用于设计下一阶段，不能写回 Anchor Bank：Cruise
`anchor_cruise-c08-weight_optimaltime_high`，Static Dense
`anchor_static_dense-c01-min_obstacle_dist_low`，Corridor
`anchor_corridor-c02-max_vel_x_high`，Maneuver Forward/Reverse 均为各自
`c05-weight_obstacle_low`。Balanced 不可排序。

## 验收

- Python：201/201；`teb_mode_manager` catkin：20/20；`thesis_experiment` catkin：43/43；
- 全工作空间 73 包构建通过；当前 catkin 汇总 168 tests、0 errors、0 failures；
- 合同/计划/90 个 evaluation/trace 哈希链由 batch 和 freeze evaluator 逐项校验；
- 结束后无 ROS/Gazebo 残留进程。

## 机器入口

- 合同：`config/thesis_experiments/v2/typed_transaction_calibration_contract.yaml`；
- typed 后端：`src/application/teb_mode_manager/src/teb_mode_manager/typed_teb_transaction.py`；
- calibration runner：`src/tools/thesis_experiment/scripts/v2_04b_calibration_episode.py`；
- batch orchestrator：`src/tools/thesis_experiment/scripts/v2_04b_calibration_batch.py`；
- batch 进度：`artifacts/v2/calibration/v2_04b_batch_progress.yaml`；
- 30 条门点评估：`artifacts/v2/calibration/v2_04b_freeze_gate_030.yaml`；
- 最终冻结评估：`artifacts/v2/calibration/v2_04b_final_freeze_assessment.yaml`；
- 汇总验收：`artifacts/v2/component_acceptance/v2_04b_acceptance.yaml`。

## 下一实施门

进入新的 V2-04C 前必须先预注册，而不是直接修改现有合同：

1. 固定 Balanced 的跨族聚合方法及缺失 TTC 的排序语义；
2. 调整 Dynamic calibration 交互时序，使 TTC 主指标可观测；
3. 固定 bounded refinement 的候选生成、预算、停止条件和独立 calibration seeds；
4. 完成 refinement 后重新执行 fail-closed freeze assessment；
5. 只有 Anchor Bank 真正冻结后，才允许五类无训练配对对照。

SAC 训练和实车闭环仍未授权。
