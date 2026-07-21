# FAM-TEB V2-04 Handoff

> 最新 calibration-only supervisor 修复与全新 held-out 配对结论见
> `docs/thesis_experiment/CURRENT_V2_04F_HANDOFF.md`。V2-04F 三方法均为 10/10 成功，
> Cruise/Static 混淆已消除且 Maneuver Anchor 已触发；但净空、抖动、TTC 覆盖和效率门
> 仍失败，因此当前不授权 V2-05。本文保留为
> V2-04 shadow 边界记录。

更新时间：2026-07-14 00:54 CST

## 当前结论

V2-04 的软件与 shadow 事务门已经完成：Anchor Bank、类型化 profile、约束内生的 feasible
decoder、从上一 `executed` 开始的平滑事务、四阶段 action trace，以及不加载任何学习策略的
规则闭环均已实现并通过验收。

这不等于 Anchor 已正式标定，也不等于完成了 Gazebo 导航性能验证。当前执行后端固定为
`deterministic_shadow`，没有接入 dynamic_reconfigure，不写真实 TEB 参数。全部部署注册表、
模式阈值和 V2-04 合同继续保持 `runtime_ready=false`。

## 已完成范围

- 建立 6 个几何 Anchor：Balanced、Cruise、Static Dense、Corridor、Maneuver Forward 和
  Maneuver Reverse；动态风险以 NONE/CROSSING/HEAD_ON/FOLLOW/OVERTAKE_OR_YIELD 五个
  factorized overlay 叠加，不为笛卡尔积复制 profile；
- 固定 20 参数顺序与类型：18 个 `double`、1 个 `int`、1 个 `bool`；fast continuous 与
  slow mode profile 生命周期分离，未知键、缺键、类型漂移、非有限数和越界值均 fail closed；
- feasible decoder 接收 `[-1,1]` 语义残差，以有界映射、正权重映射、Ackermann
  `omega <= v/Rmin`、clearance+positive-gap 生成可行动作；末端 projection 只保留异常审计；
- 动态 overlay 和 zero residual 规则路径的 6 Anchor x 5 overlay 共 30 个组合均无需投影；
- 连续参数每周期都从上一条已 ack/readback/active 的 `executed` 值按参数变化率推进，不从
  新 Anchor 或固定 TEB-Tuned 瞬跳；`int/bool` 仅在连续参数收敛后原子提交；
- timeout、ack mismatch、readback mismatch 均保持上一 `executed`，不污染下一事务；
- `ParameterTransaction.msg` 现在携带 world/mode/config sequence、模式/overlay/transition、
  参数名和类型、commanded/feasible/safe/executed、reason mask、四个事务时间戳、激活和
  slow-profile commit 状态；
- ROS 规则节点只订阅 `ContextState`，使用 zero residual，不读取场景 manifest/标签、Gazebo
  真值或 checkpoint，不发布 `/cmd_vel`。

## 验收结果

- 全工作空间 `catkin_make -j4 -l4`：通过，73 个包；
- Python/config 回归：191 passed；
- `teb_mode_manager` catkin：15/15；`thesis_experiment` 新 V2-04：4/4；原有
  `nav_world_model`、`m2_gazebo`、`teb_rl_tuner` 回归继续通过；
- 离线规则闭环：800/800 事务激活，projection 0/800，连续参数越变化率跳变 0，最大变化率
  比 1.0000000000000862（浮点误差），四阶段 trace 100% 可重建；
- 三类事务故障 3/3 原子保持上一 executed；
- ROS topic 探针：20 条 transaction，15 条有效激活、5 条 invalid hold，20 参数四阶段均
  完整，normal projection 0，最大相邻 `max_vel_x` 变化 0.10002；
- 运行期间没有 SAC、checkpoint、实车、速度命令或真实 TEB 参数写入。

## 机器入口

- 合同：`config/thesis_experiments/v2/action_pipeline_contract.yaml`；
- Anchor Bank：`src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml`；
- 核心实现：`src/application/teb_mode_manager/src/teb_mode_manager/action_pipeline.py`；
- ROS 节点：`src/application/teb_mode_manager/scripts/rule_anchor_transaction_node.py`；
- 离线报告：`artifacts/v2/component_acceptance/v2_04_rule_loop_acceptance.yaml`；
- ROS 报告：`artifacts/v2/component_acceptance/v2_04_ros_runtime_probe.yaml`；
- 汇总验收：`artifacts/v2/component_acceptance/v2_04_acceptance.yaml`。

## 不得误解的边界

- Anchor 数值是从 V1 Gazebo 中点和工程先验派生的未标定 simulation candidate，正式 test
  场景没有参与选择；不能写成“独立优化后的最优 Anchor”；
- shadow ack/readback 证明事务状态机和 ROS 接口，不证明 TEB 接受全部 typed profile，也不
  证明导航成功率或效率提升；
- 当前 `safe=feasible` 属于 V2-04 参数安全占位，预测运动盾和最小干预速度安全属于后续门；
- Maneuver Reverse Anchor 已定义，但规则 supervisor 尚无离散换挡/目标姿态决策输入；
- dynamic obstacle tracker 到 TEB obstacle bridge、topology lock、corridor centerline 和
  maneuver planner 均未实现；
- 本阶段不授权 SAC 训练、实车闭环或实车参数写入。

## 下一实施门

先在 calibration split 完成 Anchor 独立标定，并为仿真 TEB typed 参数接口增加同等严格的
simulation-only 写门、真实 ack/readback/restore 和五类无训练导航对照。该门通过后再按单因素
顺序进入 V2-05：dynamic bridge、static topology lock、corridor centerline、maneuver planner
和 predictive motion shield。任何学习仍须等规则多 Anchor 基线冻结后另行授权。
