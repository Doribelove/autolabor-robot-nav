# FAM-TEB V2 Foundation Handoff

更新时间：2026-07-13 22:56 CST

## 当前结论

V2-00（V1 基线冻结与隔离）和 V2-01（机器合同与消息骨架）已经完成并通过组件级
验收。该结论只证明配置隔离、接口、空节点和基础修复可构建、可测试，不代表 V2 导航
算法已经实现，也不是论文正式结果。

机器验收记录：
`artifacts/v2/component_acceptance/v2_foundation_acceptance.yaml`。

## 已完成范围

- 已记录主仓、全部子模块、dirty 状态，以及关键 T11/T12 配置和结果的 SHA256；
- 已建立独立的 `config/thesis_experiments/v2/`、`experiments/manifests/v2/` 和
  `artifacts/v2/`；
- V1 runner 对路径中的 `v2`、V2 schema 和 V2 architecture generation fail closed；
- 已冻结 `GeometryMode`、`DynamicOverlay`、`TransitionState` 和
  `commanded/feasible/safe/executed` 四阶段动作语义；
- 已建立 architecture、parameter registry、mode thresholds、state contract 及严格
  validator；所有运行阈值仍故意保持未冻结，`runtime_ready=false`；
- 已建立 `nav_world_model` 和 `teb_mode_manager` ROS 包、8 个消息和 fail-closed 空节点；
- 已修复 corridor latch 跨 episode 泄漏；LaserScan 角域与四方向覆盖（包括后向覆盖）
  已进入状态合同，且未改变 V1 observation 维度。

## 验收结果

- 全工作空间 `catkin_make -j4 -l4`：通过，73 个包；
- Python/config 测试：150 passed；
- catkin 测试：`m2_gazebo` 14/14、`teb_rl_tuner` 60/60、
  `thesis_experiment` 21/21；
- `rospack` 和 `rosmsg` 可发现两个新包及其消息；
- 未启动 Gazebo 训练，未启动实车，未写入在线 TEB 参数。

## 不得误解的边界

- 两个节点当前只发布 invalid/stale/faulted 健康或模式状态，不发布速度命令，也不写 TEB；
- 参数注册表和模式阈值中的待标定值为 `null`，validator 的 runtime gate 必须拒绝启动；
- 动态跟踪、预测、Hybrid A*、拓扑锁、走廊专用约束、Anchor Bank 和 SAC 均未实现；
- V1/T11/T12 保持冻结，不得借 V2 名义重跑历史 T12 pilot 或扩大其预算。

## 下一实施门：V2-02

下一步应先建立可信的仿真执行器、制动/时延模型、30--60 m Cruise、动态 crossing 和基础
dead-end 场景，以及统一 evaluator。只有 V2-02 通过并冻结相应机器合同后，才进入世界模型、
规则监督器和无训练多 Anchor 闭环；当前不授权任何新训练或实车闭环。
