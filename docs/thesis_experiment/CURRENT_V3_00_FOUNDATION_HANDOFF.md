# V3-SCOPE V3-00 纯离线基础闭环交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`v3_00_pure_offline_foundation_closure_pass`

## 1. 一句话结论

V3-00 已建立独立于 V2 的 V3-SCOPE package/config/manifest/artifact namespace，冻结系统指南、
五类 ROS 消息、Ackermann 候选轨迹合同、hermetic 环境与 package-resolution 协议、
反应式 actor/cloned-rollout 数据合同、静态 fixtures、deterministic builder、dependency
closure 和 independent reviewer。所有纯离线门通过。

这只证明 V3 后续研发具备可复核的离线基础，不证明 actor 已在 Gazebo 中响应自车、预测模型
已训练、规划器已运行、安全或性能已通过。

## 2. 新增 namespace 与入口

```text
src/application/v3_scope_planner/
config/thesis_experiments/v3/
experiments/manifests/v3/
artifacts/v3/
docs/thesis_experiment/V3_SYSTEM_GUIDE.md
```

V3 ROS package 当前只有接口，没有 node、launch、模型或执行入口。五类消息为：

- `TrajectoryPoint`
- `CandidateTrajectory`
- `CandidateSet`
- `CandidateEvaluation`
- `CandidateDecision`

未来 tracker 的唯一命令出口仍为 `/cmd_vel_navigation`；直接发布 `/cmd_vel` 或访问 M2 driver
被合同禁止。

## 3. V3-00 静态闭环结果

- source freeze：7 个历史/概念源绑定通过，包含 P4 终态、V3 handoff/roadmap 和
  simulation-candidate vehicle profile；
- dependency closure：33 个本地资源，0 unresolved；
- source package resolution：`v3_scope_planner`、`thesis_experiment`、`m2_gazebo`、
  `nav_world_model`、`teb_mode_manager`、`gps_module` 六包均唯一解析；
- message registry：5/5；
- candidate fixture：8 条候选、ID 唯一、1 条 certified stop、1 条 TEB fallback；
- candidate bicycle-model maximum discrete residual：`0.0`；
- actor fixture：3 个 hidden policy × 8 cloned candidates = 24 rows；
- 每个 policy 的 candidate propensity 均为 `0.125`，严格正且总和为 1；
- clone initial-state digest 一致，hidden-label firewall 通过；
- 三个合成 response-effect schema fixture 均超过 0.05 s 噪声地板；
- fixture 明确为 schema/gate test，`simulation_evidence=false`、
  `counterfactual_identifiability_demonstrated=false`。

## 4. 构建与测试

初次 target build 暴露既有 CMake cache 中的 V2 whitelist 未包含新包，因而没有生成
`build/application/v3_scope_planner`。未删除 build；随后显式冻结五包 whitelist：

```text
v3_scope_planner;thesis_experiment;m2_gazebo;nav_world_model;teb_mode_manager
```

干净 `env -i`、Noetic-only、non-login shell 下重新配置和编译成功：

- catkin 遍历 5 个白名单包；
- `v3_scope_planner` 5 类消息的 C++/Python/EusLisp/NodeJS/Lisp 全部生成；
- 生成的 5 个 Python message import 通过；
- pytest：`7 passed`；
- catkin nosetests：`7 tests, OK`；
- evidence freeze 中全部 33 个资源重新哈希通过；
- `git diff --check` 通过；
- 未启动 ROS master、Gazebo、move_base、训练或实车。

## 5. 核心证据 SHA-256

| artifact | SHA-256 |
|---|---|
| foundation evidence freeze | `4a08325b67c0706a860560ab7ced1191100e5ecb91e4b6138679aa0c11a05d5b` |
| independent offline review | `bb9ce5641ed650f2f3b14688f15f7a3787679f7e06c9bc4d62cd435be7b6b1bf` |
| foundation summary | `c297ba8e92057638fe7d3142864a9f03be49b3b29c1737d75a23cfd198342b3c` |
| candidate fixture | `7007a3f79c57bd6c9555fa9f4c2eef3546fdee49b5e5d69f49a7a6cefeaaa8a0` |
| actor/clone data contract | `b2257c38379547670b70147fbdc7703aef53fab314d832dcb248c54475335dbd` |

终态 artifact 根：

```text
artifacts/v3/foundation/v3_00/
```

## 6. 权限与声明边界

V3-00 终态保持：

```text
formal_result=false
runtime_ready=false
execution_ready=false
execution_authorized=false
training_authorized=false
evidence_budget_authorized=0
seed_allocation_count=0
schedule_row_count=0
real_vehicle_authorized=false
next_stage_automatically_authorized=false
```

不存在 execution release、simulation authorization、seed allocation、schedule、attempt、
journal、model 或 checkpoint。不得把 V3-00 pass 写成仿真、性能、学习、安全或部署通过。

## 7. 下一安全入口

下一阶段只能是独立 `V3-01`：

> 反应式 actor、cloned intervention 场景族和反事实数据资格的离线设计、实现审查与单独
> bounded simulation release。

进入任何 Gazebo 消费边界前必须新建：

1. V3-01 preregistration 与 scene/actor policy family；
2. fresh seed allocation、exact schedule 和不可回收预算；
3. reactive actor 实现与 clone-reset deterministic fixtures；
4. hermetic environment/package-resolution runtime bindings；
5. exact runner、raw journal/replay 和 failure index；
6. independent release review；
7. 明确的 simulation authorization。

V3-01 必须先证明 actor response 随 ego candidate 发生可辨识变化且大于噪声地板；否则停止
反事实主张，转为非交互的 uncertainty-calibrated safe planner。V3-00 本身不提供执行授权。
