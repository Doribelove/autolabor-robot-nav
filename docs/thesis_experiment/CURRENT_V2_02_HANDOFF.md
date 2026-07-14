# FAM-TEB V2-02 Handoff

更新时间：2026-07-13 23:49 CST

## 当前结论

V2-02（可信仿真动力学、制动/时延模型和五类基础场景系统）已经完成组件级实现与验收。
机器记录为 `artifacts/v2/component_acceptance/v2_02_acceptance.yaml`，SHA256 清单为
`artifacts/v2/component_acceptance/v2_02_acceptance.sha256`。

该结论只证明仿真执行链、场景编译与评估基础设施可构建、可重复运行并 fail closed，
不代表世界模型、规划器、规则监督器或学习策略已经产生导航性能提升。所有参数和模式阈值
继续保持 `runtime_ready=false`。

## 已完成范围

- M2 Gazebo 插件新增隔离启用的一阶速度/转向响应、加减速限制、物理制动/急停权限、
  倒车零速穿越、命令 timeout、确定性延迟/抖动队列和 reset 原子清理；V1 默认行为不变；
- LaserScan 加入保留采集时间戳的确定性时延、抖动与噪声传输，底盘加入接触碰撞话题；
- 创建严格的 `simulation_contract.yaml` 和 `evaluation_contract.yaml`，训练、正式结果、
  实车安全声明均明确关闭；
- 创建五类机器场景 manifest、严格编译器和 SDF 实例：`CRUISE`、`DYNAMIC`、
  `STATIC_DENSE`、`CORRIDOR`、`MANEUVER`；运行时策略禁止读取 family/split/可行性标签；
- 动态 crossing agent 使用 pose-driven static collision object：既能被激光与碰撞系统看到，
  又避免 Gazebo 在机器人并发插入期间修改动态物理体导致的 `gzserver` SIGSEGV；
- 创建统一 episode evaluator，固定 trace schema、终止优先级、原始 trace/scene hash 校验、
  通用指标和五类场景专用指标。

## 验收结果

- 全工作空间 `catkin_make -j4 -l4`：通过，73 个包；
- 全 Python/config：162 passed；
- catkin：`m2_gazebo` 34/34、`teb_rl_tuner` 60/60、
  `thesis_experiment` 29/29；
- 两次 seed=42 Gazebo 动力学运行共 12/12 case 通过；代表值为 1 m/s 直线稳态、
  1.500000 m 定圆半径、0.252243 m 非零制动距离、0.92 s 停车、低速倒车成功、
  命令激活延迟 0.074--0.114 s、平均 LaserScan age 0.06375 s；
- 当前重新编译的五类场景均通过确定性与合同测试；动态 crossing 运行探针观测 agent
  运动 1.498 m，机器人同时存在且 `gzserver` 无崩溃；
- 未启动 Gazebo 训练，未启动实车，未写入实车或在线 TEB 参数。

## 证据入口

- 仿真合同：`config/thesis_experiments/v2/simulation_contract.yaml`；
- 评估合同：`config/thesis_experiments/v2/evaluation_contract.yaml`；
- 场景 manifest：`experiments/manifests/v2/scenes/v2_02_foundation_scenes.yaml`；
- 编译场景：`artifacts/v2/scenes/v2_02_foundation/`；
- 动力学报告：`artifacts/v2/component_acceptance/v2_02_dynamics_regression.yaml` 与
  `v2_02_dynamics_regression_repeat.yaml`；
- 动态场景探针：`artifacts/v2/component_acceptance/v2_02_dynamic_scene_runtime_probe.yaml`。

## 不得误解的边界

- 当前仍是平面运动学 pose integration，不是轮胎力学或高保真车辆动力学；
- 所有数值均是未标定的 `simulation_candidate`，不能用于实车停车距离或高速安全声明；
- 五类场景可生成、可运行不等于五类任务已被规划器成功解决；
- tracker、预测、拓扑锁、走廊约束、Hybrid A*、Anchor Bank、predictive shield 和 SAC
  均未实现；
- Gazebo Classic 已 EOL，当前仍可复现但必须保留迁移风险；
- 不得借 V2 名义重跑或扩大历史 T12 预算。

## 下一实施门：V2-03

下一步实现 `nav_world_model` 的静态几何、动态跟踪/预测与健康状态，以及不读取场景标签的
规则监督器、模式滞回和 `BALANCED` fallback。V2-03 必须先以 Pedsim/Gazebo 真值仅作
evaluator 对照，给出跟踪 RMSE、ID switch、预测误差和模式混淆矩阵；仍不授权训练或实车闭环。
