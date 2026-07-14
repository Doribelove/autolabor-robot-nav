# TEB RL Thesis Current Handoff

更新时间：2026-07-14 00:54 CST

这是 `/home/robot/robot_ws_base_rl` 论文工作空间的当前主交接书。新 Codex
会话应先读本文件，再读 `DEVELOPMENT_STATUS.md`、实验合同和实验书。旧的 T00
首轮提示词及 `system_inventory.yaml` 已按用户要求删除；本文件承接当前有效事实。

## 1. 新会话启动顺序

```bash
cd /home/robot/robot_ws_base_rl
git status --short --branch
git remote -v
git submodule status
```

然后完整阅读：

1. `AGENTS.md`
2. `docs/thesis_experiment/CURRENT_TEB_RL_HANDOFF.md`
3. `docs/thesis_experiment/DEVELOPMENT_STATUS.md`
4. `docs/thesis_experiment/experiment_contract.yaml`
5. `docs/thesis_experiment/UBUNTU20_TEB_RL_EXPERIMENT_BOOK.md`
6. `docs/thesis_experiment/CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md`
7. `docs/thesis_experiment/CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`
8. `artifacts/t02/m2_gazebo_acceptance.yaml`
9. `artifacts/t02/m2_chassis_regression.yaml`
10. `artifacts/t02/m2_fixed_teb_regression.yaml`
11. `artifacts/t06/t04_t06_pipeline_acceptance.yaml`
12. `artifacts/t06/t04_t06_pipeline_run/run_manifest.yaml`
13. `artifacts/t07/training_environment_acceptance.yaml`
14. `artifacts/t07/training_environment_run/run_manifest.yaml`
15. `artifacts/t07/calibration_pilot/t07_calibration_report.yaml`
16. `config/thesis_experiments/A_TEB_v1.yaml`
17. `config/thesis_experiments/t08_baselines.yaml`
18. `artifacts/t08/evaluation/t08_evaluation_report.yaml`
19. `requirements/thesis-rl-lock.yaml`
20. `config/thesis_experiments/t09_sac.yaml`
21. `artifacts/t09/gazebo_sac_smoke/t09_gazebo_sac_smoke.yaml`
22. `config/thesis_experiments/t10_direct_theta_sac.yaml`
23. `artifacts/t10/gazebo_sac_smoke/t10_gazebo_sac_smoke.yaml`
24. `artifacts/t10/paired_sac_acceptance.yaml`

当前用户已确定另建 FAM-TEB V2 系统架构。完成第 6--7 项的 T12 冻结核验后，如任务涉及
新一代场景感知、多模式规划、动态预测、拓扑锁、狭窄通道或死角机动，必须继续完整阅读
`docs/thesis_experiment/V2_SYSTEM_GUIDE.md` 和
`docs/thesis_experiment/CURRENT_V2_FOUNDATION_HANDOFF.md`，随后阅读
`docs/thesis_experiment/CURRENT_V2_02_HANDOFF.md` 和
`docs/thesis_experiment/CURRENT_V2_03_HANDOFF.md`，最后阅读
`docs/thesis_experiment/CURRENT_V2_04_HANDOFF.md`，并以
`docs/thesis_experiment/CURRENT_V2_04F_HANDOFF.md` 作为当前最新入口。V2-04C Anchor Bank
已冻结；V2-04E--E4 已用独立 calibration seeds 冻结规则监督器；V2-04F 已完成 30/30 全新
held-out 三方法配对。三方法均 10/10 成功，Cruise/Static 混淆和 Maneuver 不触发已修复，
但净空、抖动、TTC 覆盖和相对 Fixed 的时间效率门仍失败。所有配置继续
`runtime_ready=false`，不授权 V2-05、训练、历史 T12 重跑、实车闭环或实车参数写入。

不要重新执行 T00--T11；T12 Residual SAC 两 seed 小预算训练已经完成，先按
`CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md` 和 `CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`
核对冻结报告与 checksum。学习性诊断、配对和 curriculum 单因素 pilot 已完成；episode-anchor
和 boundary atomicity 曾暴露 `move_base` SIGSEGV。最新 activation barrier + 静态 footprint
生命周期修复已完成两 seed 各 2000-step 且无 SIGSEGV，但 validation change 为
+0.4527/-0.0784，学习门失败，新三方法配对未启动。后续离线 projection 对齐诊断也已完成：
训练 projection 均值 67.3%，且安全干预后一步的 projection 均值 95.7%。严禁重复启动或扩大预算。不要清理、reset
或覆盖当前 dirty worktree。

## 2. 项目边界与安全要求

- 论文工作空间：`/home/robot/robot_ws_base_rl`
- 稳定实车工作空间：`/home/robot/robot_ws`
- 两个工作空间必须分别编译；一个终端只 source 一个工作空间的 `devel/setup.bash`。
- 论文开发只修改 `robot_ws_base_rl`，不得顺手修改稳定实车工作空间。
- 默认只允许 Gazebo、离线回放或 shadow mode。
- 未取得用户现场逐次许可，不得启动真实车辆运动。
- 未取得明确许可，不得对实车 `TebLocalPlannerROS/set_parameters` 写参数。
- 不得启动 `m2_driver`、访问 `/dev/ttyUSB*` 或 CAN 来完成论文仿真测试。
- NoSafety、ProjectionOnly、NoFallback 不允许实车闭环。
- T11 已生成 4-seed 完整缩减矩阵和描述性配对结果；原 5-seed 计划未完成，不得写成完整正式 5-seed 结论。

## 3. Git 与工作区状态

- 当前分支：`base_on_rl`
- 当前主仓 HEAD：`e9b9a9e17ac5e7e35d95eec2a7ad7c7667049da7`
- HEAD 描述：`e9b9a9e Update GPS navigation workspace`
- 远端：`git@github.com:Doribelove/autolabor-robot-nav.git`
- `base_on_rl` 尚未设置 upstream，尚未 push。
- 当前修改尚未 commit；不要擅自提交或推送。
- 工作树在论文工作开始前已有大量 dirty 文件和 dirty 子模块，必须保留。
- 已知重要子模块：
  - Arena：`634bcb091a90b362087cdba5a9cd3856466d493c`
  - TEB fork：`b4cf0639775e4521cdf7681158043ad3eef4b01a`
  - bundled SB3：`d47012ba8005177651f8597be14c5f1c34aeaa88`
  - FAST_LIO：`7cc4175de6f8ba2edf34bab02a42195b141027e9`
  - Livox driver：`6b9356cadf77084619ba406e6a0eb41163b08039`

当前 dirty 状态中包含 GPS 实车开发文件、Arena 子模块修改和论文新增文件。任何后续
改动都应限制在任务范围内，不得通过 `git reset --hard`、`git checkout --` 或批量
清理来“整理”工作树。

## 4. 已确认环境

- Ubuntu 20.04
- ROS Noetic `1.17.4`
- catkin `0.8.12`
- move_base `1.17.3`
- Gazebo Classic `11.15.1`
- gazebo_ros `2.9.3`
- Python `3.8.10`
- 工作空间 TEB fork package version `0.8.4`，优先于系统 TEB `0.9.1`
- 完整工作空间当前遍历 71 个 catkin 包并编译成功。

Gazebo Classic 已上游 EOL，但它仍是本论文当前合同指定的 Ubuntu 20.04/Noetic
仿真器，不要在没有论文范围变更的情况下迁移到新版 Gazebo。

## 5. 阶段完成情况

### T00：环境盘点——完成

已完成的事实性工作：

- 恢复了来自 `/home/robot/robot_ws` 的正确 Git 元数据，而不是重新 `git init`。
- 创建本地论文分支 `base_on_rl`。
- 核验工作空间、ROS、Gazebo、Python、catkin、TEB、dynamic_reconfigure 等版本。
- 找到实际 TEB fork、参数 YAML、DWA、Gazebo worlds、URDF/xacro 和 Arena 入口。
- 通过无运动探测确认：
  - `/move_base/TebLocalPlannerROS/set_parameters`
  - 类型 `dynamic_reconfigure/Reconfigure`
  - 参数描述和 update 话题存在。
- 交接书候选的 9 个 TEB θ 参数均存在于 fork cfg，并在运行时 callback 中赋值，
  因此具备在线更新入口。
- 未发送任何 dynamic_reconfigure 写请求。

旧 T00 首轮提示词及旧 inventory 文件已经删除，不应恢复或继续引用。

### T01：包骨架、schema、Python 隔离——完成

新增包：

- `src/application/teb_rl_tuner`
- `src/tools/thesis_experiment`

完成内容：

- `teb_rl_tuner` 提供论文调参包边界和 runtime defaults。
- `thesis_experiment` 提供合同、metric schema、manifest 的加载与校验 CLI。
- episode schema 当前 43 字段。
- step schema 当前 57 字段。
- T01 Python/配置 pytest：9 passed。
- `scripts/activate_thesis_env.sh` 清理外部 ROS/Python workspace 泄漏，并设置
  `PYTHONNOUSERSITE=1`。
- 已确认普通用户环境会误载 `/home/robot/catkin_ws` 的 SB3；严格论文环境会阻止它。

当时尚未完成的 RL freeze gate 已在 T09 关闭：严格论文 `.venv` 已冻结 CPU-only
Torch、Gymnasium 和 SB3；外部 CUDA Torch、旧 Gym 和 bundled SB3 均不属于论文 runtime。

相关文件：

- `requirements/rl_stack_status.yaml`
- `requirements/base.txt`
- `requirements/rl.in`
- `scripts/activate_thesis_env.sh`
- `docs/thesis_experiment/schemas/`
- `docs/thesis_experiment/templates/`

### T02：M2 Gazebo 接口和定量回归——完成

独立包：`src/simulation/m2_gazebo`

没有修改 Arena 的第三方机器人模型。当前包包含：

- M2 四轮 Ackermann URDF/xacro
- `base_link`、`chassis_link`
- 左右前轮转向 joint
- 四个车轮 joint
- collision、inertia
- `laser_link`、`imu_link`
- Gazebo laser 和 IMU
- 自定义 `libm2_ackermann_plugin.so`
- empty、obstacle、regression worlds
- spawn、底盘回归、固定 TEB、固定 TEB 回归 launch
- C++ 运动学测试和模型合同测试

#### T02 ROS 接口

仿真发布/订阅：

- `/scan`：`sensor_msgs/LaserScan`
- `/odom`：`nav_msgs/Odometry`
- `/cmd_vel`：`geometry_msgs/Twist`
- `/ackerman_vel`：`geometry_msgs/Twist`
- `/joint_states`：`sensor_msgs/JointState`
- `/tf`、`/tf_static`
- `/m2_driver/wheel_angle`：`std_msgs/Float64`
- `/m2_driver/left_wheel_vel`：`std_msgs/Float64`
- `/m2_driver/right_wheel_vel`：`std_msgs/Float64`
- `/m2_driver/brake_set`
- `/m2_driver/emergency_stop`
- `/m2_driver/reset_odom`
- `/m2_driver/steer_center_bias`

服务：

- `/m2_driver/chassis_parameter`
- 类型：`autolabor_canbus_driver/ChassisParameterServer`
- 当前 checked-in srv 请求体为空，源码行为是只查询，不支持说明文字所称的设置。

导航侧已验证：

- `/move_base/status`：`actionlib_msgs/GoalStatusArray`
- `/move_base/TebLocalPlannerROS/global_plan`：`nav_msgs/Path`
- `/move_base/TebLocalPlannerROS/local_plan`：`nav_msgs/Path`
- `/move_base/local_costmap/costmap`：`nav_msgs/OccupancyGrid`
- `/move_base/TebLocalPlannerROS/set_parameters`：`dynamic_reconfigure/Reconfigure`

TF 规则：

- 仿真唯一移动 TF：`odom -> base_link`
- `base_link -> laser_link`
- `base_link -> imu_link`
- 不伪造 `/gps/odom`
- 已修复早期 `base_footprint` 与 `base_link` 双父节点问题。

#### T02 控制语义

- `/cmd_vel.linear.x`：目标线速度 m/s
- `/cmd_vel.angular.z`：目标车体 yaw rate rad/s
- `cmd_angle_instead_rotvel=false`
- `/ackerman_vel.angular.z`：直接中心转向角 rad
- Twist 转转角：`atan(wheelbase * angular_velocity / signed_linear_velocity)`
- 已处理零速/低速、饱和、倒车符号、左右 Ackermann 转角、后轮电子差速和
  0.5s command timeout。

重要源码差异：实车 `m2_driver.cpp` 使用 `abs(target_vel)` 做 `/cmd_vel` 换算，
仿真使用有符号速度以保持倒车时目标 yaw rate 的符号。该差异必须在低速实车阶段
验证，不能把仿真行为直接声明为实车事实。

#### T02 候选参数

唯一来源：`src/simulation/m2_gazebo/config/simulation_candidates.yaml`

所有结构和动力学值均明确标记：

```yaml
status: simulation_candidate
calibrated: false
```

当前主要候选值：

- 车体：1.04m × 0.70m × 0.43m
- 质量：80kg
- 轴距：0.65m
- 轮距：0.60m
- 最小转弯半径：1.20m
- 最大速度：2.778m/s
- 中心最大转角（推导）：约 0.4964rad
- 轮径候选：0.15m，仅来自说明文档举例，不是实车测量值
- 激光 z 候选：0.60m
- IMU z 候选：0.40m

这些值不能写成最终实车标定结果。

#### T02 底盘回归：11/11

机器报告：`artifacts/t02/m2_chassis_regression.yaml`

运行命令：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch m2_gazebo m2_regression.launch \
  run_test:=true gui:=false seed:=42 \
  report_path:=/home/robot/robot_ws_base_rl/artifacts/t02/m2_chassis_regression.yaml
```

测试覆盖：

1. spawn 和 reset
2. 静止 3s 不滑动、不倾倒
3. 直行 5m
4. 直行 10m
5. 0.2m/s 低速倒车
6. 左 1.5m 半径整圆
7. 右 1.5m 半径整圆
8. 停止响应时间和距离
9. 固定障碍激光测距
10. TF 无环、父级和时间戳
11. seed=42 三次 reset 重复性

关键实测：

- 5m 误差：0.04m
- 10m 误差：0.04m
- 左右整圆闭合误差：均约 0.01922m
- 停止响应：0.012s
- 运动学制动距离：0.0m
- 激光量距误差：约 0.00267m
- TF 最新时间戳年龄：0.014s
- TF cycle：false
- base_link parent：odom
- 三次 reset 位姿跨度：0

制动距离 0m 只说明运动学插件会即时执行停止命令，不是物理制动模型，不得用于
论文实车制动性能结论。

#### T02 固定 TEB 回归：5/5

机器报告：`artifacts/t02/m2_fixed_teb_regression.yaml`

运行命令：

```bash
roslaunch m2_gazebo m2_fixed_teb_regression.launch \
  gui:=false seed:=42 \
  report_path:=/home/robot/robot_ws_base_rl/artifacts/t02/m2_fixed_teb_regression.yaml
```

场景：

1. 4m 直线目标
2. 左转目标 `(3, 3, pi/2)`
3. 右转目标 `(3, -3, -pi/2)`
4. 5m 单障碍绕行
5. 1.8m 窄通道

最终五项 action 均为 `SUCCEEDED`，且：

- 所有场景均收到 global plan 和 TEB local plan
- 最大 `/cmd_vel` 间隔：约 0.116s，阈值 0.5s
- planner error count：0
- control deadline miss count：0
- 单障碍最小 scan range：约 0.6775m
- 窄通道最小 scan range：约 0.8495m

固定 T02 仿真基线将 `enable_homotopy_class_planning=false`，原因是长墙的密集
LaserScan 点会让多同伦优化器产生约 1s 级控制超期。关闭后仍由 Navfn 全局路径完成
单障碍绕行，最终零 planner error、零 deadline miss。这是确定性 T02 baseline，
不是对论文所有后续基线/消融配置的永久决定。

### T03：TEB dynamic_reconfigure 参数客户端——完成

实现位于 `src/application/teb_rl_tuner`，核心文件为
`teb_parameter_client.py`、`safety_gate.py`、Gazebo 验收脚本/launch 和对应测试。

已实现并验证：

- 四重 fail-closed 写门控：调用方显式 `simulation=true`、`/use_sim_time=true`、
  `/m2_gazebo/simulation_only=true`、命名空间精确为
  `/move_base/TebLocalPlannerROS`；缺一即拒写。
- 启动时读取参数描述和当前配置，验证论文合同 9 个参数的名称、`double` 类型、
  dynamic_reconfigure min/max、当前值及有限数值。
- 每次应用必须精确包含全部 9 个参数，并只调用一次 `Reconfigure` 服务；缺失、
  多余、错误类型、NaN/Inf、越界均在写前拒绝。
- 服务返回 ack 后等待新的 `parameter_updates`，分别核对 request、ack、readback，
  记录 `config_seq`、墙钟 request/ack/readback 时间和端到端延迟。
- 描述、当前值、服务和 readback timeout，以及 ack/readback mismatch 均进入
  `interface_fault` 路径并尝试恢复启动快照。
- 正常退出、上下文退出和 ROS shutdown 都恢复启动快照；Gazebo 验收确认最终值与
  启动值完全一致。

机器报告：`artifacts/t03/teb_parameter_client_acceptance.yaml`。seed=42 headless
Gazebo 结果为 `passed: true`，一次 9 参数事务 request/ack/readback 一致，快照恢复
成功。当前 fork 报告的接口范围是：速度/加速度 4 项 `[0.01, 100]`、
`min_obstacle_dist` `[0, 10]`、`inflation_dist` `[0, 15]`、3 个优化权重
`[0, 1000]`，均为 `double`。这些只是 dynamic_reconfigure 接口范围，不是物理可行
范围、正式仿真动作范围或实车安全范围。

验收 probe 来自 `config/t03_simulation_probe.yaml`，明确标记
`simulation_probe_only`、`calibrated: false`。没有修改实车 TEB YAML，没有启动
`m2_driver`，没有访问串口/CAN，没有连接实车 ROS master，也没有真实车辆运动。

### T04：状态、时间窗、奖励和 episode manager——完成

- `timing.py`：ROS 时间流单调/同步、`t_ack -> t_active` 和激活超时；
- `state_builder.py`：36 扇区低分位激光、命名特征和严格 K=4 历史窗口；
- `reward_cost.py`：只归因 `t_active` 后反馈，以物理时间积分并分别输出全部
  reward/CMDP cost 分量；
- `episode_state_machine.py`：termination/truncation 互斥、唯一结束原因，activation
  timeout 丢弃 transition 并进入 `interface_fault`。

### T05：投影、安全过滤和保守回退——完成

- 精确要求 9 个 theta；错误键/类型、bool、NaN/Inf 在写前拒绝；
- 完整 box、单步变化率、在线支持、TEB/Ackermann 耦合投影，耦合修正破坏边界时拒绝；
- `NORMAL/WARNING/EMERGENCY/FAULT` 四态，连续裕度、即时升级、滞回和健康恢复；
- emergency/fault 返回完整 9 参数保守快照；接口故障只使用最后一次 ack/readback
  确认安全快照，否则 fail closed。

`config/t05_simulation_safety.yaml` 仅供 T02 Gazebo pipeline 使用，明确
`calibrated: false`、`real_vehicle_use_forbidden: true`，不是实车安全配置。

### T06：CSV、manifest、checksum 和 run validator——完成

- 按冻结顺序原子写 43 字段 episode CSV 和 57 字段 step CSV；
- required/type/enum/JSON 和 float NaN/Inf fail-closed；
- 原子写 manifest，生成 SHA256；validator 检查 schema、hash、引用、ID/seq 单调、
  episode 唯一性和结束语义；rosbag 仅记录 URI。

统一证据：`artifacts/t06/t04_t06_pipeline_acceptance.yaml`。固定 TEB 以 `goal` 完成
1.5m episode；K=4、堆叠状态 244 维；`t_request < t_ack < t_active`；窗口记录 9 个
规划周期。T05 normal 路径不修改参数，emergency dry probe 选择完整 9 参数回退且
未写入 TEB。T06 bundle 为 1 episode/1 step/2 checksums，validator 为 `valid: true`，
明确排除正式结果；启动快照恢复成功。

### T04--T06 长期训练环境扩展——完成

- `TrainingEnvironment` 提供无 Gym 强依赖的 `reset(seed)` / `step(action)` 接口、固定策略和
  随机小扰动策略，可由后续 SAC wrapper 直接复用；
- Gazebo adapter 在一个持久仿真进程中完成多 episode reset、时间因果检查、参数事务、
  reward/termination 归因、CSV 写入和 episode 后快照恢复；
- 激活超时、action result 与 reward window 边界均 fail closed，不保存不合法 transition；
- 纯 Python fake adapter 连续 300 steps 通过；Gazebo 验收完成 5/5 episode、18 steps，
  config sequence 每 episode 重新开始，所有 transition 入库，validator 为 `valid: true`。

证据：`artifacts/t07/training_environment_acceptance.yaml` 和
`artifacts/t07/training_environment_run/run_manifest.yaml`。Gazebo 长时验收使用确定性
`FixedPolicy`；`RandomSmallPolicy` 已单元测试，但尚未作为正式算法组运行。

### T07：Gazebo 局部敏感性标定与 A_TEB——完成

- checked-in 场景合同覆盖 obstacle/clear/corridor 三种几何、2 seeds 和每个 theta 的
  baseline/正扰动/负扰动，共 114 个计划 run；
- 本轮 pilot 实际执行每场景 1 seed，共 57 个 Gazebo episode；56 success，1 个
  oscillation/failure 原样保留，没有因结果不佳删除数据；
- 生成 81 条 sensitivity observation；每个矩阵项都有 source file/line/hash、正负扰动
  配对、中心差分和符号一致性证据，不完整 pair 数为 0；
- 依据非零已解析效应的幅值中位数和 top-k sparsity 冻结 9x5 `A_TEB_v1`；15 个非零项，
  30 个稀疏零项；
- frozen mapping：`config/thesis_experiments/A_TEB_v1.yaml`；canonical SHA256：
  `1ca660f8d4f1863a93d75686bc0cafe8259942aaac60c3e2817c31162fcb1000`；
- 标定报告：`artifacts/t07/calibration_pilot/t07_calibration_report.yaml`；原始 CSV：
  `artifacts/t07/calibration_pilot/sensitivity_observations.csv`。

该映射只冻结当前 Gazebo 训练合同，不是实车映射。合同仍保留第二 seed 的 57 个 run，
后续可作为扩展复核，不影响当前 pilot 的正负扰动完整性。

### T08：固定/规则基线与统一 evaluator——完成

- 冻结合同 `config/thesis_experiments/t08_baselines.yaml`，算法顺序为 Fixed-DWA、
  TEB-Default、TEB-Tuned、Rule-TEB；TEB-Default 是配对参考组；
- Rule-TEB 是只读取当前观测与自身上一模式的三态滞回策略，不读取 goal result、未来轨迹、
  episode 汇总或 evaluator 统计；候选仍经过 T05 投影/安全层；
- T06 validator 扩展为显式 `scene_ids` 多场景 bundle，未声明场景仍 fail closed；
- 统一 evaluator 强制完整 algorithm×scene×seed 矩阵，拒绝缺失、重复、未知组、NaN/Inf
  和 success/termination 不一致，并对 TEB-Default 计算同 scene/seed 配对差值；
- 实际 validation pilot 为 4 算法×3 场景×seed 42，共 12 episode；全部源 bundle 和统一
  evaluator 均 valid，失败样本没有删除；
- 结果：TEB-Default 3/3、TEB-Tuned 3/3、Rule-TEB 2/3（1 safety emergency stop）、
  Fixed-DWA 1/3（1 collision、1 timeout）。

统一证据为 `artifacts/t08/evaluation/t08_evaluation_report.yaml`，合并 CSV 和 checksums
在同目录。`TEB-Tuned` 明确是 T07 参数范围中点工程基线，不是优化所得最优参数。
本轮为 simulation validation pilot、单 seed，不可作为正式论文性能结论。

### T09：Semantic-Eta SAC——完成

- 严格 `.venv` 冻结 Python 3.8.10、Torch 2.4.1+cpu、Gymnasium 1.0.0、SB3 2.4.1；
  `PYTHONNOUSERSITE=1`，CUDA false，禁止外部 catkin/arena workspace import；
- `requirements/thesis-rl-lock.txt` 冻结版本，`thesis-rl-lock.yaml` 冻结关键 distribution
  RECORD hashes；`scripts/verify_rl_stack.py` 检查版本、来源、hash 和 CPU 边界；
- `FrozenSemanticMapping` 实现 5D `delta_eta`、累计 eta、`A_TEB` normalized-theta 映射、
  物理反归一化和 clipping audit；映射后仍进入 T05 投影与安全过滤；
- 为与 T10 公平配对，Gymnasium wrapper 输出 244D T04 core state + 上一步实际
  normalized-theta delta 9D + L1 1D = 254D；动作空间为 `Box[-1,1]^5`；
  VecNormalize statistics 只由训练更新并随 checkpoint 保存；
- SAC checkpoint 保存 model、replay buffer、VecNormalize 和 manifest hashes；加载时拒绝
  文件 hash、Python/Torch/Gymnasium/SB3/Numpy 或 device 不一致；
- Gazebo smoke 训练 16 steps，保存并恢复后继续 4 steps；replay buffer 20，actor 参数
  L1 change 11.790；20/20 transition 存储；训练 8 episodes 全部 goal；确定性评估 2/2 goal；
- 证据：`artifacts/t09/rl_stack_validation.yaml`、
  `artifacts/t09/gazebo_sac_smoke/t09_gazebo_sac_smoke.yaml` 和 checkpoint 目录。

T09 是实现与短 smoke 验收，不是正式收敛训练。上述正式合同已在 T11 冻结并执行。

### T10：Direct-Theta SAC 公平对照——完成

- `DirectThetaMapping` 接受 9D `delta_normalized_theta`，按冻结 theta 顺序在归一化空间
  累加、裁剪和反归一化；候选继续进入与 T09 相同的 T05 投影、安全过滤和回退；
- T09/T10 共用 `GazeboSacSmoke`，场景顺序、ROS 环境、reward、termination、seed、SAC
  参数、20-step 预算、VecNormalize 和 checkpoint/resume 实现相同；
- `load_and_validate_sac_pair` fail closed 比较八个公共配置块，动作空间之外的漂移会拒绝；
- Direct smoke：16 steps 保存、恢复后继续 4 steps，20/20 transition 存储，训练 7 episodes
  全部 goal，确定性评估 2/2 goal；
- Semantic/Direct actor 参数量为 9546/9810，trainable 参数量为 47182/47958；差异仅来自
  5D/9D SAC 输出；短 smoke 推理时延均值为 1.193/1.363 ms，不用于论文性能结论；
- 配对报告 `artifacts/t10/paired_sac_acceptance.yaml` 为 `passed: true`；两侧 checkpoint
  文件 SHA256、算法身份和 20 timesteps 均验证通过；
- T10 后固定 TEB 回归 5/5，报告为 `artifacts/t10/m2_fixed_teb_post_t10_regression.yaml`。

T10 仅证明 Direct-Theta 对照已实现且管线公平。T11 缩减矩阵的描述性结果没有显示
Semantic-Eta 相对 Direct-Theta 的任务成功率优势，且两者均未达到冻结收敛阈值。

### T11：多场景、多 seed 训练与安全消融——完成（预算修订）

- 原预注册为 5 seeds、25 runs、1750 test episodes；用户报告额度不足后保留原合同，
  通过 `experiments/manifests/t11/budget_amendment.yaml` 将主分析限定为 seed101--104；
- 主矩阵为 4 seeds×5 groups×70 test episodes = 20 runs/1400 episodes；全部 manifest、
  checksum 和 RunValidator 通过，四组 paired comparison 各 280 对；
- seed105 已完成 Semantic、Direct、ProjectionOnly 三组，只作 supplementary；缺失的
  NoSafety/NoFallback 不插补；
- 主矩阵成功率为 Semantic 42.86%、Direct 43.21%、ProjectionOnly 90.00%、NoSafety
  90.71%、NoFallback 42.86%，记录碰撞率均为 0；
- FullSafety 的 160/280 test episodes 由 emergency stop 终止，表明安全门限过度保守；
- NoSafety seed104 保留一次 move_base SIGSEGV fatal attempt，因此高任务成功率不能解释为更安全；
- 8 个主训练 run 均未达到 validation return 阈值 10，500-step 预算下不得宣称收敛；
- 证据：`artifacts/t11/evaluation/t11_evaluation_report.yaml`、
  `artifacts/t11/evaluation/T11_REDUCED_STUDY_SUMMARY.txt` 和
  `artifacts/t11/t11_reduced_study_checksums.sha256`。

## 6. 当前构建和测试状态

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4
source devel/setup.bash
catkin_make run_tests_m2_gazebo -j4 -l4
catkin_test_results build/test_results/m2_gazebo
catkin_make run_tests_teb_rl_tuner -j2 -l2
catkin_test_results build/test_results/teb_rl_tuner
catkin_make run_tests_thesis_experiment -j2 -l2
catkin_test_results build/test_results/thesis_experiment
roslaunch teb_rl_tuner t03_teb_client_acceptance.launch gui:=false seed:=42
roslaunch thesis_experiment t04_t06_pipeline_acceptance.launch gui:=false seed:=42
roslaunch thesis_experiment long_training_environment_acceptance.launch \
  gui:=false seed:=42 episode_count:=5
rosrun thesis_experiment validate_run.py \
  artifacts/t06/t04_t06_pipeline_run/run_manifest.yaml
rosrun thesis_experiment validate_run.py \
  artifacts/t07/training_environment_run/run_manifest.yaml
source scripts/activate_thesis_env.sh
python -m pytest -q \
  src/application/teb_rl_tuner/tests \
  src/tools/thesis_experiment/tests
python scripts/verify_rl_stack.py --output artifacts/t09/rl_stack_validation.yaml
roslaunch thesis_experiment t09_gazebo_sac_smoke.launch gui:=false seed:=42
roslaunch thesis_experiment t10_gazebo_sac_smoke.launch gui:=false seed:=42
rosrun thesis_experiment evaluate_t10_sac_pair.py
```

最近结果：

- full `catkin_make`：成功，71 packages
- m2_gazebo：5 个 C++ Ackermann tests + 4 个模型合同 tests 通过
- catkin_test_results 对 m2_gazebo：0 errors，0 failures
- T01/T03 与配置 pytest：23 passed
- teb_rl_tuner catkin tests：20 tests，0 errors，0 failures（含 Gazebo rostest）
- T03 验收 YAML：`passed: true`，9 参数原子事务和快照恢复通过
- T03 后固定 TEB 回归：5/5，通过，报告为
  `artifacts/t03/m2_fixed_teb_post_t03_regression.yaml`
- T01--T12 Python/配置 pytest：135 passed；TEB 原生测试 6 tests、0 failures
- teb_rl_tuner catkin：54 tests，0 errors，0 failures
- thesis_experiment catkin：11 tests，0 errors，0 failures（含 T04--T06 Gazebo rostest）
- T04--T06 bundle validator：1 episode、1 step、2 checksums，`valid: true`
- T06 后固定 TEB 回归：5/5，报告为
  `artifacts/t06/m2_fixed_teb_post_t06_regression.yaml`
- 长期训练环境：5 episodes、18 steps、3 checksums，validator `valid: true`
- T07 calibration：57 Gazebo episodes、81 observations、56 success/1 failure retained；
  `A_TEB_v1` 已冻结并通过 `--require-frozen` 校验
- T07 后固定 TEB 回归：5/5，报告为
  `artifacts/t07/m2_fixed_teb_post_t07_regression.yaml`
- T08：四个 3-episode bundles 均 valid；统一 12-episode paired evaluator `passed: true`
- T08 后固定 TEB 回归：5/5，报告为
  `artifacts/t08/m2_fixed_teb_post_t08_regression.yaml`
- T09 RL stack verifier：valid；pip check：无 broken requirements
- T09 Gazebo SAC smoke：20 steps、checkpoint resume、2/2 deterministic evaluation，passed
- T09 后固定 TEB 回归：5/5，报告为
  `artifacts/t09/m2_fixed_teb_post_t09_regression.yaml`
- T10 Gazebo Direct-Theta smoke：20 steps、checkpoint resume、2/2 deterministic evaluation，passed
- T10 paired evaluator：公共配置/预算/observation/checkpoint 全部一致，passed
- T10 后固定 TEB 回归：5/5，报告为
  `artifacts/t10/m2_fixed_teb_post_t10_regression.yaml`
- T11 缩减矩阵：20/20 主 run、1400/1400 test episodes，正式 evaluator passed
- T12 T11 遥测离线 shadow：280 episodes、991 steps，验收 passed；安全误停候选
  131/160，动作 L1 经 EMA+投影降低 62.24%，报告在
  `artifacts/t12/offline_replay/t12_replay_report.yaml`
- T12 无训练闭环复验：3 方法 × 2 checkpoint seeds × 10 场景 = 60 episodes；旧 FullSafety
  40% success/60% emergency，T12Safety 90% success/10% emergency，ProjectionOnly 95%
  success 且有 1 次 planner failure；三组 collision 均为 0。T12Safety 无 planner/interface
  fault，正式门槛 passed，报告在 `artifacts/t12/closed_loop/t12_closed_loop_report.yaml`
- T12 窄走廊 + Residual pilot：3方法 × 2 seed × 4困难场景 = 24 episodes，不训练；legacy
  3/8 goal、5 emergency，directional 5/8 goal、0 emergency/碰撞/planner failure，zero-residual
  6/8 goal、0 emergency/碰撞、1 navigation abort。有效 run 无进程崩溃、动态参数失败或
  interface fault，acceptance passed；报告在
  `artifacts/t12/residual_pilot/t12_residual_pilot_report.yaml`。开发期 SIGSEGV 尝试已保留，
  不属于最终 bundle。
- T12 Residual SAC 两 seed 小预算训练已经完成。seed101 validation 为 24.4235（1000）
  和 24.1838（2000），seed102 为 23.5964（1000）和 23.4201（2000）；两者均按
  validation-only 规则选择 1000-step。合计 14/14 test goal，0 collision/emergency/
  planner/interface fault、0 crash，run validator/checksum 通过。两个 seed 的 validation
  趋势均下降，因此只通过完整性门槛，`formal_result: false`，不得宣称学习增益或直接扩预算。
- 后续离线诊断发现原训练 curriculum 索引每 episode 被重置，实际只访问第一个 clear 场景；
  24-episode selected/zero-residual/TEB-Tuned 配对中三组均 50% success，selected 仅有 return
  优势而无成功率/效率优势。curriculum 单因素修复后两 seed 均覆盖全部 5 场景且 14/14 test
  goal，但 validation change 为 -0.2487/+0.0452，cross-seed mean -0.1018，学习门槛仍失败。
  修复后 projection intervention rate 仍为 87.06%。episode-anchor 第二单因素已冻结并启动，
  但 seed101 两次 bounded attempt 均在 episode reset 参数恢复边界触发 `move_base` -11。
  boundary atomicity 修复后 500-step pilot 为 0 crash、7/7 test goal，projection 59.8%；首个
  2000-step 尝试仍在在线 activation 阶段 -11。后续 activation-timeout barrier 与静态 footprint
  生命周期修复通过 1300-step stress，并完成 seed101/102 各 2000-step：0 crash/SIGSEGV，
  14/14 test goal，validation change +0.4527/-0.0784，projection 65.1%/69.5%。系统崩溃链已
  修复，但学习门仍失败，新配对未授权。详见
  `docs/thesis_experiment/CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`。

已知构建警告来自既有第三方/旧包，包括 `gazebo_ros_2Dmap_plugin` 包名规范、
Gazebo Classic EOL、部分 VTK 缺失 target 等；本次 T02 没有通过修改第三方包来消除。

## 7. 仍未解决的标定与技术债

实车标定 TBD：

- 有效轮径和滚动半径
- 激光/IMU 安装外参
- 轴距、轮距、footprint 实测
- 最小转弯半径的定义与转角极限
- 正向/倒车 `/cmd_vel` 符号语义
- 最大速度和转向饱和
- 转向响应延迟和转向速率限制
- 制动减速度、停车距离和端到端延迟
- `ChassisParameter.robot_length` 字段到底表示车长还是运动学轴距
- emergency stop 实车链路

仿真技术债：

- 当前为确定性运动学模型，不是轮胎力学/悬挂模型。
- 没有真实转向执行器滞后、噪声、打滑、坡度、制动动力学。
- Gazebo collision 不负责物理阻停；安全评价主要依赖 scan/costmap/planner。
- T02 报告只证明候选接口和固定 baseline 可重复，不证明 sim-to-real 有效。

Python/RL 技术债：

- RL stack 已冻结；Semantic-Eta/Direct-Theta SAC、replay buffer、checkpoint/resume 和确定性推理已配对 smoke-test。
- T12 已实现独立、无副作用的 shadow runtime、CSV 遥测回放和 60-episode 无训练闭环
  Gazebo 验证；原始 rosbag 适配与 ROS live shadow 节点仍待真实数据接口确认。
- T11 已有 episode/step CSV、缩减 seed sweep 和安全消融描述性结果；统计推断仍属于 T14。

## 8. T12 后续范围

T12 Residual SAC 离线诊断、历史 selected/zero-residual/TEB-Tuned 冻结配对、curriculum、
boundary atomicity、activation barrier 与静态 footprint 生命周期单因素均已有冻结证据。
最新两 seed 2000-step 系统运行完整，但 seed102 validation 下降且 projection 仍为
65.1%/69.5%。离线 action/projection 审查已完成：动作饱和与 radius 利用率均不高，projection
由 Ackermann 耦合边界和安全 WARNING 后回指 anchor 的 rate-limit 共同造成。当前禁止重启、
扩大预算或启动新三方法配对；只能先冻结一个动作—执行对齐学习因素 amendment。
原始 rosbag 适配和 ROS live shadow 节点仍待实现；默认不得写实车 TEB 参数或发布实车运动命令。

## 9. 下次 Codex 的最短接手提示

```text
你正在 /home/robot/robot_ws_base_rl 继续 Autolabor M2 论文系统。
先读 AGENTS.md、docs/thesis_experiment/CURRENT_TEB_RL_HANDOFF.md、
DEVELOPMENT_STATUS.md 和 experiment_contract.yaml。T00--T11 已完成，不要重做。
T12 静态 footprint 生命周期修复后的两 seed 2000-step pilot 已完成。先读
CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md 和 CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md，
核对冻结报告/checksum，禁止重复启动或扩预算。validation change 为 +0.4527/-0.0784，
学习门失败且新配对未授权；离线 Residual action/projection 诊断已完成，下一步是先预注册
一个且仅一个动作—执行对齐学习因素 amendment，尚未授权开跑。
不得删除失败样本，不得把 Gazebo 值写成实车结论，不得启动实车运动或向实车写 TEB 参数，
不得清理 dirty worktree。
```
