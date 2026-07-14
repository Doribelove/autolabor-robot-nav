# Ubuntu 20.04 / robot_ws：TEB 安全强化学习系统开发与论文实验交接书

版本：v1.0  
日期：2026-07-11  
目标工作空间：`/home/robot/robot_ws_base_rl`  
目标系统：Ubuntu 20.04、ROS Noetic、Gazebo 11、Autolabor M2、move_base + TEB

---

## 0. 本交接书的用途

本文件不是一般性的开发建议，而是论文方法、ROS/Gazebo 实现、M2 实车验证和论文结果回填之间的统一实验合同。Ubuntu 端 Codex CLI 应以本文件为主任务书，并同时阅读：

1. `/home/robot/robot_ws_base_rl/AGENTS.md`；
2. `/home/robot/robot_ws_base_rl/CURRENT_GPS_DEV_HANDOFF.md`；
3. `/home/robot/robot_ws_base_rl/README.md`；
4. `docs/thesis_experiment/experiment_contract.yaml`；
5. `docs/thesis_experiment/schemas/episode_metrics_schema.csv`；
6. `docs/thesis_experiment/schemas/step_metrics_schema.csv`。

本项目最终必须形成“论文提出什么，代码实现什么，实验记录什么，统计脚本证明什么”的闭环。任何未由原始 CSV、rosbag、配置快照和统计脚本支持的数值，不得回填论文。

---

## 1. 不可改变的论文主线

### 1.1 可验证主命题

> 通过低维语义参数空间和安全投影，使强化学习调参在保持安全约束的同时，提高 TEB 在多类场景中的导航效率和参数适应能力。

### 1.2 研究边界

- 唯一学习调参对象：TEB。
- 唯一核心实车平台：Autolabor M2。
- 主要验证环境：Gazebo 仿真 + M2 实车。
- DWA：只作为固定参数传统基线，不训练 RL-DWA。
- 策略不得直接发布 `/cmd_vel`，不得替代 TEB 或 M2 底盘控制器。
- 当前不研究跨规划器共享策略、多机器人迁移、端到端底盘控制。
- 安全模块统一称为“基于安全裕度的参数安全过滤器”，不得宣称已经获得严格 CBF 前向不变性保证。

### 1.3 四个必须回答的实验问题

1. 直接调节原始 TEB 参数增量 `Δθ` 与调节低维语义增量 `Δη` 相比，样本效率、OOD 泛化和可解释性有何差异？
2. 无安全层、仅参数投影、完整安全过滤分别降低多少风险，带来多少效率代价？
3. 固定参数、规则调参和 RL 调参之间是否存在稳定、可重复的性能差异？
4. 回退机制是否减少不可恢复失败，是否引入过度保守或模式振荡？

如果某个假设没有被数据支持，应如实报告，不允许通过更换测试集、挑选最好种子或删除失败 episode 得到预期结论。

---

## 2. 已确认的 robot_ws 实车基础

当前真实工程主链为：

```text
双天线 GNSS / Livox MID360 / FAST_LIO
  -> /gps/fix, /gps/heading, /gps/odom, /scan
  -> camera_init -> base_link TF
  -> move_base + TEB
  -> /cmd_vel
  -> m2_driver
```

已确认的重要事实：

- 工作空间：`/home/robot/robot_ws_base_rl`。
- ROS：Noetic；构建：catkin。
- 实车一键入口：`./scripts/bringup.sh gps` 或 `./scripts/bringup.sh fast_lio`。
- GPS 模式定位：`/gps/odom`，双天线航向：`/gps/heading`。
- 障碍物输入：Livox/FAST_LIO 点云处理后发布 `/scan`。
- 主天线相对 `base_link` 候选偏置：`x=-0.3 m, y=0.0 m`。
- GPS 模式默认 `GPS_USE_WHEEL_ODOM=false`。
- M2 驱动把 `/cmd_vel.angular.z` 解释为角速度并内部换算转角，所以 TEB 必须保持 `cmd_angle_instead_rotvel=false`。
- 用户提供的硬件最小转弯半径候选值为 `1.2 m`，必须通过低速标定复核；该几何量不得作为 RL 在线动作。
- 当前候选 TEB 值包括 `max_vel_theta=1.5`、`acc_lim_theta=0.5`、`min_turning_radius=1.2`、`min_obstacle_dist=0.3` 等；它们是工程起点，不是论文最优参数。
- 已有电子围栏、急停流程、GPS 静态误差监测和 rosbag 记录基础。
- 曾观察到约 5 cm RMS、12 cm 最大静态漂移，但在论文工作区没有对应原始 CSV，因此必须重新采集后才能成为正式结果。

### 2.1 Ubuntu 端第一条命令

Windows 交接副本中的多个子模块目录可能为空。Ubuntu 端首先执行：

```bash
cd /home/robot/robot_ws_base_rl
git status --short --branch
git submodule update --init --recursive
```

禁止使用 `git reset --hard`、`git clean -fd` 或覆盖已有用户改动。先记录当前 commit、dirty 状态和子模块 commit，再开始开发。

---

## 3. 目标系统总体架构

```text
ROS 观测与历史窗口
  -> state_builder
  -> s_t
  -> policy (fixed / rule / SAC-direct-theta / SAC-semantic-eta)
  -> a_t = Δη_t 或 Δθ_t
  -> semantic_adapter（仅 η 组）
  -> θ_candidate
  -> parameter_projection
  -> safety_margin_filter
  -> fallback_policy
  -> atomic_teb_parameter_writer
  -> TEB
  -> local_plan / cmd_vel / odom / scan / status
  -> reward_and_cost_window
  -> transition + step log
```

建议采用“纯 Python 核心库 + 少量 ROS 薄节点”的结构。不要把策略、投影、安全规则、日志和 dynamic reconfigure 全部写进一个无法单元测试的脚本。

### 3.1 建议新增包

```text
src/application/teb_rl_tuner/
  CMakeLists.txt
  package.xml
  launch/
  config/
  scripts/
  src/teb_rl_tuner/
    state_builder.py
    semantic_adapter.py
    parameter_projection.py
    safety_margin_filter.py
    fallback_policy.py
    reward_cost.py
    episode_state_machine.py
    teb_parameter_client.py
    policies/
      fixed_policy.py
      rule_policy.py
      sac_policy.py
      direct_theta_policy.py

src/tools/thesis_experiment/
  launch/
  scripts/
    experiment_manager_node.py
    metrics_logger_node.py
    export_thesis_bundle.py
    validate_run.py
  config/

src/simulation/m2_gazebo/            # 仅在现有子模块没有可复用 M2 模型时创建
  urdf/
  worlds/
  launch/
  config/
```

ROS 运行时建议只有以下主节点：

- `teb_rl_runtime_node`：1 Hz 左右的调参闭环，调用纯 Python 模块；
- `experiment_manager_node`：episode reset、场景/目标、终止与截断；
- `metrics_logger_node`：step/episode 数据与事件日志；
- `gazebo_scenario_manager_node`：仅仿真使用，负责确定性 reset 与随机化。

实车默认启动模式必须为 `shadow`，策略只计算和记录参数建议，不写入 TEB。只有显式设置 `ALLOW_MOTION=1` 和 `ALLOW_PARAMETER_WRITE=1`，并通过实车检查表后，才允许闭环写参数。

---

## 4. ROS 接口合同

### 4.1 必需输入

| 信息 | 仿真建议话题 | M2 实车话题 | 用途 |
| --- | --- | --- | --- |
| 激光/障碍物 | `/scan` | `/scan` | 最近距离、密度、TTC、状态特征 |
| 里程计 | `/odom` 或仿真别名 | `/gps/odom` / `/Odometry` | 位姿、速度、进度 |
| 全局路径 | `/move_base/*/global_plan` | 同左 | 路径偏差、局部目标 |
| TEB 局部轨迹 | `/move_base/TebLocalPlannerROS/local_plan` | 同左 | 参数生效检测、规划状态 |
| 控制命令 | `/cmd_vel` | `/cmd_vel` | 平滑度、响应分析，不由 RL 发布 |
| move_base 状态 | `/move_base/status` | 同左 | 成功、失败和终止判断 |
| 局部代价地图 | `/move_base/local_costmap/costmap` | 同左 | 可选的障碍物与可行域特征 |
| GPS 航向质量 | 可模拟 | `/gps/heading` | 实车输入有效性与故障判断 |
| TF | 仿真固定树 | `camera_init -> base_link` | 坐标统一 |

Ubuntu 端必须先通过 `rostopic list`、`rostopic info` 和 `rosmsg info` 确认实际全局/局部路径话题名。不得在代码中假设 Arena fork 与标准 move_base 完全相同。

### 4.2 建议输出命名空间

```text
/thesis_teb_rl/state_summary
/thesis_teb_rl/policy_action
/thesis_teb_rl/parameter_candidate
/thesis_teb_rl/parameter_applied
/thesis_teb_rl/safety_status
/thesis_teb_rl/episode_event
/thesis_teb_rl/diagnostics
```

调试阶段可以使用 JSON 字符串消息，但正式实验前应定义稳定的自定义消息，至少带：

- `header.stamp`；
- `run_id`、`episode_id`、`step_id`、`config_seq`；
- 原始/投影/安全/最终参数；
- 安全模式和触发原因；
- request、ack、activation 时间戳。

### 4.3 TEB 参数写入

运行时禁止逐条执行 `rosparam set`。必须：

1. 在启动时探测 `/move_base/TebLocalPlannerROS` 的 dynamic reconfigure 参数描述；
2. 一次请求原子写入一组参数；
3. 等待服务返回并读回核对；
4. 为每次请求分配单调递增的 `config_seq`；
5. 记录 `t_request`、`t_ack` 和读回值；
6. 读回不一致、服务超时或参数不存在时进入 `interface_fault`，停止学习写入并回退。

若当前 TEB fork 某些参数不支持在线更新，应将它们移出动作空间，改为 episode 启动前静态配置。

---

## 5. 状态、动作与参数空间

### 5.1 状态 `s_t`

建议每个基础观测帧包含：

1. 36 或 72 个激光扇区的稳健最小值/低分位数；
2. 最近 footprint 净空 `d_obs`；
3. 局部障碍物密度 `rho`；
4. 最小近似 TTC；
5. 目标距离、目标方位角的 `sin/cos`；
6. 横向路径偏差、路径航向误差；
7. 实际线速度、角速度、线加速度；
8. 当前归一化 TEB 参数 `z_theta`；
9. 上一个语义状态 `eta` 和上一个动作；
10. TEB 是否产生有效局部轨迹、近期规划失败次数；
11. 传感器、TF、定位、dynamic reconfigure 有效标志；
12. safety mode one-hot。

历史窗口建议初始取 `K=4`，窗口间隔等于 RL 决策周期或其子采样。`K` 是超参数，最终必须冻结并记录。历史窗口的目的，是缓解“参数写入—TEB 重规划—底盘响应—性能反馈”延迟造成的非马尔可夫性，不是为了无限堆叠历史数据。

所有连续特征必须使用训练集统计量或物理上界归一化。验证集和测试集不得参与归一化统计量拟合。

### 5.2 TEB 参数向量 `θ`

第一阶段建议候选在线参数为：

```text
max_vel_x
max_vel_theta
acc_lim_x
acc_lim_theta
min_obstacle_dist
inflation_dist
weight_obstacle
weight_viapoint
weight_optimaltime
```

只有在 Ubuntu 实际 TEB fork 中确认存在、支持在线写入且不会造成优化器结构突变后，才能正式纳入。以下参数必须固定：

- `cmd_angle_instead_rotvel=false`；
- `min_turning_radius`（由 M2 标定）；
- footprint/车体几何；
- odom topic、坐标系；
- homotopy 开关和优化器结构参数；
- costmap 尺寸与传感器源；
- 底盘驱动语义。

### 5.3 归一化原始参数

为消除量纲差异，所有动作映射在归一化参数空间完成：

```text
z_i = 2 * (theta_i - theta_i_min) / (theta_i_max - theta_i_min) - 1
```

策略或适配器输出 `Δz`，再反归一化为物理参数。Direct-Theta 组与 Semantic-Eta 组必须共享相同的物理上下界、变化率和安全过滤器。

### 5.4 语义动作 `η`

```text
eta_v : 速度/效率偏好
eta_o : 避障保守性
eta_d : 安全距离偏好
eta_p : 全局路径/途经点跟踪偏好
eta_s : 平滑性偏好
```

策略动作：

```text
a_t = Δη_t ∈ [-1, 1]^5
```

固定映射：

```text
Δz_theta = A_TEB * Δη
```

`A_TEB` 必须稀疏、固定、带版本号，并在正式训练前冻结。首阶段不得与 SAC 联合学习。

### 5.5 `A_TEB` 初始符号模板

以下只是需要通过敏感性试验验证的符号先验，不是最终数值：

| 语义维度 | 预期主要影响 | 符号意图 |
| --- | --- | --- |
| `eta_v` | `max_vel_x`、`acc_lim_x`、`weight_optimaltime` | 提高效率偏好时倾向增大 |
| `eta_o` | `weight_obstacle`，必要时联动速度上限 | 更保守时增大障碍权重、限制速度 |
| `eta_d` | `min_obstacle_dist`、`inflation_dist` | 安全距离偏好增大时同向增大 |
| `eta_p` | `weight_viapoint` | 路径跟踪偏好增大时同向增大 |
| `eta_s` | `acc_lim_x`、`acc_lim_theta`、必要时 `max_vel_theta` | 更平滑时限制突变与高角速度 |

不要让同一语义维度同时产生相互矛盾的强耦合。若敏感性结果显示某个参数方向不稳定，应从映射中删除，而不是强行解释。

---

## 6. RL step、参数生效与奖励归因

### 6.1 三层时间尺度

初始建议：

- 底盘/控制与 TEB 规划：约 10 Hz，以现有 `controller_frequency` 和实际 topic 频率为准；
- RL 调参：初始 1 Hz；
- 每个 RL step 覆盖 `N_p = floor(T_RL / T_planner)` 个完整规划周期。

频率必须从 ROS 时间戳测量，不能仅依赖 `sleep()` 计数。

### 6.2 参数生效定义

一次动作的时间线：

```text
t_decision -> t_request -> t_ack -> t_active -> [reward window] -> t_window_end
```

- `t_decision`：策略完成推理；
- `t_request`：发送 dynamic reconfigure 请求；
- `t_ack`：服务返回且读回参数一致；
- `t_active`：`t_ack` 后首个完整 TEB 局部轨迹/规划周期完成；
- reward window：只统计 `t_active` 之后的有效反馈；
- 写入过渡区间不得归因给当前动作。

若 `t_active` 在超时内没有出现，记录 `parameter_activation_timeout`，该 step 不作为普通转移写入经验池，系统进入规划/接口故障处理。

### 6.3 窗口奖励

奖励必须以物理时间积分或窗口净变化构造，避免受采样频率影响：

```text
R_t = w_progress * (goal_dist_start - goal_dist_end)
      - w_time * T_valid
      - ∫ w_near * near_risk(d_obs) dt
      - ∫ w_path * path_error^2 dt
      - ∫ w_smooth * (a^2 + lambda_w * angular_acc^2) dt
      - w_plan_fail * planner_fail_count
      - w_adjust * ||Δz_theta||_1
```

终止事件另外加入明确的 goal/collision 奖惩。所有分量必须单独记录，禁止只保存总奖励。

### 6.4 CMDP 成本

至少记录以下独立成本：

```text
c_collision
c_near_collision
c_parameter_violation
c_planner_failure
c_emergency_or_fallback
```

第一版可以使用标准 SAC 学习奖励、运行时安全过滤保证执行边界，同时记录 CMDP 成本用于训练和评估。只有在基础 SAC 管线稳定后，才考虑实现 Lagrangian SAC/成本 critic。CPO 只作为理论参照，不要求作为本论文实验算法。

---

## 7. 参数投影、安全过滤与回退

### 7.1 Parameter projection

投影层必须完成：

1. 物理上下界裁剪；
2. 单步变化率限制；
3. 不可在线更新参数拒绝；
4. NaN/Inf/维度错误拒绝；
5. Ackermann/TEB 已知耦合约束检查。

分别保存 `theta_candidate` 和 `theta_projected`。只要两者不同，就增加 `projection_intervention_count` 并记录原因位掩码。

### 7.2 安全距离模型

实车安全裕度至少包含：

```text
d_safe(v) = v^2 / (2 * a_brake_lower)
            + v * tau_total_upper
            + d_margin
```

其中：

- `a_brake_lower`：实车制动标定得到的保守减速度下界；
- `tau_total_upper`：感知、状态构造、推理、参数写入、TEB 重规划和底盘响应总时延的高分位上界；
- `d_margin`：定位、点云、footprint 和场地误差余量。

安全裕度：

```text
h = d_obs - d_safe(v)
```

该 `h` 用于工程风险筛选，不得在论文或代码注释中称为已经证明的严格 CBF。

### 7.3 Safety mode

建议四态状态机：

```text
NORMAL -> WARNING -> EMERGENCY -> FAULT
```

- `NORMAL`：正常投影与写入；
- `WARNING`：禁止增加速度，提升保守性或缩小动作；
- `EMERGENCY`：停止学习参数写入，切换保守参数，必要时发布独立减速/停车请求；
- `FAULT`：传感器、TF、定位、参数接口或规划持续无效，保持安全停止并要求人工复位。

恢复必须有滞回和连续健康时间，不能在阈值附近每个周期来回切换。

### 7.4 完整安全过滤器不是 if-else 集合

完整实现必须同时包括：

- 连续安全裕度和风险评分；
- 参数盒约束和变化率投影；
- 速度、安全距离、障碍物权重之间的联合修正；
- 输入有效性和规划状态检查；
- 离散模式状态机；
- 滞回恢复；
- 回退参数原子写入；
- 可审计的触发原因日志。

### 7.5 回退参数

至少准备三组冻结配置：

1. `teb_default.yaml`：软件/模板默认；
2. `teb_tuned.yaml`：独立标定集人工调参并冻结；
3. `teb_conservative.yaml`：低速、较大安全裕度、实车验证通过。

回退优先使用 `teb_conservative.yaml`；接口故障时使用最后一次已确认安全参数；严重故障进入停车并要求人工复位。每次回退和恢复都必须记录开始时间、原因、持续时间和恢复结果。

---

## 8. M2 与 Gazebo 仿真系统

### 8.1 仿真必须保持的接口一致性

Gazebo 模型必须尽量与实车保持：

- 相同 `base_link`、激光坐标和 footprint；
- Ackermann 转向或至少相同曲率/最小转弯半径约束；
- `/cmd_vel` 输入语义与 `m2_driver` 一致；
- 相同 `/scan`、odom、TF、move_base 与 TEB 接口；
- 可配置传感器频率、噪声、丢帧和执行延迟；
- 可确定性 reset，并能根据 seed 重建同一场景。

禁止使用差速底盘模型训练后直接称为 M2 sim-to-real。若暂时只能用差速模型做软件管线验证，必须标记为 `pipeline_only`，不得进入正式论文结果。

### 8.2 建模顺序

1. 盘点 Arena 子模块中已有 Gazebo/机器人模型；
2. 优先复用现有插件与 scenario manager；
3. 若缺少 M2，创建最小 M2 xacro、惯量、碰撞体、激光和 Ackermann 驱动；
4. 单独验证直线、定半径圆、停止、倒车和 `/cmd_vel.angular.z` 语义；
5. 再接 move_base + TEB；
6. 最后接 RL 调参。

### 8.3 Domain randomization

随机化范围必须来自实车标定或合理工程范围：

- 激光距离噪声、随机丢点、扫描延迟；
- odom/GNSS 位置与航向噪声、偶发短时冻结；
- 参数写入和规划激活延迟；
- 底盘一阶响应、制动能力、转向响应；
- 地面摩擦和轻微侧滑；
- 障碍物位置、尺寸和动态速度；
- 全局路径轻微偏差。

训练随机化范围、ID 测试范围、OOD 范围和 disturbance 范围必须分开保存。测试扰动不得在训练中泄漏。

---

## 9. 场景库与数据划分

### 9.1 场景类型

| 类型 | 主要验证内容 |
| --- | --- |
| Open | 效率、参数稳定、无意义调参次数 |
| Sparse | 普通静态绕行 |
| Narrow | 窄通道可行性、最小净空、局部失败 |
| Dense | 密集障碍、振荡、局部最优、回退 |
| Semi-dynamic | 动态干扰与低频调参响应 |
| M2-like | 按实车场地、车宽、转弯半径、速度和传感器构造 |

场景尺度不要仅写绝对数字，应同时记录相对车体尺度。例如窄通道记录：

```text
clearance_ratio = (corridor_width - vehicle_width) / 2
```

正式范围由 M2 footprint 和标定安全距离确定。

### 9.2 划分

- `train`：策略更新；
- `validation`：模型选择、早停和奖励调试；
- `test_id`：同分布新 seed；
- `test_ood`：未见通道宽度、密度、组合或动态模式；
- `test_disturbance`：固定几何，改变噪声、时延和响应；
- `real`：rosbag、shadow、实车闭环。

每个 scene manifest 必须包含：`scene_id`、split、world 文件、起终点、障碍物参数、seed、timeout、碰撞判据、随机化配置 hash。

---

## 10. 正式实验矩阵

| 实验 ID | 组别 | 目的 | 运行范围 |
| --- | --- | --- | --- |
| E0 | 标定与接口测试 | 建立 M2/TEB/时延/安全范围 | Gazebo + M2 低风险标定 |
| E1 | Fixed-DWA | 传统局部规划基线 | Gazebo；实车可选 |
| E2 | TEB-Default / TEB-Tuned | 默认与人工调参基线 | Gazebo + M2 |
| E3 | Rule-TEB | 排除简单规则即可解释收益 | Gazebo + M2 |
| E4 | RL-TEB-Direct-Theta | 原始参数动作对照 | Gazebo only |
| E5 | RL-TEB-Semantic-Eta | 论文提出方法 | Gazebo + M2 |
| E6a | Eta-NoSafety | 安全层消融 | Gazebo only |
| E6b | Eta-ProjectionOnly | 参数投影贡献 | Gazebo only |
| E6c | Eta-FullSafety | 完整安全过滤 | Gazebo + M2 |
| E7a | Eta-NoFallback | 回退消融 | Gazebo only |
| E7b | Eta-WithFallback | 回退效果 | Gazebo + M2 |
| E8 | rosbag offline / shadow | sim-to-real 前置检查 | M2 不写参数 |

### 10.1 公平性要求

- 所有 TEB 组共享相同全局规划器、costmap、M2 模型、状态、奖励、终止规则、场景和评估 seed。
- Direct-Theta 与 Semantic-Eta 共享相同物理参数边界、安全过滤和训练预算。
- 两组报告网络参数量、推理时延；必要时调整隐藏层做容量匹配检查。
- `TEB-Tuned` 只允许使用独立 calibration 场景，不能查看正式测试集。
- `Rule-TEB` 的阈值、映射和安全模块必须在测试前冻结。
- 模型选择只能根据 validation，不得按 test 结果选择 checkpoint。
- NoSafety、ProjectionOnly、NoFallback 禁止 M2 闭环。
- DWA 不进入语义空间或 RL 动作空间比较。

---

## 11. 分阶段实验方法和验收门槛

### Phase A：软件与依赖盘点

任务：

- 初始化子模块；
- 记录 ROS/Gazebo/Python/TEB/SB3 版本；
- 查清 TEB fork、dynamic reconfigure 参数、路径话题和 Gazebo 资产；
- 保存 `git status`、主仓和子模块 commit。

历史验收：完成环境盘点且不修改现有运行参数。旧 T00 inventory 已在 T02
完成后按用户要求删除，当前有效事实见 `CURRENT_TEB_RL_HANDOFF.md`。

### Phase B：M2 基础标定

必须完成：

1. footprint、轴距、传感器外参；
2. 最小转弯半径：多个低速定圆，记录转角/角速度/轨迹半径；
3. `/cmd_vel` 到实际速度和角速度响应；
4. 多档低速制动距离和保守减速度下界；
5. scan/odom/TF/topic 频率与端到端时延；
6. GPS 静态位置、航向质量和短时冻结；
7. dynamic reconfigure request/ack/readback/activation 时延；
8. 当前固定 TEB 的基本成功率和失败类型。

验收：原始 CSV + rosbag + 标定报告齐全。没有标定数据，不允许填写实车参数边界和 `d_safe`。

### Phase C：Gazebo M2 模型与回归

回归用例：

- 直线 5/10 m；
- 正反向低速；
- 固定 `/cmd_vel` 定半径圆；
- 停车与响应时延；
- `/scan` 障碍物距离；
- move_base + 固定 TEB 到点；
- reset 后相同 seed 结果可重复。

验收：接口一致、Ackermann/曲率行为合理、无 RL 时固定 TEB 可稳定完成简单场景。

### Phase D：参数接口、日志和伪策略

先实现：

- fixed policy；
- random-small-eta（仅管线压力测试）；
- atomic TEB writer；
- projection/filter/fallback；
- step/episode logger。

验收：每个 step 的候选、投影、安全和应用参数可以完整重建；写入失败会回退；日志通过 schema 校验。

### Phase E：单参数敏感性与 `A_TEB`

在 calibration 场景中：

1. 固定其他参数；
2. 对每个归一化参数做小幅正/负扰动；
3. 记录效率、安全、路径偏差、平滑和规划失败变化；
4. 计算局部敏感性和符号一致率；
5. 建立稀疏 `A_TEB`；
6. 在多个场景复核单调方向；
7. 冻结 `A_TEB_v1.yaml` 和 hash。

如果某参数在不同场景符号高度不一致，删除对应映射或降低幅值，不能通过选择性场景解释。

### Phase F：固定与规则基线

顺序：

1. TEB-Default；
2. TEB-Tuned；
3. Fixed-DWA；
4. Rule-TEB。

验收：相同 episode manager、CSV 和统计脚本可运行所有基线；Rule-TEB 不依赖未来信息。

### Phase G：SAC 训练

建议先使用标准 SAC，连续动作 `[-1,1]`。训练过程：

1. 单一简单场景管线验证；
2. 多场景 curriculum；
3. 加入 domain randomization；
4. validation 早停和 checkpoint；
5. Direct-Theta 与 Semantic-Eta 使用相同环境步数预算；
6. 至少多个独立训练 seed。

建议初始使用 5 个独立训练 seed；正式数量根据预实验方差和资源决定。训练日志必须保留每个 seed，不得只保留最佳模型。

### Phase H：正式 Gazebo 评估

先进行每类场景不少于 10 个配对 seed 的 pilot，再根据方差/置信区间确定正式重复数；资源允许时建议每类每算法 30 个配对 episode。使用 common random numbers：同一 scene/seed 在所有算法组完全一致。

验收：所有测试只使用冻结配置；原始数据不可手改；失败 episode 不删除。

### Phase I：rosbag 离线回放

策略读取真实 `/scan`、odom、路径和 TEB 状态，执行完整状态构造、推理、适配、投影和安全过滤，但不调用参数写入。检查：

- 特征范围是否超出训练分布；
- 参数建议是否频繁触界；
- safety mode 是否误触发；
- 推理和模拟写入时延；
- semantic direction consistency；
- 真实传感器无效和 TF 故障处理。

### Phase J：M2 影子模式

M2 使用 TEB-Tuned 固定参数真实导航；RL 实时运行但只记录 `theta_candidate/projected/safe`。至少覆盖开放、稀疏障碍和安全可控的窄通道。

进入闭环前要求：

- 无未解释 NaN/Inf/大幅越界；
- request-to-simulated-activation 时延小于 RL 周期；
- safety/fallback 原因均可解释；
- 电子围栏、急停、保守参数恢复和 rosbag 已演练；
- 双天线方向、TF、scan、odom 和 `/cmd_vel` 链路正常。

### Phase K：M2 低速闭环

实车测试必须由人类现场批准，Codex CLI 不得自动启动运动。建议门槛式限速：

1. 首轮不超过 `0.3 m/s`；
2. 通过后不超过 `min(0.5 m/s, 标定安全速度)`；
3. 更高速度必须基于制动距离、场地和安全负责人重新批准；当前候选 `1.5 m/s` 不得直接用于第一轮 RL 闭环。

实车主要比较 TEB-Tuned、Rule-TEB、RL-TEB-Semantic-Eta。Direct-Theta 和危险安全消融不需要实车运行。

实验顺序应随机化或使用 Latin-square 平衡顺序，减少电量、温度、GNSS 时段和操作者学习效应。每条路线先做 pilot，正式重复数由方差和可用时间决定；建议以每方法每路线 10 次配对运行为起点。

---

## 12. Episode 终止、截断和人工处置

### 12.1 Termination

- `goal`：在容差内到达；
- `collision`：仿真 contact 或实车确认接触；
- `planner_failure`：TEB 连续无可行轨迹超过冻结阈值；
- `sensor_fault`、`tf_fault`、`interface_fault`：持续超过冻结阈值；
- `emergency_stop`：安全状态机或硬件急停终止。

### 12.2 Truncation

- `timeout`；
- Gazebo/ROS 基础设施故障；
- `operator_stop`；
- 非算法原因的场地中断。

每个 episode 必须同时记录 `terminated`、`truncated` 和唯一 `termination_reason`。不能把 timeout 当作自然失败后又从统计中删除。

实车发生以下任一情况立即人工停止：

- 航向与真实车头明显不一致；
- `/scan`、TF、odom 持续无效；
- 车辆越出电子围栏；
- 参数写入值与读回值不一致；
- 车辆无目标加速、持续倒车振荡或不可预测转向；
- 最近障碍物距离低于 emergency 阈值；
- 急停、CAN 或底盘链路异常。

---

## 13. 指标定义

### 13.1 任务与安全

- `success`：时限内到达且未发生碰撞/紧急故障；
- `collision`：仿真 contact 或实车物理接触；
- `min_obstacle_distance`：障碍点到 footprint 多边形的最小净空，不直接使用激光到传感器原点距离；
- near-collision rate：`d_obs < d_warn` 的 episode/时间占比；
- emergency/fallback 次数和持续时间；
- operator intervention 次数。

### 13.2 效率

- `navigation_time = t_end - t_start`；
- `path_length = Σ ||p_i-p_{i-1}||`，对异常定位跳点先按冻结规则标记，不可静默删除；
- path efficiency：最短可行参考长度/实际长度；
- goal progress；
- TEB 求解/规划失败率。

### 13.3 平滑与稳定

- 线加速度、角加速度 RMS；
- jerk 或命令差分积分；
- 前后切换次数；
- 参数总变差 `Σ||theta_t-theta_{t-1}||_1`；
- 无意义调参率：Open 场景中小收益但频繁变化的窗口比例。

### 13.4 学习与解释

- 学习曲线和 AUC；
- 达到冻结 validation 阈值所需环境步数；未达到记为删失；
- ID/OOD/disturbance 性能和相对下降；
- candidate violation、projection/filter intervention；
- semantic direction consistency；
- 网络参数量、推理时延和内存占用。

语义方向一致性规则必须预注册。例如障碍物持续接近时，`eta_v` 不应持续显著增大，`eta_o/eta_d` 的净响应应符合冻结的方向定义。该指标评价审计一致性，不等同于最优性。

---

## 14. 数据记录与可复现合同

### 14.1 目录

```text
config/thesis_experiments/          # 可提交的冻结配置
experiments/manifests/              # 可提交的 scene/run manifest
artifacts/raw/<run_id>/             # 不提交；bag、step 原始数据
artifacts/processed/<run_id>/       # 不提交或按项目规则处理
exports/thesis_<date>/              # 论文交付包
  csv/episodes.csv
  csv/steps.csv.gz 或 steps.parquet
  configs/
  figures/
  tables/
  failures/
  checksums.sha256
```

`.bag`、build/devel/install/log、训练 checkpoint 和大体积 raw artifacts 不得提交 Git。配置、schema、统计脚本、manifest 和小型汇总 CSV 可以提交。

### 14.2 每次运行必须保存

- run/scene/config manifest；
- 主仓与子模块 commit、dirty 状态；
- ROS/Gazebo/Python/依赖版本；
- policy checkpoint 路径与 SHA256；
- episode CSV；
- step 级参数/奖励/安全日志；
- rosbag 或 Gazebo 原始日志；
- stdout/stderr；
- 失败事件索引；
- 最终配置快照和 checksum。

### 14.3 rosbag 最小话题

```text
/scan
/gps/odom 或 /Odometry
/gps/heading（实车）
/tf
/tf_static
/cmd_vel
/move_base/status
/move_base_simple/goal
/move_base/TebLocalPlannerROS/local_plan
/move_base/*/global_plan
/thesis_teb_rl/*
```

根据实际 topic 名修正通配项。录包前用 `rostopic hz` 和磁盘剩余空间检查。

### 14.4 结果导回论文

Ubuntu 端只生成 export bundle，不直接人工修改论文表格。将：

```text
exports/thesis_<date>/csv/*.csv
```

复制到论文工作区的：

```text
thesis_word_workspace/06_results/csv/
```

再由冻结统计脚本生成 LaTeX 表格与图片。每个论文表格必须能追溯到 export bundle、脚本版本和配置 hash。

---

## 15. 统计分析

- 统计独立单位是 episode，不是 episode 内的 10 Hz 样本。
- 同一 scene/seed 的算法结果做配对比较。
- 学习方法使用多个独立 training seed；评估同时区分 training seed 和 evaluation seed。
- 连续指标报告均值、标准差、中位数、四分位数、95% 置信区间和效应量。
- 二元指标报告比例置信区间；配对二元结果优先使用 McNemar 或配对 bootstrap。
- 连续配对差近似正态时使用配对 t 检验，否则使用 Wilcoxon signed-rank。
- 多组/多指标比较使用 Holm 校正。
- 同时报告场景分组结果和宏平均，不能用宏平均掩盖 Narrow/Dense 的退化。
- 样本效率报告 learning curve、AUC 和 time-to-threshold，不只报告最终最好回报。
- OOD 报告绝对性能和相对 ID 降幅。
- 实车报告每次运行，不只展示一条好看的轨迹。

正式统计前生成 preregistration YAML：主要指标、主要比较、排除规则、测试、显著性水平和效应量定义全部冻结。

---

## 16. 论文主张与实验的对应关系

| 论文内容 | 必需证据 | 对应实验/文件 |
| --- | --- | --- |
| 低维 `Δη` 是否优于直接 `Δθ` | 同预算学习曲线、阈值步数、OOD、越界、网络容量 | E4 vs E5；training/episode/step logs |
| `A_TEB` 如何得到 | 单参数正负扰动、敏感性、符号一致率、冻结矩阵 | Phase E；`A_TEB_v1.yaml`、calibration CSV |
| 语义空间是否更可解释 | 语义响应曲线、方向一致率、参数响应图 | E5 step logs |
| safety layer 是否只是规则 | 连续裕度、投影、联合修正、状态机和原因日志 | 模块测试 + E6a/b/c |
| 各模块贡献多少 | projection/filter/fallback 消融及效率代价 | E6、E7 |
| RL 是否优于固定/规则 | 分场景配对统计与失败案例 | E2/E3/E5 |
| 一次 RL step 覆盖多少周期 | 真实时间戳、规划周期计数 | step log 的 planner_cycle_count |
| 参数何时生效、反馈是否延迟 | request/ack/activation 时间戳 | latency fields |
| 历史窗口作用 | K 配置、必要时 K=1 对照或定性分析 | state config / optional ablation |
| sim-to-real 是否成立 | 仿真扰动、rosbag、shadow、M2 配对结果 | Phase H--K |
| M2 实车基础 | 标定原始数据、链路检查、配置快照 | Phase B、real manifests |

没有对应证据的主张必须降级为设计动机或未来工作。

---

## 17. Codex CLI 分阶段任务清单

每次只完成一个阶段，先检查再修改，阶段结束必须运行测试并更新 `docs/thesis_experiment/DEVELOPMENT_STATUS.md`。

### T00：环境盘点

- 阅读本交接书与 GPS handoff；
- 初始化子模块；
- 记录 git/ROS/Gazebo/TEB/SB3；
- 查实际 dynamic reconfigure 和 topic。

完成标准：inventory 文件和风险清单。

### T01：实验合同与 catkin 包骨架

- 创建 `teb_rl_tuner`、`thesis_experiment`；
- 安装 Python 依赖到隔离环境或明确 requirements；
- 建立配置加载、schema 校验和单元测试。

完成标准：`catkin_make`、Python import、配置测试通过。

### T02：Gazebo M2 接口

- 复用或创建 M2 模型；
- 验证 Ackermann、TF、scan、odom、cmd_vel；
- 写简单回归测试。

完成标准：固定 TEB 完成简单导航，重复 reset 可重现。

### T03：TEB 参数客户端

- 参数发现；
- 原子写入、readback、seq、延迟；
- 失败回退。

完成标准：对测试参数做多次写入，日志可验证 ack/active。

### T04：状态、时间窗、奖励和 episode manager

- 时间同步；
- K 历史；
- termination/truncation；
- reward/cost 分量。

完成标准：固定策略产生完整合法 transition。

### T05：投影、安全过滤和回退

- 纯函数/类实现；
- 单元测试边界、速率、NaN、风险、状态机、滞回；
- Gazebo 故障注入。

完成标准：危险动作不被应用；原因可追溯。

### T06：日志与导出

- episode/step schema；
- rosbag manifest；
- checksum；
- run validator。

完成标准：一个 run 可从原始数据重建全部指标。

### T07：标定和 `A_TEB`

- 自动参数扫描；
- 敏感性报告；
- 冻结矩阵。

完成标准：矩阵来源不是人工拍脑袋，所有非零项有证据。

### T08：基线

- TEB-Default、TEB-Tuned、Fixed-DWA、Rule-TEB；
- 共用 evaluator。

完成标准：同一 scene/seed 可批量运行并比较。

### T09：SAC Semantic-Eta

- Gym-like ROS/Gazebo 环境；
- 多 seed 训练与 validation；
- checkpoint/hash。

完成标准：训练可恢复、评估不更新模型、日志完整。

### T10：SAC Direct-Theta

- 同训练预算和安全边界；
- 报告网络容量与时延。

完成标准：动作空间以外条件一致。

### T11：正式仿真与消融

- 冻结 manifests；
- 配对 seed；
- E1--E7 批量运行；
- 失败索引。

完成标准：无 test leakage、无手工删除失败数据。

### T12：rosbag 和 shadow

- offline replay；
- real-time shadow；
- 分布漂移与时延报告。

完成标准：通过实车闭环门槛。

### T13：M2 闭环

- 现场人工批准；
- 低速、围栏、急停、随机化实验顺序；
- 完整 bag/CSV/视频索引。

完成标准：每次运行可复核，所有人工干预有记录。

### T14：统计和论文导出

- preregistration；
- 冻结统计脚本；
- 图、表、置信区间、效应量；
- export bundle 和 checksum。

完成标准：论文数值可由一条命令从 CSV 重新生成。

---

## 18. Ubuntu 端建议启动命令

基础环境：

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

构建与测试：

```bash
catkin_make
python3 -m pytest src/application/teb_rl_tuner/tests
bash -n scripts/bringup.sh
```

GPS 实车链路只读检查：

```bash
./scripts/bringup.sh gps
rostopic hz /scan /gps/odom /cmd_vel
rosrun tf tf_echo camera_init base_link
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
```

任何真实运动命令都必须由现场用户手动执行。本交接书不授权 Codex CLI 自主启动实车运动。

---

## 19. 当前待定项，开发时必须逐项关闭

- [ ] Arena/TEB/Gazebo 子模块在 Ubuntu 的实际 commit 与可用资产；
- [ ] TEB dynamic reconfigure 支持的准确参数名、类型和范围；
- [ ] 全局/局部路径和 TEB 失败状态的准确话题；
- [ ] M2 footprint、轴距、最小转弯半径；
- [ ] `/cmd_vel` 到实际速度/转向的响应与饱和；
- [ ] 制动减速度下界和总时延上界；
- [ ] `min_obstacle_dist`、inflation 与 footprint 净空的统一定义；
- [ ] `theta_min/max`、单步变化率、实车子范围；
- [ ] `A_TEB` 数值；
- [ ] RL 周期、历史窗口 K、reward 权重和 CMDP cost budget；
- [ ] 训练/验证/ID/OOD/扰动场景 manifests；
- [ ] 实车路线、场地、限速、人员和重复次数；
- [ ] 统计 preregistration 与论文回填脚本。

这些项没有实验或工程证据时必须保留为 `null/TBD`，禁止为了跑通代码随意填入并写成最终配置。

---

## 20. 最终交付物

Ubuntu 端开发完成后，应交付：

1. 可构建的 ROS/catkin 源码；
2. M2 Gazebo 模型、world、scenario manifests；
3. fixed/rule/direct-theta/semantic-eta 策略实现；
4. `A_TEB` 标定程序和冻结矩阵；
5. 参数投影、安全裕度过滤、回退和单元测试；
6. SAC 训练、评估、checkpoint 管理和多 seed 脚本；
7. Gazebo 正式实验和全部消融；
8. rosbag replay、shadow 和 M2 低速闭环工具；
9. episode/step CSV、bag manifests、失败索引、配置快照；
10. 冻结统计脚本、LaTeX 表格、图片与 export bundle；
11. `DEVELOPMENT_STATUS.md`，说明已完成、未完成、已知问题和复现命令；
12. 论文第 3--6 章的证据对应表。

交付完成的判断标准不是“程序能启动”，而是：同一冻结配置能够重复运行，失败不会被隐藏，每个论文结论都能追溯到原始数据、配置 hash 和统计脚本。
