# FAM-TEB V2 场景感知混合规划系统指南

版本：`v2.0-design-1`  
日期：2026-07-13  
架构代号：`FAM-TEB V2`（Factorized Adaptive Multi-mode TEB）  
适用工作空间：`/home/robot/robot_ws_base_rl`  
当前状态：V2-00--V2-03 已通过组件验收；所有阈值保持 `runtime_ready=false`，规划后端和学习策略尚未实现，尚未授权新训练或实车闭环

## 0. 文档定位

本文是 FAM-TEB V2 的系统级开发、接口、实验与验收指南，用于把五类异构导航场景和
混合过渡场景落实为可实现、可测试、可审计的软件架构。

本文同时承担四种职责：

1. 规定 V2 的模块边界和数据流；
2. 规定模式、状态、动作、参数和安全语义；
3. 给出软件包骨架、实施顺序和阶段退出条件；
4. 规定证明系统有效所需的基线、消融、指标和效果门槛。

本文不修改或覆盖 T00--T12 的历史合同和冻结证据。现有
`docs/thesis_experiment/experiment_contract.yaml` 仍是 V1/T00--T12 的机器可读合同。
V2 正式实验必须另建带版本号的机器合同和 preregistration，不得把 V2 运行伪装成 T12
单因素续跑。

## 1. 研究目标与范围

### 1.1 V2 主命题

> 针对单一固定参数或单一连续调参策略难以同时适应高速巡航、动态障碍交互、静态密集
> 障碍绕行、狭窄通道通过和死角双向机动的问题，构建一种因子化场景感知、混合规划器
> 协同、执行感知且风险受限的多模式局部导航框架。在成功率和碰撞风险不退化的前提下，
> 提高导航效率、路径稳定性和异构场景适应能力。

### 1.2 V2 的激进修改范围

V2 允许对以下层次做结构性修改：

- 新增动态目标跟踪、短时轨迹预测和局部世界模型；
- 将单一场景标签改为几何模式与动态风险覆盖层的因子化表示；
- 将单 Anchor 改为多 Anchor 库和执行态连续过渡；
- 将单策略改为共享编码器加模式专家头的残差策略；
- 将单一 TEB 后端改为预实例化的混合规划器后端库；
- 为静态障碍增加显式拓扑锁，为死角增加 Ackermann Hybrid A*/状态格机动器；
- 将动作裁剪改为约束内生的可行动作解码；
- 在参数安全层之外增加预测轨迹级最小干预安全盾；
- 建立独立的 V2 场景、日志、评估和 preregistration 合同。

### 1.3 仍然不可突破的边界

- 学习策略不得直接发布底盘 `/cmd_vel`；
- RL 不得关闭接口故障停车、碰撞净空、停车距离、运动学约束或人工急停；
- footprint、轴距、最小转弯半径、坐标系和驱动语义不得作为在线学习动作；
- NoSafety、ProjectionOnly、NoFallback 等危险消融不得用于实车闭环；
- Pedsim 或 Gazebo 真值不得作为最终部署策略的感知输入；
- 未完成现场标定和逐次人工批准，不得启动实车运动或在线写入实车 TEB 参数；
- 不得将当前运动学仿真的零制动距离解释为真实高速安全能力；
- 不得删除失败 episode、失败日志或不利 seed；
- 不得重启或扩大历史 T12 pilot 预算。

## 2. V2 的事实依据

V2 不是对论文设想的任意扩张，而是由当前系统证据驱动：

1. 当前实际动作链是 5 维 Semantic-Eta SAC，经固定矩阵映射到 9 维 TEB 参数，并围绕
   单一 TEB-Tuned Anchor 输出非累积残差。
2. 最新两 seed 2000-step 运行已解决可复现 SIGSEGV 链，但 validation change 为
   `+0.4527/-0.0784`，学习门失败。
3. 两 seed 训练 projection rate 为 `65.1%/69.5%`，均值 `67.3%`。无前一安全干预时仍为
   `44.82%/48.81%`，主要来自 Ackermann 耦合；安全干预后下一步为
   `92.48%/98.91%`，主要来自回指固定 Anchor 的变化率限制。
4. 当前动作矩阵无法直接控制 `max_vel_theta` 和 `weight_viapoint`，不适合承担巡航航向
   稳定和静态拓扑保持。
5. 当前近似 TTC 只使用机器人自身速度与激光净空，不是相对运动 TTC。
6. 当前正式场景只覆盖 clear、single obstacle 和 straight corridor，没有动态交互、密集
   静态障碍、门框、U 形死角和混合过渡。
7. TEB 的单轨迹与 HCP 类型在初始化时确定，现有九参数事务不能安全切换规划器结构。
8. V1 默认仿真底盘是即时响应的确定性运动学模型；V2-02 已新增隔离的执行器、制动、
   转向和指令/传感器时延候选模型，但这些值尚未经过实车标定。

因此，V2 的首要目标不是增加训练步数，而是修复“场景表达—规划机制—动作可行性—
实际执行—经验回放”之间的结构错配。

## 3. 统一术语

### 3.1 几何模式 `GeometryMode`

固定枚举：

- `CRUISE`：低障碍密度、长前向净空、路径曲率低；
- `STATIC_DENSE`：静态障碍密集，需要提前选择并保持绕行拓扑；
- `CORRIDOR`：侧向净空受限，需要中心线精确跟踪；
- `MANEUVER`：常规前向局部规划不可行，需要带挡位的前后机动；
- `BALANCED`：模式置信度不足或条件混合时的保守默认模式。

### 3.2 动态风险覆盖层 `DynamicOverlay`

固定枚举：

- `NONE`；
- `CROSSING`：预测轨迹横穿机器人参考路径；
- `HEAD_ON`：相对闭合速度高且方向相反；
- `FOLLOW`：同向较慢目标位于路径前方；
- `OVERTAKE_OR_YIELD`：需要在绕行、跟随和让行之间选择。

动态风险覆盖层不是第六个互斥几何模式。例如“走廊中的横穿行人”应表示为：

    GeometryMode = CORRIDOR
    DynamicOverlay = CROSSING

### 3.3 过渡状态 `TransitionState`

至少包括：

- `STABLE`：模式稳定；
- `ENTERING`：正在进入新模式；
- `EXITING`：正在释放旧模式约束；
- `HOLDING`：受最短驻留时间约束；
- `SAFE_OVERRIDE`：安全层临时接管；
- `FAULTED`：接口、感知或规划健康失败。

### 3.4 动作与参数术语

- `anchor`：某一模式下独立标定并冻结的参数中心；
- `profile`：包含连续、整数和布尔模式参数的类型化配置；
- `residual`：策略在 Anchor 附近输出的连续性能调节量；
- `action_commanded`：策略原始输出；
- `action_feasible`：约束内生解码后的动作；
- `action_safe`：参数安全层修正后的动作；
- `action_executed`：已 ack、readback 且激活的真实执行动作；
- `projection`：最后审计层对不可行动作的修正，正常情况下应为低频事件；
- `backend`：产生局部轨迹的规划器实现；
- `topology_id`：在稳定参考系中标识绕行同伦类别的 ID；
- `gear`：`FORWARD`、`REVERSE` 或 `STOP`。

## 4. 总体架构

    LaserScan / PointCloud2 / Odom / TF / Costmap / Global Path
                                |
                                v
                        nav_world_model
          static geometry + tracks + predicted occupancy + health
                                |
                                v
                     factorized_context_supervisor
          geometry mode + dynamic overlay + confidence + dwell
                                |
                                v
                   anchor_bank + transition_manager
                                |
                                v
           mode-conditioned residual policy / rule policy
                                |
                                v
                    feasible_action_decoder
          command -> feasible -> parameter safety -> executed
                                |
               +----------------+----------------+
               |                |                |
               v                v                v
        SingleTopologyTEB   TopologyLockedHCP  ManeuverLattice
        Cruise/Corridor       StaticDense      Forward/Reverse
               +----------------+----------------+
                                |
                                v
                    timed trajectory + raw cmd
                                |
                                v
                      predictive_motion_shield
                                |
                                v
                             /cmd_vel

### 4.1 核心职责边界

| 层 | 负责 | 不负责 |
| --- | --- | --- |
| 世界模型 | 静态几何、动态轨迹、预测占据、数据健康 | 选择规划模式、修改参数 |
| 场景监督器 | 模式、覆盖层、置信度、滞回与驻留 | 直接生成速度命令 |
| Anchor/策略 | 模式内性能取舍和连续残差 | 规避硬运动学约束 |
| 可行解码器 | 通过参数化构造满足耦合约束的候选 | 代替碰撞检查 |
| 混合后端 | 生成几何与运动学可行轨迹 | 修改安全不变量 |
| 预测安全盾 | 对最终轨迹/命令做最小干预 | 追求奖励或隐藏失败 |
| 实验系统 | reset、日志、manifest、评估和失败留存 | 向策略泄漏测试结果 |

## 5. 局部世界模型

### 5.1 输入

必须输入：

- `/scan` 或过滤后的 `PointCloud2`；
- `/odom`、`/gps/odom` 或 `/Odometry` 中合同指定的一种；
- `/tf`、`/tf_static`；
- move_base 全局路径与 TEB 局部路径；
- local/global costmap；
- 机器人当前速度、加速度和上一执行参数；
- 感知、TF、定位、规划器和参数接口健康状态。

仿真可以额外订阅 Pedsim/Gazebo 真值，但只能进入 evaluator/oracle 分支，不能进入最终
策略状态。

### 5.2 静态几何输出

至少输出：

- 前、左、右、后方向净空；
- footprint 净空；
- 局部障碍密度和静态持久性；
- 前向可行宽度；
- 走廊宽度、轴向、墙平行度和中心线偏差；
- dead-end/U-shape score；
- 全局路径曲率、带符号横向误差和带符号航向误差；
- 前视范围内的候选绕行方向和遮挡情况；
- 地图版本、观测时间戳和 stale 标志。

LaserScan 角度元数据必须进入合同，不得只按数组索引假设 360 度覆盖。

### 5.3 动态目标链

正式链路：

    点云/激光
      -> 自车与静态地图过滤
      -> 聚类/检测
      -> 门控 + Hungarian 或 JPDA 数据关联
      -> EKF/IMM 速度与状态估计
      -> 2--4 s 轨迹预测和协方差传播

每个 track 至少包含：

- 稳定 `track_id`；
- 轮廓或包围形状；
- 位置、速度及协方差；
- 分类或运动类型置信度；
- track age、miss count、last_update；
- 预测时间点、预测位姿和预测协方差；
- 坐标系和原始消息时间戳。

桥接到 TEB 的速度必须旋转到 TEB 使用的全局规划坐标系。按消息时间戳查询 TF；转换失败、
消息陈旧或时间外推过大时拒绝该 track，不得用 `ros::Time(0)` 静默替代。

动态目标若作为 custom obstacle 输入 TEB，必须从 raw static costmap 障碍中掩膜或分层，
避免同一目标同时以静态点集和动态目标重复惩罚。

### 5.4 运行频率

- 点云/激光预处理和 tracking：10--20 Hz；
- 预测占据与风险聚合：10--20 Hz；
- 几何模式监督器：2--5 Hz；
- SAC/规则参数决策：2--5 Hz；
- TEB/controller：当前目标约 10 Hz；
- 最终命令安全盾：不低于控制命令频率。

所有频率由 ROS 时间戳审计，不能只依赖 `sleep()`。

## 6. 因子化场景监督器

### 6.1 输入特征

监督器只使用运行时可观测量：

- 障碍密度、前/侧/后净空；
- 路径曲率、目标方向稳定性和近期进度；
- 走廊宽度、平行墙置信度、dead-end score；
- 局部规划可行性、振荡、recovery 和重规划事件；
- 预测 TTCA、预测最小净空、闭合速度和横穿概率；
- 上一模式、驻留时间、切换次数和当前安全状态。

严禁读取 scene manifest 的 `layout` 或测试标签作为运行时模式输入。

### 6.2 模式判定原则

优先级不是简单的固定覆盖，而是“硬可行性约束优先，性能模式次之”：

1. 任何健康故障进入 `FAULTED`，由安全层接管；
2. 常规后端连续不可行、明显死角或目标要求后向调整时，提出 `MANEUVER`；
3. 走廊宽度和墙结构置信度满足条件时，选择 `CORRIDOR`；
4. 持久静态障碍阻断前向路径时，选择 `STATIC_DENSE`；
5. 低障碍密度、长净空、低曲率和高 TTC 时，选择 `CRUISE`；
6. 条件冲突或置信度不足时使用 `BALANCED`。

`DynamicOverlay` 独立计算并叠加到任何几何模式。

### 6.3 抗抖动合同

每个模式必须配置：

- enter threshold；
- exit threshold；
- 最短进入确认时间；
- 最短驻留时间；
- 退出健康确认时间；
- 最大 Anchor 变化速率；
- 安全释放时间常数。

切换不得从新 Anchor 瞬时跳变。过渡起点必须是上一
`action_executed` 对应的安全参数，而不是固定 TEB-Tuned Anchor。

模式置信度、切换原因、进入/退出阈值、驻留剩余时间和过渡进度必须进入 step log。

## 7. Anchor Bank、参数注册表与动作解码

### 7.1 Anchor Bank

至少准备：

- `anchor_balanced`；
- `anchor_cruise`；
- `anchor_static_dense`；
- `anchor_corridor`；
- `anchor_maneuver_forward`；
- `anchor_maneuver_reverse`；
- 动态覆盖修正模板 `overlay_crossing/head_on/follow/yield`。

Anchor 必须在独立 calibration 场景中通过约束优化得到，可使用 constrained Bayesian
optimization、CMA-ES 或系统敏感性扫描。不得使用正式 test 场景挑选 Anchor。

### 7.2 参数生命周期

#### 快速连续参数

允许由模式残差策略在 2--5 Hz 调节，例如：

- `max_vel_x`、`max_vel_theta`；
- `acc_lim_x`、`acc_lim_theta`；
- `min_obstacle_dist`、`inflation_dist`；
- `weight_obstacle`、`weight_viapoint`、`weight_optimaltime`；
- 经验证后纳入的动态障碍距离和权重参数。

#### 慢速类型化模式参数

只能由模式事务管理器低频修改，例如：

- `max_vel_x_backwards`；
- `max_global_plan_lookahead_dist`；
- `global_plan_viapoint_sep`；
- HCP 候选类数、切换 blocking 和选择偏置；
- 动态障碍相关 bool/int/double profile；
- 机动段速度和换挡等待时间。

#### 启动期结构参数

启动后禁止热切换：

- 实际规划器对象类型；
- costmap-converter 插件类型；
- footprint 和机器人模型；
- 轴距、轮距、最小转弯半径；
- frame、odom 和传感器源；
- HCP/单 TEB 后端对象的构造方式。

### 7.3 约束内生解码

优先通过参数化保证约束，而不是事后裁剪：

    v_max = bounded_positive(z_v)
    rho in [0, 1]
    omega_max = rho * v_max / R_min
    min_obstacle_dist = bounded_positive(z_clearance)
    inflation_dist = min_obstacle_dist + softplus(z_gap)
    positive_weight = exp(bounded_log_weight)

高速模式还必须满足：

    v_ref <= sqrt(a_lateral_max / max(abs(curvature), epsilon))
    v_ref <= v_stopping_envelope

参数变化率相对上一 `action_executed` 计算。投影器保留为最后审计层；若正常无安全事件
pilot 中 projection 超过 10%，应视为动作设计缺陷，而不是继续增加训练预算。

## 8. 模式条件残差策略

### 8.1 策略结构

推荐结构：

    observation/history
      -> shared temporal encoder
      -> geometry-mode embedding
      -> dynamic-overlay embedding
      -> mode-specific actor head
      -> bounded latent residual
      -> feasible_action_decoder

几何模式和动态覆盖层使用因子化组合，避免为所有笛卡尔积场景建立独立网络。

### 8.2 训练顺序

1. 先冻结规则监督器和每个 Anchor；
2. 每个模式以 zero residual 初始化；
3. 分模式训练 specialist；
4. 使用混合过渡场景做联合微调；
5. 最后才考虑学习式模式门控；
6. 测试阶段冻结网络、归一化统计、监督器阈值和 Anchor。

### 8.3 执行感知 replay

每条 transition 必须记录：

- `action_commanded`；
- `action_feasible`；
- `action_safe`；
- `action_executed`；
- 四者差值；
- projection reason mask；
- safety reason mask；
- 参数事务 request/ack/readback/activation 时间戳；
- 模式、覆盖层、置信度、驻留和过渡状态。

critic 使用 `action_executed` 或等价的执行潜变量学习环境动力学。actor 增加
command-to-execution mismatch 正则，或在后续实现 constrained/Lagrangian SAC。

EMA、hold、Anchor blend 和安全释放若保留，必须成为可观测状态；禁止把影响执行结果的
内部状态隐藏在环境中形成未声明的 POMDP。

## 9. 混合规划器后端

### 9.1 统一包装器

新增一个实现 `nav_core::BaseLocalPlanner` 的统一包装器，在进程启动时预实例化后端：

1. `SingleTopologyTEB`；
2. `TopologyLockedHCP`；
3. `AckermannManeuverPlanner`。

包装器负责统一输入、时限、轨迹输出、健康状态和安全切换，不在动态重配置回调中重建
planner 或 footprint。

### 9.2 高速巡航后端

使用单拓扑 TEB，主要机制：

- 增大前视距离和速度/加速度 Anchor；
- 依据全局路径曲率和停车包络生成速度上限；
- 降低无必要角速度自由度和航向振荡；
- 强化带符号横向/航向误差跟踪；
- 只对前向突发障碍和停车距离做强制限速；
- 侧向或远距离障碍不提前触发硬降速。

### 9.3 静态密集后端

使用 HCP 与独立拓扑管理器：

- 将密集障碍点压缩为线段/多边形，控制 HCP 时限；
- 为每条候选轨迹计算稳定 `topology_id` 或 H-signature；
- 第一次选择左/右绕行后显式锁定；
- 高频优化当前拓扑的轨迹细节；
- 仅在当前拓扑不可行、地图版本变化、风险超阈值或目标变化时解锁；
- 输出 topology switch、lock reason、候选代价和 deadline。

不能只依赖时间 blocking 代替拓扑锁。现有 `selection_cost_hysteresis > 1` 可能使旧解代价
变大，必须通过单元测试明确其实际偏置方向。

### 9.4 狭窄通道后端

使用中心线约束的单拓扑 TEB：

- 从平行墙或自由空间骨架估计走廊轴线和宽度；
- 发布中心线 via-points；
- 根据可用半宽限制可配置净空：

      min_obstacle_dist <=
        (corridor_width - robot_width) / 2 - uncertainty_margin

- 前方障碍决定减速/停车；
- 侧墙只进入低速走廊包络，不触发前方 emergency；
- 入口偏置、门框、渐窄、L/S 形和正前封堵必须分别验证。

当前临时 `_corridor_active` latch 不能作为 V2 实现；reset 必须清除 episode-local 状态，
正式监督器应使用显式 enter/exit 和驻留合同。

### 9.5 死角机动后端

实现真正的 Ackermann Hybrid A* 或 state lattice：

    state = (x, y, yaw, gear)

运动基元必须满足最小转弯半径、碰撞和可用后向净空约束。代价至少包含：

- 轨迹长度；
- 曲率与曲率变化；
- 换挡惩罚；
- 倒车距离；
- 终姿态误差；
- 障碍风险；
- 搜索时间。

输出带 gear 的 `ManeuverPlan` 分段。每个 cusp 执行：

    brake -> confirm zero speed -> reset segment warm start
          -> switch gear -> confirm sensors -> execute next segment

TEB 只低速细化单一 gear 段，不能依赖 carlike TEB 中无效的
`weight_kinematics_forward_drive` 抑制前后振荡。后向传感器覆盖无效或 stale 时禁止 reverse。

### 9.6 后端切换协议

结构切换必须原子执行：

1. 请求切换并停止接受新性能动作；
2. 取消或冻结当前局部轨迹；
3. 发布减速并确认零速；
4. 确认 action server、感知和 TF 健康；
5. reset 目标后端 warm start；
6. 应用目标模式 Anchor/profile；
7. 等待参数 readback 和新轨迹 activation；
8. 恢复运动并记录完整切换事件。

任一步超时进入 fail-closed，不得在运动中重建 planner 对象。

## 10. 双层安全系统

### 10.1 参数级安全层

必须保留：

- 类型、维度、有限数值和参数支持检查；
- box、变化率和 Ackermann 耦合；
- `NORMAL/WARNING/EMERGENCY/FAULT` 状态机；
- 原子参数事务、ack/readback/activation；
- 最后一次已确认安全快照；
- interface fault 的 fail-closed 行为。

新增：

- 横向加速度和曲率速度约束；
- 转向角速度/转向速率约束；
- 前/侧/后方向安全语义；
- reverse 的后向覆盖和陈旧度门控；
- 模式/profile 事务的一致性检查。

### 10.2 轨迹与命令级预测安全盾

推荐数据流：

    raw TEB/maneuver trajectory + predicted obstacle occupancy
      -> time-indexed swept-footprint collision check
      -> find largest safe speed scale alpha in [0, 1]
      -> scaled command or brake

安全盾应最小化对原轨迹的干预，不应重新承担全局规划。至少记录：

- `alpha`；
- 预测最小 TTC/净空；
- 触发 track ID 或静态障碍 ID；
- 干预原因；
- 干预开始、持续和释放时间；
- 原始与最终命令。

在没有严格可证明控制屏障函数之前，只能称为“预测最小干预安全盾”，不得宣称严格 CBF。

### 10.3 不可变安全规则

- health/fault 停车优先于任何模式和 RL 输出；
- emergency 人工链路优先于软件恢复；
- reverse 必须有后向感知、低速上限和 cusp 零速确认；
- 动态目标消息陈旧时不得继续使用旧速度外推；
- 任何参数/后端事务未 activation 时不得把结果归因给当前动作；
- 安全干预释放必须平滑，不能直接跳回 Anchor；
- 实车配置的制动、时延、净空和速度边界未标定时保持 TBD，不得使用仿真候选替代。

## 11. ROS 接口指南

### 11.1 建议命名空间

| Topic/Service | 类型方向 | 用途 |
| --- | --- | --- |
| `/nav_world_model/tracks` | publisher | 统一动态 track 和预测 |
| `/nav_world_model/geometry` | publisher | 走廊、死角、方向净空和路径几何 |
| `/nav_world_model/health` | publisher | 感知、TF、时间同步和 stale 状态 |
| `/teb_mode_manager/context` | publisher | 几何模式、动态覆盖层和置信度 |
| `/teb_mode_manager/transition` | publisher | 驻留、切换原因和进度 |
| `/teb_mode_manager/set_profile` | service/action | 类型化模式参数事务 |
| `/teb_rl_v2/action_trace` | publisher | commanded/feasible/safe/executed |
| `/hybrid_local_planner/trajectory` | publisher | 统一时间化局部轨迹 |
| `/hybrid_local_planner/status` | publisher | backend、topology、gear、deadline |
| `/predictive_shield/status` | publisher | 风险、speed scale 和干预原因 |
| `/cmd_vel_raw` | internal input | 未经过最终安全盾的规划命令 |
| `/cmd_vel` | final output | 经过预测安全盾的底盘命令 |

TEB 已有自定义动态障碍接口：

    /move_base/TebLocalPlannerROS/obstacles
      costmap_converter/ObstacleArrayMsg

V2 world model 应通过独立 bridge 发布该消息，并同时保留信息更完整的 V2 track 消息。

### 11.2 消息共同头

所有 V2 自定义消息至少包含：

- `header.stamp` 和 `header.frame_id`；
- `architecture_generation = v2`；
- `run_id`、`episode_id`、`step_id`；
- `world_model_seq`、`mode_seq`、`config_seq`；
- source stamp、processing stamp 和 age；
- validity、stale、fault reason；
- schema version。

### 11.3 时间和坐标合同

- 所有动态位置和速度必须声明 frame；
- velocity 的旋转与位置的平移/旋转必须使用相同消息 stamp；
- 不允许将旧 track 与新机器人位姿静默组合；
- 模式决策时间必须晚于其输入世界模型时间；
- reward 窗口从参数/后端 activation 后开始；
- 一次 transition 只能归因一个 `config_seq` 和一个稳定 backend activation；
- 时间倒退、TF 外推过大或 sequence 不单调时丢弃 transition 并进入诊断/故障路径。

## 12. V2 状态合同

V2 observation 不在设计阶段冻结具体维度，但必须按命名字段组成并版本化。

### 12.1 几何与运动

- 激光扇区低分位值及角域；
- 前/左/右/后 footprint 净空；
- 障碍密度和静态持久性；
- 走廊宽度、轴向、墙平行度；
- dead-end score；
- 带符号路径横向误差和航向误差；
- 路径曲率和目标方向稳定度；
- 线/角速度、线/角加速度；
- goal distance 和 bearing。

### 12.2 动态风险

可采用 top-K tracks 加风险池化：

- 相对位置和相对速度；
- TTCA、预测最小净空；
- crossing/head-on/follow 概率；
- 左/中/右预测占据；
- track confidence、age、miss count 和 covariance；
- 聚合的最高风险 track ID。

### 12.3 模式与执行上下文

- GeometryMode one-hot；
- DynamicOverlay one-hot；
- mode confidence、dwell、transition progress；
- backend、topology_id、gear；
- 当前 Anchor/profile ID；
- `action_commanded/feasible/safe/executed` 的上一时刻摘要；
- projection/safety reason mask；
- safety mode one-hot；
- planner/sensor/TF/localization/interface health；
- shield speed scale 和最近干预持续时间。

### 12.4 历史窗口

历史窗口必须覆盖：

- 至少一个完整模式决策周期；
- 参数 request 到 activation 的延迟；
- 动态目标短时运动趋势；
- 上一次安全干预和释放过程。

窗口长度和采样间隔由 pilot 的实际时延与自相关确定。验证/测试数据不得参与 normalization。

## 13. 各场景运行合同

### 13.1 高速巡航

进入条件示例：

- 低障碍密度；
- 前向净空和停车包络充分；
- DynamicOverlay 为 NONE 或低风险远离；
- 路径曲率低且目标方向稳定；
- 感知、定位和规划健康。

行为：

- 使用 cruise Anchor；
- 曲率和停车距离共同限制速度；
- 提高直线效率，抑制角速度波动；
- 对远侧障碍不提前降速；
- 一旦预测风险或曲率升高，平滑退出而非急跳参数。

### 13.2 动态交互

行为由 overlay 决定：

- HEAD_ON：提前减速、扩大预测净空，保留转向敏捷度；
- CROSSING：评估交叉点和时间窗，选择前过、后过或等待；
- FOLLOW：控制时距，判断跟随或在拓扑允许时绕行；
- OVERTAKE_OR_YIELD：比较候选轨迹风险和延迟；
- 远离目标：通过释放时间常数快速但平滑解除约束。

安全与奖励都应依据相对运动和预测占据，不能只依据瞬时距离反复急停。

### 13.3 静态密集绕行

- 增大静态前视范围；
- 第一次选择可行拓扑后锁定；
- 地图不变且拓扑可行时禁止无理由切换；
- 高频更新轨迹细节，低频改变 topology_id；
- 减少全局重规划、倒车、recovery 和角速度振荡。

### 13.4 狭窄通道

- 依据实测可用宽度判断可行性；
- 走廊中心线与路径跟踪优先；
- 侧墙近不等于前方碰撞；
- 正前封堵必须停车；
- 不可行宽度应明确判定不可达，不能无限降低净空约束；
- 出口后通过滞回释放 corridor profile。

### 13.5 死角机动

- 只有常规前向后端不可行且后向感知健康时才能进入；
- 规划带 gear 的有限段机动；
- 每次换挡先停车确认；
- 限制换挡次数和反复 F-R 抖动；
- 后向覆盖丢失立即停止并退出自动倒车；
- 不可达时返回明确失败，不进行无限 recovery。

### 13.6 混合过渡

- 记录每次模式切换和触发原因；
- Anchor 从上一执行安全参数连续插值；
- 参数变化率和 shield 释放率受限；
- 低置信度阶段使用 Balanced；
- 过渡窗口单独统计速度、角速度冲击、projection 和安全干预。

## 14. 软件骨架

### 14.1 新增包

    src/perception/nav_world_model/
      CMakeLists.txt
      package.xml
      msg/
        TrackedObstacle.msg
        TrackedObstacleArray.msg
        PredictedState.msg
        LocalGeometry.msg
        WorldModelHealth.msg
      config/
      launch/
      src/
        dynamic_obstacle_tracker_node.cpp
        local_geometry_node.cpp
        teb_obstacle_bridge_node.cpp
      test/

    src/application/teb_mode_manager/
      CMakeLists.txt
      package.xml
      msg/
        ContextState.msg
        ModeTransition.msg
        ParameterTransaction.msg
      config/
        anchors/
        profiles/
      launch/
      src/teb_mode_manager/
        context_supervisor.py
        anchor_bank.py
        transition_manager.py
        typed_parameter_registry.py
        profile_transaction.py
      test/

    src/application/m2_hybrid_local_planner/
      CMakeLists.txt
      package.xml
      include/
      src/
        hybrid_local_planner_ros.cpp
        backend_manager.cpp
        single_topology_teb_backend.cpp
        topology_locked_hcp_backend.cpp
      test/

    src/application/m2_maneuver_planner/
      CMakeLists.txt
      package.xml
      include/
      src/
        hybrid_astar.cpp
        motion_primitives.cpp
        maneuver_segment_executor.cpp
      test/

### 14.2 扩展现有 `teb_rl_tuner`

建议新增：

    src/application/teb_rl_tuner/src/teb_rl_tuner/
      v2_parameter_registry.py
      feasible_action_decoder.py
      execution_aware_replay.py
      mode_conditioned_sac_env.py
      predictive_shield.py
      v2_state_schema.py

必须继续复用并版本化，而不是复制：

- `StateBuilder` 和 `HistoryWindow` 的同步/历史骨架；
- `TrainingEnvironment` 的 reset、activation 和 fail-closed 生命周期；
- `TebParameterClient` 的 request/ack/readback/snapshot 事务语义；
- `SafetyMarginFilter` 的四态状态机；
- checkpoint、VecNormalize、manifest、checksum 和失败留存；
- T12 已修复的 episode boundary atomicity、activation barrier 和静态 footprint 生命周期。

V1 精确九参数类和配置继续保留。V2 使用新类名和 schema version，禁止通过修改常量让 V1
历史运行失去可复现性。

### 14.3 V2 实验目录

    config/thesis_experiments/v2/
      architecture_contract.yaml
      parameter_registry.yaml
      mode_thresholds.yaml
      anchors/
      profiles/
      safety/

    experiments/manifests/v2/
      scenes/
      splits/
      preregistrations/
      amendments/

    artifacts/v2/
      calibration/
      component_acceptance/
      training/
      evaluation/
      shadow/

生成的 artifacts、bag、build、devel、install 和 log 不进入 Git。

## 15. 场景库合同

### 15.1 必须新增的场景族

| 场景族 | 最低覆盖 |
| --- | --- |
| Cruise | 30--60 m 长直路、缓弯、侧向远障碍、末端减速 |
| Dynamic | head-on、crossing、follow、stop-go、远离、多目标 |
| StaticDense | 货架、3--8 柱、左右非对称、交替绕行、部分堵路 |
| Corridor | 门框、偏置入口、渐窄、L/S 形、正前封堵、不可行宽度 |
| Maneuver | 死胡同、U-bay、货架巷堵塞、目标在后方、不可达 |
| Transition | 巡航到人群、走廊到大厅、静态区到死角、动态目标进入/离开 |

当前 2.5--4 m clear scene 不得作为高速巡航主结果场景。

### 15.2 manifest 必需字段

- scene ID、family、split 和 seed；
- world/生成器版本及 hash；
- 起终点和 timeout；
- 机器人候选几何与 footprint hash；
- 静态障碍参数；
- 动态 agent 行为及随机化；
- 传感器、时延和执行器随机化；
- 碰撞与成功判据；
- 可行性标签仅供 evaluator 使用；
- 禁止暴露给运行时策略的字段清单。

## 16. V2 实施里程碑

### V2-00：基线冻结与隔离

任务：

- 记录主仓、子模块和 dirty 状态；
- 建立 source-only 可复现快照；
- 明确 V1 与 V2 的配置、类名、schema 和 artifact 根目录；
- 不删除既有 artifacts 或失败证据。

退出条件：

- V1/T12 hash 和测试不漂移；
- V2 新文件不会被 V1 runner 自动加载。

### V2-01：机器合同与消息骨架

任务：

- 创建 V2 architecture contract、parameter registry 和消息；
- 冻结 GeometryMode、DynamicOverlay、TransitionState 和 action trace 语义；
- 为 schema 和配置编写 fail-closed validator。

退出条件：

- 配置、消息字段和枚举测试通过；
- 缺失、额外、NaN、错误类型和 schema 漂移均被拒绝。

### V2-02：可信仿真动力学与场景系统

实施状态：`COMPONENT COMPLETE`。机器证据为
`artifacts/v2/component_acceptance/v2_02_acceptance.yaml`；该状态不等于规划器或策略
性能已经验证，且所有运行阈值继续保持 `runtime_ready=false`。

任务：

- 为 M2 插件加入驱动/转向一阶滞后；
- 加速度、减速度、转向速率和命令 timeout；
- 可配置命令延迟/抖动、传感器延迟/噪声和接触碰撞；
- 实现 V2 场景生成器和 evaluator。

退出条件：

- 直线、定圆、制动、倒车和延迟回归可重复；
- 高速场景中的停车距离不再恒为 0；
- 不使用未标定仿真值声称实车安全。

### V2-03：世界模型和规则监督器

实施状态：`COMPONENT COMPLETE`。已实现基于 LaserScan/里程计/TF 的局部几何、聚类跟踪、
常速度预测、健康检查，以及不读取场景标签的规则监督器。机器证据为
`artifacts/v2/component_acceptance/v2_03_acceptance.yaml`。当前跟踪器和阈值仍是未标定
仿真候选，不能解释为生产感知或实车能力。

任务：

- 实现静态几何、tracker、预测和健康状态；
- 先接 `CostmapToDynamicObstacles` 做 MVP；
- 使用 Pedsim 真值评估跟踪误差；
- 实现规则监督器、滞回和 Balanced fallback。

退出条件：

- 模式混淆矩阵、跟踪 RMSE/ID switch 和预测误差可计算；
- 运行时不读取 manifest 标签；
- stale/TF 故障 fail closed。

### V2-04：Anchor Bank 和无训练闭环

实现记录（2026-07-14）：软件与 shadow 事务门已通过。当前包含 6 Anchor、5 个 factorized
dynamic overlay、20 参数 `double/int/bool` profile、约束内生 feasible decoder、从上一
`executed` 出发的 rate-limited 事务和完整 action trace。800 周期零训练规则验收的正常
projection 为 0，连续越变化率跳变为 0；ROS topic 探针也通过。Anchor 仍是未正式标定的
simulation candidate，执行后端固定为 `deterministic_shadow`，因此不得把本记录解读为完成
Anchor 独立优化、TEB 参数在线闭环或导航效果提升，所有部署合同继续 `runtime_ready=false`。

任务：

- 独立标定各模式 Anchor；
- 实现类型化 profile 和从上一 executed 参数开始的平滑过渡；
- 接入 feasible decoder 和 action trace。

退出条件：

- 无训练 pilot 的正常 projection rate 低于 10%；
- 模式切换无参数跳变；
- command/feasible/safe/executed 可完整重建。

### V2-05：专用规划机制

按单因素顺序实现：

1. 动态 tracker 到 TEB bridge；
2. 静态 topology lock；
3. corridor centerline 与方向安全；
4. Ackermann maneuver planner；
5. predictive motion shield。

退出条件：

- 每个机制都有独立基线和消融；
- 不通过的机制不得进入全系统组合；
- planner deadline、崩溃和接口故障门全部通过。

### V2-06：执行感知模式残差学习

任务：

- 实现 mode-conditioned actor 和 execution-aware replay；
- specialist 分模式训练；
- 混合过渡联合微调；
- 只登记一个 V2 学习 amendment 后启动 bounded pilot。

退出条件：

- 两个 pilot seed 的 validation 都改善；
- projection 和 command-to-execution gap 达标；
- 未达标时停止，不扩大预算。

### V2-07：冻结配对与系统消融

完成本指南第 17 节的完整矩阵、统计和失败审计。

### V2-08：rosbag、shadow 与实车门禁

先 offline replay，再 live shadow。实车闭环必须重新完成标定、现场审批、低速门禁和人工
急停演练，不能继承 Gazebo 的高速参数。

## 17. 实验矩阵

### 17.1 主比较组

1. `TEB-Tuned-V1`：固定单配置；
2. `Residual-SAC-V1`：当前单 Anchor 方法；
3. `V2-RuleMode-AnchorBank`：规则模式、多 Anchor、无 RL；
4. `V2-ModeResidual-NoPredictiveShield`；
5. `V2-Full`：完整系统。

### 17.2 机制消融

只在对应场景运行：

- `V2-NoTracker`；
- `V2-OracleTracker`；
- `V2-NoTopologyLock`；
- `V2-NoCorridorCenterline`；
- `V2-NoManeuverBackend`；
- `V2-NoExecutionAwareReplay`；
- `V2-NoTransitionBlend`；
- `V2-NoPredictiveShield`，仅 Gazebo。

动态障碍建议固定四级比较：

1. 静态 TEB；
2. oracle 动态 TEB；
3. 估计轨迹动态 TEB；
4. 估计轨迹 + V2 多模式残差。

这样可以分离动态信息上界、真实感知误差和策略决策的贡献。

### 17.3 公平性

- 相同 scene/seed 使用 common random numbers；
- 相同机器人模型、global planner、costmap 和成功/碰撞判据；
- validation-only 模型选择；
- test 不参与 Anchor、阈值、reward 或 checkpoint 选择；
- episode 是统计独立单位；
- 分场景报告，不用宏平均掩盖局部退化；
- 全部失败保留并进入 failure index。

## 18. 指标与建议效果门槛

门槛需在正式运行前根据 pilot 方差冻结。以下数值是 V2 首版 preregistration 的建议起点，
不是已经获得的结果。

### 18.1 系统总门

- 0 未解释进程崩溃和接口崩溃；
- 0 记录碰撞，或相对最佳安全基线不退化；
- success rate 的 95% CI 下界相对最佳安全基线不退化超过 2 percentage points；
- 最小净空不退化超过 0.05 m；
- 正常无安全事件 projection rate <10%，硬上限 <20%；
- planner/control deadline 无 >200 ms 未处理超期。

### 18.2 高速巡航

- median navigation time 改善至少 15%；
- 平均速度显著提高；
- 横向 RMS、航向振荡和末端停车不退化；
- 不必要减速次数下降至少 30%。

### 18.3 动态交互

- 碰撞率不退化；
- 不必要停车下降至少 30%；
- 最小预测 TTC 和净空不退化；
- 横穿/让行成功率提高；
- 报告 tracker RMSE、ID switch、预测误差和 stale rate。

### 18.4 静态密集

- topology switch 下降至少 50%，目标 60%；
- global replan/recovery 下降至少 50%；
- 时间或路径长度改善至少 10%；
- 倒车和前后切换不增加。

### 18.5 狭窄通道

- 可行通道 success >=95%；
- 正前封堵停车率 100%；
- 误 emergency <=2%；
- 横向 RMS <=0.08 m 或不超过可用半宽的 10%；
- 角速度振荡积分下降至少 30%。

### 18.6 死角机动

- success >=85%，且相对当前前向后端提高至少 25 percentage points；
- median gear switch <=3，p95 <=5；
- 无持续 F-R 抖动；
- escape time 下降至少 30%；
- 后向感知失败时 100% 禁止继续倒车。

### 18.7 混合过渡

- 无模式快速抖动；
- 统计 mode switch、dwell、参数总变差；
- 切换窗口无不可接受速度/角速度冲击；
- 切换窗口 projection 和安全干预显著低于 V1；
- 报告从风险出现到模式生效的 transition latency。

## 19. 组件时限门

建议首版：

- Corridor 单 TEB：10 Hz 下 p99 <80--100 ms；
- 压缩障碍后的 Static HCP：p99 <100 ms，不得出现 >200 ms 未处理周期；
- Maneuver 搜索：p95 <250 ms，1 s 硬超时并安全停车；
- world model/tracker：p99 小于其输入周期；
- mode/SAC 推理：p99 <50 ms；
- predictive shield：必须在下一个控制命令发布前完成。

具体阈值以实测 controller frequency 和硬件性能冻结。

## 20. 开发测试清单

### 20.1 单元测试

- 模式 enter/exit、滞回、驻留和 Balanced fallback；
- corridor episode-local 状态 reset；
- LaserScan angle_min/max/increment 和非 360 度覆盖；
- track association、坐标变换、stale 和 covariance；
- feasible decoder 的 Ackermann、distance 和 positive weight 约束；
- Anchor 从上一 executed 参数平滑过渡；
- action 四阶段语义和 replay 写入；
- topology ID 稳定、lock/unlock 原因；
- Hybrid A* 曲率、gear、碰撞和不可达；
- cusp 零速确认；
- reverse 后向覆盖门；
- predictive shield speed scale；
- 参数/profile 事务的原子性和 timeout。

### 20.2 集成测试

- world model 到 TEB dynamic obstacle bridge；
- 动态目标不重复进入 static/dynamic 两层；
- backend switch 的 stop-reset-activate-resume；
- safety WARNING 后不回跳固定 Anchor；
- planner recovery 与参数事务不并发破坏；
- 传感器/TF/定位/接口故障 fail closed；
- episode reset 清除所有 episode-local latch；
- manifest、CSV、checksum 和 failure retention。

### 20.3 回归测试

每个 V2 阶段必须继续通过：

- V1 Python/config 测试；
- TEB fork 原生测试；
- m2_gazebo 直线、转弯、倒车和固定 TEB 回归；
- parameter client 事务/快照恢复；
- training environment boundary/activation 测试；
- V1 runner 不加载 V2 配置的隔离测试。

## 21. 已知高风险项

1. TEB dynamic obstacle edge 在当前 fork 中仍带 experimental 性质，且内部主要是常速度外推；
2. 自定义障碍转换若忽略消息 stamp，会对高速目标产生时空错位；
3. rolling costmap 的定位抖动可能被 CostmapToDynamicObstacles 误判为目标运动；
4. 长墙密集点曾使 HCP 产生约 1 s 规划周期，必须先做障碍压缩和时限门；
5. carlike TEB 不依赖 `weight_kinematics_forward_drive`，不能靠该权重阻止换挡振荡；
6. 当前现有 Kinodynamic A* 缺少完整 yaw、gear 和 steering 状态，不能冒充 V2 机动器；
7. 仿真与实车倒车转向符号语义存在差异，标定前禁止 sim-to-real 声明；
8. 实车后向激光覆盖、盲区和 stale 检测尚未冻结；
9. V1 默认运动学仿真仍会即时执行速度/停车；V2-02 候选模型已有非零制动和时延，
   但任何高速收益仍必须在该模型上重新验证，且不得外推为实车安全结论；
10. 当前 worktree 包含大量用户文件、实验 artifacts 和 dirty 子模块，不得通过 reset/clean
    为 V2 建立“干净环境”。

## 22. 明确禁止的实现捷径

- 不把 SAC 原始动作从 5/9 维直接扩大到 20 个 TEB 参数；
- 不用一个 monolithic policy 从零同时学习全部模式；
- 不让 RL 直接选择 emergency 状态或发布 `/cmd_vel`；
- 不每秒热切换 HCP、costmap plugin、footprint 或 planner 对象；
- 不以 Pedsim 真值输入证明动态感知有效；
- 不用瞬时净空/自身速度 TTC 代替相对运动预测；
- 不把“停止更新局部规划器”作为静态拓扑保持；
- 不用时间 blocking 代替稳定 topology ID 与显式锁；
- 不用普通 TEB 的倒车初值冒充完整 F-R-F 死角机动；
- 不在当前短 clear scene 上宣称高速巡航提升；
- 不把 14/14 goal 或单 seed return 改善写成学习有效；
- 不在失败后扩大训练预算寻找有利 seed。

## 23. Definition of Done

V2 只有同时满足以下条件才算系统完成：

1. 五类场景和混合过渡都有冻结 manifest、ID/OOD 测试和失败案例；
2. 世界模型输出可由原始 sensor/log 重建，动态 tracking 有 oracle 对照；
3. 模式和 overlay 不读取场景标签，置信度、滞回和切换可审计；
4. 各后端满足运动学、碰撞和时限门；
5. commanded/feasible/safe/executed 动作语义贯穿策略、replay 和日志；
6. 正常 projection 率达到门槛，安全干预平滑释放；
7. 全系统相对 V1、规则多 Anchor 和各机制消融都有配对结果；
8. success/safety 总门通过，并至少在三个目标场景族达到预注册的明显效果；
9. 所有论文数值可追溯到原始 CSV、配置、checkpoint、脚本和 SHA256；
10. rosbag/offline/shadow 通过后，实车仍需单独现场审批和低速门禁。

“程序能够启动”或“某个 seed 到达率高”都不等于 V2 完成。

## 24. 第一实施批次

在开始任何新训练前，第一批次只做以下工作：

1. 建立 V2 source/config/artifact 隔离和机器合同；
2. 定义 V2 消息、枚举、参数注册表与 validator；
3. 修复 corridor episode-local latch 和 LaserScan 角域合同；
4. 建立 commanded/feasible/safe/executed action trace；
5. 升级 M2 仿真执行器、制动和时延模型；
6. 增加 30--60 m Cruise、动态 crossing 和基础 dead-end 场景；
7. 实现规则监督器与多 Anchor 无训练闭环；
8. 将正常 projection rate 压到 10% 以下。

只有该批次通过，才进入动态 tracker、拓扑锁、机动后端和模式残差训练。

## 25. 论文方法表述建议

V2 方法可统一表述为：

> 本文提出一种面向异构导航场景的因子化自适应多模式 TEB 框架 FAM-TEB。该框架利用
> 局部世界模型分离描述几何通行结构与动态交互风险，通过带滞回的场景监督器选择参数
> Anchor 和规划后端，并由执行感知的模式条件残差策略在可行域内调节性能强度。系统使用
> 拓扑锁定、走廊中心线约束和带挡位 Ackermann 机动规划分别处理静态密集、狭窄通道和
> 死角场景，同时采用参数级约束解码与预测最小干预安全盾保证运动学、碰撞和接口安全。

正式论文采用该表述前，必须先完成 V2 机器合同 amendment、系统实现和冻结配对证据。
