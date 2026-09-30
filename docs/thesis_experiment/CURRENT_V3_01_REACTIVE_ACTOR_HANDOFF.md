# V3-SCOPE V3-01 反应式 actor / 克隆干预资格交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`v3_01_r2_crossing_reactive_actor_qualification_pass`

## 1. 结论

V3-01 完成了确定性 crossing 场景的 actor-response 数据生成资格。最终 R2 使用三类隐藏
policy（cautious/assertive/hesitant）、四条直线 Ackermann 可行 ego intervention、三个全新
seed，共 36 个 attempt=1 单元；36/36 完成，未重试。独立复核重放全部 raw CSV 后通过。

该结论只证明确定性 crossing fixture 能从同一克隆初态产生稳定且与候选轨迹相关的 actor
响应。它不证明随机 actor 已标定、其他场景族可识别、预测器准确、规划器安全、性能提升或
实车可用。

## 2. 自纠正链

- V3-01：首单元因 runner 错把 Gazebo Classic `-u` 当成 unpause 而超时；实际 `-u` 是
  pause。1 个单元消费、35 个作废，未重试。
- V3-01-R1：移除 pause flag 后 36/36 完成；9/9 响应效应块通过，但人为加入的 10%
  response-margin seed jitter 造成最大跨 seed 差异 0.2244 m，超过冻结 0.20 m 门，终态失败。
- V3-01-R2：只移除未经标定的 threshold jitter，不改变场景、actor policy dynamics、
  候选、记录器或门；使用 fresh seeds 6121--6123，36/36 完成并通过。

所有失败、旧 release、seed 和预算均保留且不复用。

## 3. R2 冻结结果

- 9/9 policy×seed block 通过；
- 4 s actor 位置差异：最小 0.777245 m，最大 2.310975 m；
- 每个 block distinct response signatures：至少 3；
- 最大初态位置误差：0.006 m（门 0.01 m）；
- 最大 pre-intervention spread：0.00243 m（门 0.02 m）；
- ego crossing-time 最大误差：0 s（门 0.10 s）；
- 最大跨 seed actor response spread：0 m（门 0.20 m）；
- formal_result=false，training=false，real_vehicle=false。

核心 SHA-256：

| 资源 | SHA-256 |
|---|---|
| R2 release | `cdf2489a925675273b6a41ded1e5fdc5c7b4ac4cf14053d11b8d57a61716ac0b` |
| R2 release review | `9e44a88204136a568b56e8be31f36283b3a4cf911fb7c7fd68f25d566f2130eb` |
| R2 assessment | `23638408651ac8ebf1f8dead619ff2fe656fdd3426d792b976944c62ddd60d90` |
| R2 independent result review | `f9363de30f018208d3e2adf0c9cdbe6b19e4c9b2708ec3eba356f03bca2d0ffd` |
| R2 execution freeze | `053359e737d3d945f1d7ecd931cea0047c8591e3ff8e65e09ee51cd7c7f9b4be` |

## 4. 新增实现

- `m2_gazebo/v3_reactive_actor.h`：candidate-blind 三类响应动力学；
- `libv3_reactive_actor_plugin.so`：只读取 ego pose history、conflict point 与 sim time；
- `libv3_episode_recorder_plugin.so`：20 Hz 双方位姿 raw CSV 与原子终止标记；
- V3-01/R1/R2 exact schedule、hermetic bindings、release、authorization、journal、raw replay
  与 independent review。

actor plugin 不读取 candidate ID、调度行、scene role 或 future truth。未启动 move_base、
训练、CAN/串口或实车。

## 5. 下一安全入口

下一阶段为独立 V3-02：

1. 基于 candidate trajectory 与可观测 actor history 的 candidate-conditioned predictor；
2. 用独立 calibration/test 数据构建保守 reachable-occupancy envelope；
3. hidden policy 只能用于 evaluator 分层，不能作为 predictor runtime 输入；
4. 先做纯离线数据/模型/统计设计与独立 review，再单独申请仿真或训练预算；
5. 若数据不足以做随机/多场景校准，必须保留确定性 crossing 限定，不得外推。

