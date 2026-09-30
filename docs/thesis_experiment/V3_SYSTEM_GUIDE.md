# V3-SCOPE System Guide

日期：2026-07-25  
代号：V3-SCOPE  
当前阶段：V3-12-P1 actual-TEB terminal paper-evidence closure  
当前权限：NO NEW EXECUTION / NO TRAINING / NO REAL VEHICLE

## 0. 当前终态

V3-00--V3-12-P1 已完成。最新确认性证据来自 V3-11-F1：fresh seeds 7101--7130
构成 30 个 scene-policy-seed 配对 blocks，三个冻结仿真模板、cautious/hesitant 两种
evaluator-only actor policies、两种方法共 60 episodes，全部 attempt=1 完成。V3
spatial-safe 与 actual `TebLocalPlannerROS` 0.8.4 的平均受限 gate-to-goal 时间分别为
6.7683 s 与 7.7733 s，配对相对下降 12.9288%，单侧 97.5% paired-bootstrap lower
为 12.7253%；双方目标成功均为 100%，contact 为 0，六个 scene×policy strata 全为正，
全部预注册完整性、安全、任务与效应门通过。

正式 assessment SHA 为
`9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`，
独立 review SHA 为
`2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39`。
V3-12-P1 paper evidence manifest SHA 为
`319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`。

该结果只适用于三个冻结 single-reactive-actor Gazebo 模板、cautious/hesitant policy 和
双方共享的 oracle actor track。它证明限定总体内相对 actual TEB 的收益，但不证明感知、
multi-actor/OOD、形式化安全、训练收益、部署或实车性能。`runtime_ready=false`，
`training_authorized=false`，`real_vehicle_ready=false`。

## 1. 研究问题与证据边界

V3-SCOPE 研究以下问题：

> 在不改变 Autolabor M2 硬件和最终命令安全链的前提下，候选轨迹条件交互预测、
> 校准可达占据和硬安全过滤，能否在 family-held-out 动态交互场景中，相对最强公平
> 固定 TEB 减少不必要让行和交互完成时间；学习排序器能否在解析排序器之上产生稳定、
> 可归因的额外收益？

V3 不延续 V2 的性能搜索身份、seed、release、预算或 winner 选择。V2 的终态结论保持为
“未证明明显提升”。V3 使用新的 package、config、manifest、artifact 和 stage namespace。

V3 全链已经证明限定仿真总体内的解析空间候选性能改善，但仍不证明：

- actor 模型代表真实人群行为分布；
- predictor 在其他交互 family 中保持校准；
- planner 已作为连续 ROS 导航节点在复杂地图运行；
- safety filter 具有形式化或实车保证；
- V3 在其他场景族、多人交互或真实感知条件下仍相对 actual TEB 有性能优势；
- 系统可部署或可用于实车。

## 2. 不变的硬件与命令安全链

保留现有 M2 底盘、Livox 激光、GPS、里程计、相机和稳定导航控制权边界。未来 V3 tracker
只能发布到：

```text
/cmd_vel_navigation
  -> GPS goal speed limiter
  -> /cmd_vel_gps
  -> FOD mode arbiter
  -> /cmd_vel
  -> M2 driver sanitizer
```

任何 V3 节点都不得直接发布 `/cmd_vel`、控制 M2 driver、绕过终点 stop latch、FOD 仲裁或
GPS 速度限制。真实运动、CAN/串口和在线实车参数写入仍需现场逐次批准。

## 3. 系统架构

```text
/scan + /odom 或 /gps/odom + TF + global path + costmap + health
                              |
                              v
 scene_graph_world_model（静态几何、动态 track history、不可变 snapshot）
                              |
                              v
 Ackermann candidate generator（8--12 条、certified stop、TEB fallback）
                              |
                              v
 candidate-conditioned predictor p(other future | snapshot, ego candidate)
                              |
                              v
 calibrated reachable occupancy tubes + distribution-shift monitor
                              |
                              v
 hard verifier（曲率、转角率、净空、制动、时延、健康、backup viability）
                              |
                              v
 safe candidate set -> analytic ranker -> optional bandit -> gated offline RL
                              |
                              v
 trajectory tracker -> /cmd_vel_navigation -> 现有安全命令链
```

学习器的动作是 `candidate_id`，不是高维 TEB 参数。所有候选在进入 policy 前已通过
Ackermann 可行性检查。安全拒绝必须显式记录为 rejection event；不得像历史参数投影那样
静默把一个 commanded action 改成另一执行动作。

## 4. ROS 接口

独立 catkin package：

```text
src/application/v3_scope_planner/
```

package 定义五类消息：

- `TrajectoryPoint`：带时间的 pose、速度、加速度、转角、转角率和曲率；
- `CandidateTrajectory`：单条候选及 generator digest、stop/fallback 属性；
- `CandidateSet`：不可变 scene snapshot 下的候选集合；
- `CandidateEvaluation`：预测、占据管、安全判定、拒绝原因和延迟；
- `CandidateDecision`：选择结果、policy 身份、propensity 与 fallback 原因。

后续阶段已实现可重放的 Python candidate generator、predictor、hard verifier、解析 ranker
与仿真 dispatcher，但仍未提供可部署的连续 ROS node、trajectory tracker 或实车 launch。

## 5. 候选轨迹合同

候选点使用运动学自行车模型：

```text
x(k+1)   = x(k) + v(k) cos(yaw(k)) dt
y(k+1)   = y(k) + v(k) sin(yaw(k)) dt
yaw(k+1) = yaw(k) + v(k) tan(delta(k)) / wheelbase * dt
```

进入 policy 前必须满足：

1. 每个 set 的 candidate ID 唯一；
2. 时间严格递增，数值有限；
3. 速度、加速度、转角、转角率和曲率在冻结 simulation profile 内；
4. 离散运动学残差低于合同阈值；
5. 至少一条 certified-stop candidate；
6. 至少一条 TEB fallback candidate；
7. candidate set、vehicle model、global path 和 scene snapshot 都有 digest；
8. 决策必须引用同一 snapshot 与 candidate-set digest。

当前 0.65 m 轴距、1.20 m 最小转弯半径及动力学值只是已有 Gazebo
`simulation_candidate`，不自动成为实车标定值。

## 6. 反应式 actor 与 cloned intervention 数据合同

当前 V2 waypoint actor 是开环的。V3-01 必须建立 actor policy family，例如：

- `cautious`；
- `assertive`；
- `hesitant`。

actor hidden type 只允许 evaluator 分层使用，runtime policy、predictor 输入和 planner 不能读取。
每个 cloned block 必须保持同一初态 digest、同一 geometry/timing/actor hidden state，仅改变
ego candidate。每行必须记录：

- block、family、clone 和 candidate identity；
- immutable initial-state digest；
- candidate-set 与 selected-candidate digest；
- behavior propensity；
- actor hidden type（evaluator-only）；
- actor response trace；
- predictor output 与 occupancy tube；
- safety verdict/rejection codes；
- executed candidate 与 tracker deviation；
- outcome、terminal reason、raw resource path+SHA；
- process/environment/release identity。

只有在不同 ego candidate 引起的 actor response 大于预注册噪声地板、每类候选具有非零可审计
propensity、clone reset 通过且 runtime 不读取 future truth 后，才能声称“反事实数据资格通过”。
V3-00 的静态 fixture 只测试 schema 和 gate 逻辑，不是仿真证据。

## 7. Hermetic 执行边界

任何未来 execution release 必须在消费边界前完成：

1. 从 `env -i` non-login shell 启动；
2. 只按顺序 source `/opt/ros/noetic/setup.bash` 和本 workspace `devel/setup.bash`；
3. 绑定并哈希 source scripts、环境键值、Python executable、ROS executable；
4. 绑定 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH`、`PATH`；
5. 对每个 launch/package 执行 package-resolution probe；
6. package name、唯一 source root、`package.xml` SHA 和 launch SHA 全部匹配；
7. package probe 失败时不得越过 evidence consumption boundary；
8. 子进程环境不得继承凭据、代理、SSH agent、Git token 或 RabbitMQ secret；
9. 单次打开、component-wise no-follow、regular-file 和稳定读校验；
10. 将 environment fingerprint、package lookup receipt 和失败索引写入 raw journal。

V3-00 只冻结该协议和本地 source package map，不建立未来运行时环境快照。

## 8. 安全论证层级

硬确定层包括：

- 自车曲率、转角、转角率、速度、加速度和制动包络；
- footprint、tracker deviation 与命令新鲜度；
- health/TF/localization fail-closed；
- 至少一条可继续执行的 stop/backup trajectory。

概率层包括：

- candidate-conditioned actor prediction；
- calibrated reachable occupancy tube；
- coverage、tube area 与 OOD/shift monitor；
- 多 actor 组合误覆盖的显式规则。

预测均值或 conformal coverage 不能单独写成无条件碰撞零风险。一旦 health 无效、shift 触发、
tube 过宽、规划超时或无正常候选，必须进入 certified stop 或冻结的安全 fallback。

## 9. 阶段和停止门

### V3-00：纯离线 foundation

产物：本指南、机器合同、独立 package namespace、消息接口、hermetic 设计、actor/clone
数据合同、静态 fixtures、builder/reviewer 和 hash closure。权限始终为 0。

### V3-01：反应式 actor 与数据资格

需要独立 preregistration、fresh seed/schedule、预算、release 和仿真授权。失败时停止反事实
主张，转为 uncertainty-calibrated non-interactive planner。

### V3-02：预测与校准

比较 constant velocity、非条件模型和 candidate-conditioned predictor。必须同时通过反应方向、
collision-relevant error、NLL/Brier、coverage/tube area、held-out family 和 p99 延迟门。

### V3-03：无学习候选规划器

先用解析 cost 完成公平比较。若已通过正式门，先冻结结果，不为加入 RL 重开 test。

### V3-04：有条件启用学习排序

顺序固定为 supervised ranker、offline contextual bandit、最后才是 IQL/CQL。只有上一层存在
稳定长期 regret 且 action support/propensity 通过，才能升级。

### V3-05：一次性正式评测

在独立 geometry/actor-policy family block 上 paired，保留最强公平固定 TEB 与解析 planner。
建议目标仍为 point 至少 10%、单侧 lower 至少 5%，同时通过成功率、碰撞、急停、净空、
safety-intervention 和延迟门。最终阈值和样本量必须由新 preregistration 冻结。

V3-05-B0 已按该路径完成但形成有效阴性结果：270/270，点效应 1.2007%、lower 0.9647%，
V3 目标成功率 93.33%。该结果和全部身份保持冻结。

### V3-06：纵向单因素诊断与终止

V3-06 对 V3-05 的 goal/time failure 进行离线归因，并在有限 fresh calibration/
qualification 中检查 anticipatory ranker、低速候选和候选密度。没有一个纵向单因素同时
通过冻结门；全部阶段终止，不再扩预算。

### V3-07：空间候选与空间预测

V3-07 建立横向空间候选、observed-mode safe-overlap gate 和 candidate-conditioned spatial
predictor。B0-R1 冻结 0.496740409 m conformal radius；B1 fresh held-out 20/20，
global/per-head coverage 均为 100%。

### V3-08：空间 hard verifier 闭环

V3-08-A0 通过 520 个离线 fixtures。A1/A1-R1 暴露并修复非有限拒绝诊断，但 R1 为 17/18
目标成功的有效阴性结果。A3 只实施预先归因的时长与 receipt identity 修复，在 fresh
seeds 6691--6696 上完成 18/18，保留 certified-stop 与硬安全门。

### V3-09：公平资格与确认性评测

Q1 因双方重规划 commitment 不一致而停止；Q1-R1 只统一双方 0.25 s 重规划周期，在 fresh
qualification seeds 6721--6726 上通过。F1 使用独立 fresh seeds 6801--6830 完成
180/180 正式矩阵，点效应 22.3844%、单侧 lower 21.8235%、positive blocks 30/30，
正式阳性且独立重放通过。

### V3-10-P1：论文证据闭环

纯离线导出 summary/policy/block CSV、SVG 主图与 claims YAML，并用规范多行 SHA manifest
绑定历史证据。该阶段权限和预算均为 0，不新增实验结论。

### V3-11：actual TEB 公平资格与正式评测

V3-11 把主要基线升级为工作区实际构建的
`teb_local_planner/TebLocalPlannerROS` 0.8.4。双方共享 M2 Ackermann 车辆、执行器动力学、
`/cmd_vel_navigation` 命令边界、1.2 m/s 速度上限、起始前缀、handoff gate、目标、
simulator seed 和 Gazebo-oracle actor track。Q1-R1 使用 fresh seeds 7021--7032 完成
24/24 并通过 14 项资格门。F1 使用 fresh seeds 7101--7130 完成 30 paired blocks / 60
episodes，点效应 12.9288%、单侧 97.5% lower 12.7253%，六个 scene×policy strata
全为正，双方目标成功 100%、contact 0，正式阳性且独立重放通过。

### V3-12-P1：actual TEB 论文证据闭环

纯离线导出 summary/stratum/block CSV、SVG 主图与 claims YAML。paper evidence manifest
SHA 为 `319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`，
独立闭环 review SHA 为
`96d7d0333ef2eba5b62819fb0fadc689c3756e5015804d2ea8fb86b4a4ca17b2`。
该阶段权限和预算均为 0，不新增仿真或训练。

## 10. 证据与论文规则

- 每个阶段的阳性、阴性和完整性失败都必须保留；
- 不允许删除失败身份、替换 seed、事后降低门槛或成功后追加测试；
- 高频 frame 不是独立统计样本；统计单位是 geometry/actor-policy block；
- calibration/validation/test 按 family 隔离；
- test 只访问一次；
- 任何性能声明必须相对最强公平基线；
- `formal_result=true` 与 `positive_claim=true` 分离；
- RL 阴性不否定 predictor、safety verifier 或解析 planner 的独立结果。

## 11. V3-00 foundation 终止条件

只有以下全部满足，V3-00 才能关闭：

- contract 和 preregistration 严格解析；
- package/message namespace 无 V2 身份复用；
- source package map 唯一且 package resolution 设计闭合；
- candidate fixture 运动学、ID、stop/fallback 和 digest 门通过；
- actor/cloned fixture 的 schema、hidden-label firewall、propensity 与 response-effect gate 通过；
- dependency closure 无 unresolved/hash drift；
- builder 两次重建 byte-identical；
- independent reviewer 与 persisted report exact-equal；
- execution/training/seed/schedule/release/attempt/journal 全部不存在；
- 无 ROS/Gazebo/move_base/training 进程；
- 终态 SHA freeze 通过。

通过 V3-00 只允许设计 V3-01，不自动授权仿真。

## 12. V3 最终声明规则

最新允许的确认性陈述是：

> Within 30 fresh paired blocks from three frozen single-reactive-actor
> Gazebo templates and cautious/hesitant policies, V3 spatial-safe reduced
> restricted gate-to-goal time by 12.9288% relative to actual
> TebLocalPlannerROS 0.8.4; the one-sided 97.5% paired-bootstrap lower bound
> was 12.7253%. Both methods achieved 100% goal success and no contact state
> was observed.

必须同时保留：

- actual TEB 最小经验保守净空只有 0.0192 m，不得外推为无条件安全排名；
- 双方共享 Gazebo oracle actor track，不存在 perception robustness 证据；
- 只覆盖三个冻结 single-reactive-actor 模板和 cautious/hesitant policy；
- 训练步数为 0，不存在 learned-policy benefit；
- 结果不覆盖 multi-actor、分布外输入、真实传感器、独立复杂地图或硬件；
- 经验净空门不等于形式化安全证明；
- 不得把 `formal_result=true` 外推为 `runtime_ready=true`。

V3-09-F1 相对 fixed-v1.20 shielded comparator 的 22.3844% 历史阳性结果继续保留，
但不再作为回答 actual-TEB 基线问题的最新证据。
