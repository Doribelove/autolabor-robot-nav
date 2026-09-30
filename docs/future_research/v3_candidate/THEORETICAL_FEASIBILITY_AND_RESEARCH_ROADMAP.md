# V3-SCOPE 理论可行性审查与未来研究路线

日期：2026-07-24  
状态：`concept_only_offline_review`  
执行授权：`false`  
训练授权：`false`  
实车授权：`false`  
历史阶段复用：`forbidden`

## 1. 结论先行

推荐的新主线是：

> **V3-SCOPE：安全反事实占据预测、可行候选轨迹选择与证据闭环**

它保留现有 M2 底盘、激光、GPS、里程计、相机和最终命令安全链，不再把“RL 调很多
TEB 参数”作为核心动作。系统先生成一组已经满足 Ackermann 运动学的候选轨迹，再对每条
候选分别预测动态参与者可能的反应和不确定性，由硬安全过滤器删除不可接受候选，最后才由
解析代价、离线上下文 bandit 或有条件启用的离线 RL 选择候选。

理论审查的结论不是“现在即可训练”：

| 子命题 | 结论 |
|---|---|
| Ackermann 可行轨迹集 + 硬安全过滤 | **可行，可立即作为 V3 首个算法主线** |
| 小型向量化场景编码与候选条件预测 | **计算上可行** |
| 从当前 Gazebo/实车日志学习真正反事实反应 | **当前不可识别，必须先补交互干预数据** |
| 离线上下文 bandit 选择候选 | **在记录 propensity 且候选覆盖充分后可行** |
| 直接上 CQL/IQL 时序离线 RL | **条件可行，不应作为第一阶段** |
| 概率校准直接等价于安全保证 | **不成立，必须与可达性/制动 backup 结合** |
| 硬件完全不变 | **可行；软件输出仍进入现有 `/cmd_vel_navigation` 安全链** |
| 保证最终一定超过强基线 10% | **不能诚实保证，但可用前置止损门显著提高成功概率** |

最重要的改写是：主论文先证明“交互场景中的安全效率与可解释性”，RL 只是候选排序器的
可选增强。即使 RL 没有额外收益，预测校准、硬安全候选规划、反事实数据集和难例闭环仍分别
构成可验证结果，避免把整项研究押在一次不稳定的训练上。

## 2. 本地证据对设计的约束

### 2.1 FAM-TEB 历史证据

- P1 正式试验完成 90 paired blocks / 270 episodes，安全、任务、完整性与机制门通过。
- restricted completion time 点估计降低 `3.7624%`，单侧 95% lower 为 `2.7904%`，
  未达到明显提升门。
- T12 后续诊断中训练 action projection 平均 `67.3%`。Ackermann 参数耦合和
  warning 后回 Anchor 的变化率限制，是 commanded action 与 executed action 不一致的两个
  主要结构来源。
- P2-D1 在 30 个 semantic-clear blocks 上得到保留真实前缀的同路径纵向包络：
  aggregate point `12.0144%`、单侧 95% lower `11.2649%`。这只证明 clear 子集存在物理
  回收空间，不证明某个参数或新系统能实现它。
- post-P1 搜索已经终态关闭，不能复用其 seed、budget、release 或 stage identity。

因此，V3 应直接学习“最终会执行哪条轨迹”，而不是先学参数、再经过解码、投影、事务、TEB
优化和底盘约束后才间接影响运动。

### 2.2 当前仿真的反事实缺口

当前 `v2_trajectory_actor_plugin.cpp` 按仿真经过时间在预注册 waypoint 间插值，并直接
`SetWorldPose`；插件不读取自车状态或自车候选轨迹。它是确定性、开环、非反应式 actor。

令 `Y_i(s)` 表示相同初始场景 `s` 下，其他参与者面对自车候选轨迹 `τ_i` 的未来。当前 actor
近似满足：

```text
Y_i(s) = Y_j(s),  对任意 i、j
```

所以现有场景可以验证 TTC、跟踪、风险分类和执行完整性，却不能提供“他人会如何回应不同自车
动作”的训练信号。直接在这些数据上训练候选条件预测器，最多学到时空相关性，不能证明交互
反事实能力。

### 2.3 稳定实车工作区可复用的安全壳

`/home/robot/robot_ws` 的 2026-07-23 交接显示：

- 已有 15 m rolling GPS subgoal manager；
- 已有 TEB Ackermann 曲率保持、正反向语义修复和终点 stop latch；
- 已有 `/cmd_vel_navigation -> /cmd_vel_gps -> /fod_navigation_mode -> /cmd_vel`
  的唯一命令所有权链；
- GPS/FOD 切换会先屏蔽 GPS、取消子目标、确认停车，再转移控制权；
- 已有完整 Qt/RViz/GPS/FOD/视觉入口，73 包可构建，相关测试 196 项通过；
- 集成 GPS/FOD 实车低速验证仍未完成，不能把自动化通过写成实车资格。

稳定工作区没有 `nav_world_model` 动态轨迹包；当前 FOD tracker 明确面向 stationary FOD，
不估计速度，且米制地面投影仍依赖尚未验证的相机外参。因此它不能直接充当行人/车辆交互
预测输入。

### 2.4 当前计算资源

只读检查得到：

- NVIDIA GeForce RTX 4060 Laptop GPU，8 GB；
- Intel i7-14650HX；
- 15 GiB RAM。

这足以支持 track/path 向量输入的小型 GNN/Transformer、8--12 条候选的共享编码推理和离线
训练；不适合把大型视频生成世界模型或原始多相机端到端模型作为本项目必需组件。最终部署可
导出 ONNX/TensorRT 或 C++ 小模型，并保留解析降级路径，无需更换底盘或传感器。

## 3. V3-SCOPE 架构

```text
现有 /scan、/odom 或 /gps/odom、TF、global path、costmap、health
                              |
                              v
             scene_graph_world_model（静态几何 + track history）
                              |
                              v
        Ackermann candidate generator（8--12 条 + certified stop + TEB）
                              |
                              v
      candidate-conditioned predictor p(other future | scene, ego candidate)
                              |
                              v
       calibration + reachable occupancy tubes + distribution-shift monitor
                              |
                              v
      hard safety verifier（净空、制动、曲率、转角率、时延、接口健康）
                              |
                              v
          safe candidate set -> analytic ranker / bandit / gated offline RL
                              |
                              v
              trajectory tracker -> /cmd_vel_navigation
                              |
                              v
  现有 gps_goal_speed_limiter -> FOD mode arbiter -> m2_driver sanitizer
```

TEB 不再是被 RL 高频修改的黑盒。`TEB-Tuned-V1` 和独立公平调优固定 TEB 继续作为强基线；
当前 TEB 局部轨迹也可以作为候选集的一条 fallback。V3 不直接发布 `/cmd_vel`，不绕过现有
终点继电器、FOD 仲裁、急停和 M2 驱动边界。

### 3.1 动作域

每个动作是一个离散候选索引及其已冻结轨迹：

```text
action = candidate_id
candidate = {x_k, y_k, yaw_k, v_k, steering_k, time_k}_{k=0..H}
```

候选生成器先满足运动学自行车模型和边界：

```text
x(k+1)   = x(k) + v(k) cos(yaw(k)) dt
y(k+1)   = y(k) + v(k) sin(yaw(k)) dt
yaw(k+1) = yaw(k) + v(k) tan(steering(k)) / wheelbase * dt
```

并约束速度、加速度、转角、转角率、曲率和规划最小半径。学习器无法输出不可表达的参数组合，
因此旧系统的高 projection 问题被改写为可解释的“候选被安全过滤器拒绝”。

### 3.2 候选条件交互预测

共享场景编码器只运行一次；轻量 decoder 并行接收每条 `τ_i`，输出其他参与者的多模态轨迹和
不确定性：

```text
p_phi(Y | history, map/geometry, tau_i)
```

候选之间共享输入表示，避免为每条轨迹重新运行完整网络。第一版只用 track/path 向量，不用
原始图像。建议先做 2 层、`d_model=128` 的小模型；只有在 p99 延迟和内存门通过后才扩大。

### 3.3 安全过滤

正常制动候选至少满足：

```text
d_free >= v * Delta_total + v^2 / (2 * a_brake_min) + margin
```

`Delta_total` 必须包含感知延迟、一次规划周期、命令延迟、执行器响应和接口 timeout。其他
参与者的预测均值不能单独作为障碍；应由校准误差和动力学可达性形成 time-indexed occupancy
tube，再与自车 footprint/跟踪误差 tube 做分离检查。

安全结论只能在以下条件下成立：

1. 初始状态位于可行安全域；
2. 制动/跟踪模型误差有有效上界；
3. 至少一条 backup/stop trajectory 可递归执行到终端安全集；
4. 感知、定位、TF 和命令链健康；
5. 预测误覆盖保证所依赖的数据条件成立。

如果车辆已经以过高速度进入不足的净空，任何规划器都无法倒推出绝对安全；此时只能最小化
后果。报告中不得把概率占据管写成无条件形式化安全。

## 4. 五项理论审查

### 4.1 可观测性：部分满足

现有激光、里程计、GPS、全局路径和 costmap 足以形成结构化场景状态；V2 已有 simulation-only
的激光聚类、alpha-beta tracking 和 constant-velocity prediction 候选实现。缺口是稳定
实车工作区尚无经过实车标定的动态参与者 tracker。

结论：仿真研究可行；实车部署需要同硬件下的软件跟踪标定，不需要新增传感器，但不能跳过。

### 4.2 反事实可识别性：当前不满足

普通日志每个状态只观察被执行动作的结果 `Y_A(s)`，未选择候选的 `Y_i(s)` 是缺失潜在结果。
若要把条件预测解释成近似 `p(Y | do(tau_i), s)`，至少需要：

- safe candidate 的非零干预概率，即 positivity/coverage；
- 相同初态的 cloned rollout 或可接受的无混杂假设；
- actor 确实会响应 ego；
- action propensity、actor hidden type 和随机种子可审计。

当前开环 waypoint actor 不满足第三项。因此 V3 第一项工作不是训练网络，而是建立反应式 actor
和 cloned counterfactual rollout。建议包含 `cautious / assertive / hesitant` 三类隐藏行为，
在 crossing、head-on、merge、overtake/yield、corridor-crossing 和 occlusion-emergence 等
family 中对安全候选进行随机干预。

### 4.3 可学习性：候选排序优于参数调节

候选动作与执行轨迹直接对齐，信用分配链显著缩短。若每 100 ms 重规划且主要收益在当前窗口
即可观察，问题首先是 contextual ranking/bandit，而不是完整 MDP。

只有出现以下证据才升级为时序 RL：

- “现在让行/抢行”对多个后续窗口产生显著延迟收益；
- 相同即时风险和进度下，长期 return 标签能稳定区分候选；
- analytic ranker 或 contextual bandit 无法表达该差异；
- 数据中对应 action support 足够。

CQL/IQL 可以缓解分布偏移，但不能弥补从未执行过的候选。旧 T12/P1 数据的 action space 是
TEB 参数/语义 profile，不能直接作为 V3 candidate policy 的离线 RL 数据，只能用于系统辨识、
场景复现和基线。

### 4.4 安全性：条件可证，不可无条件保证

预测安全过滤论文要求已知/有界的动力学误差、初始可行、终端安全集和可行 backup trajectory。
多智能体共形可达方法提供的是假设下的误覆盖控制；标准 split conformal 依赖同分布与
exchangeability，RollingRC 等顺序方法可以放松这些条件，但保证转为相应的渐近误覆盖控制，
多智能体组合还会引入参与者条件独立或 union-bound 的保守性。

因此 V3 的安全论证应分两层：

- 硬确定层：自车运动学、制动包络、footprint、转角率、命令新鲜度、stop backup；
- 概率层：他车预测误差管、覆盖率和 OOD/shift 监测。

一旦 shift monitor 触发、tube 过宽或全部候选被拒绝，必须退化为 stop/现有安全 fallback。

### 4.5 实时性：可行但必须实测

现有控制频率约 10 Hz，一个周期为 100 ms。冻结目标应为：

- planner + predictor p99 `< 40 ms`；
- 整个感知到 `/cmd_vel_navigation` p99 `< 80 ms`；
- timeout、NaN、超预算全部 fail closed；
- 先固定 8 条候选，只有延迟有余量才升到 12 条。

采用“一次 scene encoder + K 个轻量 decoder”、两阶段候选粗筛和解析 backup，在当前
RTX 4060/i7 上有合理可行性。此处只是复杂度判断，不是已测性能结论。

## 5. 为提高出好结果概率而设计的阶段

### V3-00：Hermetic 证据与执行底座

目标：先消除 P2/P3/P4 暴露的 package resolution、schema/lifecycle 和环境绑定问题。

冻结并审查：

- workspace overlay、`ROS_PACKAGE_PATH`、package lookup 与 launcher 解析；
- exact scene reset、cloned initial state 和随机源；
- 每个候选、propensity、过滤原因、预测、执行和 outcome 的 raw journal；
- container/environment fingerprint、路径 + SHA、failure index；
- 运行前 package-resolution probe，失败不得越过消费边界。

通过门：多次独立 dry-run/preflight 零环境解析失败，clone 初态误差在预注册阈值内。未通过则
不建立算法训练预算。

### V3-01：反应式场景与反事实数据资格

每个初态对 8 个安全候选做 cloned intervention，actor 参数对 policy 不可见，只供 evaluator
分层。训练/validation/test 必须按 geometry family 与 actor policy family 隔离，不再把同一模板
换随机 seed 当成几何泛化。

资格门至少包括：

- 不同 `τ_i` 导致的 actor response 大于仿真/跟踪噪声地板；
- 交互状态中存在足够比例的非零 response effect；
- 每类候选有可审计的非零 propensity；
- clone determinism、碰撞 contact 与 evaluator replay 通过；
- runtime policy 不读取 actor type、scene role 或 future truth。

不通过：停止“反事实交互”主张，转为非交互的 uncertainty-calibrated safe planner。

### V3-02：候选条件预测与校准

比较：

1. constant velocity；
2. 非候选条件的小型 GNN/Transformer；
3. candidate-conditioned predictor；
4. 可选 GameFormer 风格层级解码器。

主指标不是只看 ADE/FDE，还要看：

- response direction / yield-assert classification；
- collision-relevant min distance error；
- NLL/Brier；
- 95% occupancy coverage 与 tube area；
- held-out actor-policy-family 性能；
- OOD 检出率与误报率；
- p99 推理延迟。

只有条件模型在 held-out family 上同时改善反应预测和校准效率，才进入 planner。

### V3-03：无学习安全候选规划器

先实现解析代价：

```text
cost = progress/time + path deviation + jerk
     + calibrated occupancy risk + switch hysteresis
```

在这个阶段就与最强公平固定 TEB 比较。若解析候选规划器已经通过目标门，先冻结阳性结果；
RL 作为后续独立问题，不能为了“必须有 RL”而重新打开已经通过的 test。

### V3-04：有条件启用 bandit / 离线 RL

优先顺序：

1. supervised pairwise ranker；
2. offline contextual bandit；
3. IQL/CQL 离线时序策略。

升级到下一层前必须证明上一层存在无法解释的长期 regret。离线学习资格门建议包含：

- behavior propensity 完整；
- 重要性权重上界和有效样本量通过预注册门；
- policy 选择保持在 safe behavior support；
- validation 上相对解析 ranker 有稳定、跨 seed 的额外收益；
- safety filter 正常干预率 `< 5%`，不得靠频繁否决学习器维持安全。

若 RL validation 增益不足或跨 seed 不稳定，保留阴性结果并停止 RL；不影响 V3-02/03 的独立
论文结论。

### V3-05：一次性正式评测

主 claim population 应在数据生成前冻结为“需要交互决策的动态场景 family”，而不是把大量
无遮挡直线巡航稀释进来。clear/cruise 作为安全与效率非劣检查。这样是机制对齐，不是事后
挑场景。

公平基线：

- `TEB-Tuned-V1`；
- calibration-only 独立公平调优的最强固定 TEB；
- 无学习解析候选规划器；
- 非条件预测消融；
- 若进入 V3-04，再加入 supervised/bandit/RL 消融。

正式阳性门建议保持：

- 相对最强公平基线的受限交互完成/延迟时间点估计至少降低 `10%`；
- 单侧 95% lower 至少降低 `5%`；
- success、collision、emergency 非劣；
- clearance lower 不低于 `-0.05 m`；
- safety filter 正常干预 `< 5%`；
- 无未解释 crash、完整性失败、seed 替换或 test 泄漏；
- planner p99 `< 40 ms`，端到端 p99 `< 80 ms`。

统计单位应是独立 geometry/actor-policy block，方法在同一初态 paired；置信区间按 family
cluster，而不是把 10 Hz 帧或同模板 seed 当成独立样本。历史 P2-D1 的 30-block 包络只用于
判断信号量级；新程序仍须用 V3 pilot 方差冻结正式样本量。

## 6. “好结果”的抗失败设计

不能在保持科研诚信的同时保证阳性，但可以让项目不再只有一个易失败出口：

| 结果层 | 即使下一层失败仍可保留的贡献 |
|---|---|
| V3-00 | 可复现、可重放、环境闭包的 Gazebo 评测工具链 |
| V3-01 | 反应式 actor + cloned counterfactual navigation benchmark |
| V3-02 | 候选条件预测、校准占据管与 OOD 审计 |
| V3-03 | 硬安全、可解释、实时的 Ackermann 候选规划器 |
| V3-04 | RL 是否在强解析 ranker 上产生额外长期收益的诚实结论 |
| V3-05 | 强公平基线、family-held-out 的正式性能证据 |

提高阳性概率的真正手段是：

1. 先做 response identifiability 和物理 headroom gate；
2. 把动作直接对齐到执行轨迹；
3. 主攻已知存在机制空间的 false-yield、stop-go 和交互延迟；
4. 用强基线和 paired clone 降低方差；
5. 在 test 前用 calibration 冻结模型、阈值和样本量；
6. RL 只解决已被证明存在的长期决策缺口。

不是通过弱化 TEB、筛 seed 或降低门槛制造结果。

## 7. 明确放弃的方向

- **原始传感器到 `/cmd_vel` 的大模型端到端**：数据、算力、解释性和安全边界均不匹配。
- **大型 diffusion/video world model**：当前 8 GB GPU 和 ROS 10 Hz 目标下不适合作为必需链路。
- **继续以 20 维 TEB 参数为 RL 主动作**：旧证据已显示高 projection 和长信用链。
- **在当前 waypoint actor 上声称反事实交互**：actor 不响应 ego，因果效应不可识别。
- **把 stationary FOD tracker 当作动态参与者预测器**：消息和算法没有速度状态，外参也未完成。
- **RL 直接控制底盘**：绕过已有安全壳，不符合项目和岗位的产品化要求。
- **故意做弱 TEB 基线**：不能形成可信研究或工程结论。

## 8. 与岗位要求的对应

| 岗位能力 | V3 可展示的具体产物 |
|---|---|
| 轨迹预测 | 候选条件多模态预测、校准占据管、held-out family 评测 |
| 行为决策/路径规划 | 可行轨迹集、交互候选排序、解析解与学习方法对照 |
| 运动控制/量产部署 | Ackermann/MPC 跟踪、ONNX/C++、p99 延迟和现有命令安全链接入 |
| 仿真引擎 | 反应式 actor、cloned intervention、确定性 reset 与场景 DSL |
| 云端批量仿真 | hermetic release、无 GUI worker、seed/schedule/failure index |
| 场景增广 | geometry × timing × actor-policy 因子化生成 |
| 数据闭环 | disagreement、tube width、safety intervention、counterfactual regret 难例挖掘 |
| 打开黑盒 | 候选全量日志、拒绝原因、风险分解、解析 ranker 与学习增量消融 |

## 9. 最终建议

下一项真正值得立项的不是“FAM-TEB 再调一次”，而是一个全新的、独立预注册的 V3 科学问题：

> 在不改变 M2 硬件和最终安全命令链的前提下，候选轨迹条件交互预测与校准安全过滤，能否在
> family-held-out 动态交互场景中，相对最强公平固定 TEB 显著降低不必要让行和交互完成时间；
> 在此基础上，离线上下文 bandit 或离线 RL 是否还能产生可归因的额外长期收益？

先通过 V3-00/01 的工具与可识别性门，再决定是否训练。这样最符合“性能底线优先、数据闭环、
仿真工具、解释性和最终可部署”的岗位要求，也是在当前硬件与证据下成功概率最高的路线。
