# V3-SCOPE 未来研究基础交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
状态：`concept_only_offline_foundation_handoff_complete`

## 1. 下一会话一句话入口

FAM-TEB V2/P1--P4 性能搜索已经以“未证明明显提升”终止。下一项研究不得续跑历史阶段，
而应建立全新的 V3-SCOPE：

> **安全反事实占据预测、可行候选轨迹选择与证据闭环。**

V3 保持 Autolabor M2 底盘、激光、GPS、里程计、相机和最终命令安全链不变；学习动作从
“修改大量 TEB 参数”改为“在已经满足 Ackermann 约束的候选轨迹中选择”。RL 是通过数据
资格门后的可选排序器，不是 V3 第一阶段，也不是主结论成立的必要条件。

当前权限固定为：

```text
concept_only=true
formal_result=false
runtime_ready=false
execution_ready=false
execution_authorized=false
training_authorized=false
evidence_budget_authorized=0
real_vehicle_authorized=false
```

本交接不创建 preregistration、release、seed、schedule、attempt、journal 或预算，也不授权
ROS、Gazebo、训练、动态参数写入或实车。

## 2. 下一会话阅读顺序

从新会话进入本项目后，按以下顺序恢复上下文：

1. 根目录 `AGENTS.md`；
2. `docs/thesis_experiment/CURRENT_V2_04G_P4_F1_TERMINAL_HANDOFF.md`；
3. 本文件；
4. `docs/future_research/v3_candidate/THEORETICAL_FEASIBILITY_AND_RESEARCH_ROADMAP.md`；
5. `docs/future_research/v3_candidate/literature/README.md`；
6. 若需要借鉴实车产品架构，只读
   `/home/robot/robot_ws/CURRENT_GPS_DEV_HANDOFF.md`，不得修改稳定工作区。

然后只读检查：

```bash
cd /home/robot/robot_ws_base_rl
git status --short --branch
git diff --check
ps -eo pid,ppid,stat,etime,cmd | rg \
  'roscore|rosmaster|roslaunch|gzserver|gzclient|move_base|sac|train' || true
sha256sum -c \
  docs/future_research/v3_candidate/V3_CONCEPT_FREEZE.sha256
```

不得为了让测试或工作树变干净而删除历史 artifacts、失败样本或用户 dirty changes。

## 3. 为什么需要 V3

### 3.1 历史性能结论

- P1 正式矩阵为 90 paired blocks / 270 episodes；
- integrity、安全/任务非劣和机制门通过；
- restricted completion time 点估计降低 `3.7624%`；
- 单侧 95% lower 为 `2.7904%`；
- `performance_improvement_demonstrated=false`。

post-P1 P2/P3/P4 的 2200-unit 容量已经以 4 consumed + 2196 retired 关闭，训练 0 steps，
没有新 held-out test。历史 release、seed、budget 和 stage identity 不得复用。

### 3.2 原动作域的结构问题

T12 离线诊断得到 action projection 平均 `67.3%`。Ackermann 参数耦合、warning 后回 Anchor
的变化率限制，以及“策略参数 -> 解码/投影 -> 参数事务 -> TEB 优化 -> 底盘执行”的长信用链，
使策略学习的 commanded action 与 executed behavior 严重错位。

V3 直接输出候选轨迹索引。候选在进入策略前已经满足运动学和接口约束；安全层的否决作为明确
事件记录，而不是把学习动作静默投影成另一个参数向量。

### 3.3 已知物理空间

P2-D1 对 30 个冻结 semantic-clear traces 的纯离线分析得到：

- same-path post-cruise envelope 相对 legacy 的 aggregate point：`12.0144%`；
- paired bootstrap 单侧 95% lower：`11.2649%`。

这只说明 clear 子集存在达到 10% 量级的物理空间，不证明 V3、某个速度或某个模型已经获得该
收益。V3 正式 claim population 应以需要交互决策的动态 family 为主，clear/cruise 用于非劣
检查。

## 4. 关键理论结论

| 项目 | 当前判断 | 进入下一步的必要条件 |
|---|---|---|
| Ackermann 可行候选轨迹 | 可行 | 冻结车辆模型、曲率、制动和时延边界 |
| 候选条件动态预测 | 计算上可行 | track/path 向量输入；p99 延迟通过 |
| 真正反事实反应 | 当前不可识别 | 反应式 actor + cloned intervention + action coverage |
| contextual bandit | 条件可行 | propensity、support、有效样本量通过 |
| CQL/IQL 时序离线 RL | 后置可选 | 先证明存在 bandit/解析排序无法解释的长期 regret |
| 概率预测安全 | 条件成立 | 校准误差管 + 可达性 + backup trajectory |
| 无条件安全保证 | 不成立 | 不得作此表述 |
| 保持硬件不变 | 可行 | 软件 tracker、模型和延迟需同硬件标定 |

当前 `v2_trajectory_actor_plugin.cpp` 只按仿真时间插值 waypoint 并直接设置 actor pose，
不读取自车状态或候选轨迹。它是开环、非反应式 actor。现有数据能验证 TTC、跟踪和执行
完整性，但不能训练或证明“其他参与者会如何响应不同自车动作”。

## 5. V3-SCOPE 冻结概念架构

```text
/scan + /odom 或 /gps/odom + TF + global path + costmap + health
                              |
                              v
        scene_graph_world_model（静态几何 + dynamic track history）
                              |
                              v
 Ackermann candidate generator（8--12 条 + certified stop + TEB fallback）
                              |
                              v
 candidate-conditioned predictor p(other future | scene, ego candidate)
                              |
                              v
 calibrated reachable occupancy tubes + distribution-shift monitor
                              |
                              v
 hard safety verifier（净空、制动、曲率、转角率、时延、接口健康）
                              |
                              v
 safe candidate set -> analytic ranker -> bandit -> gated offline RL
                              |
                              v
 trajectory tracker -> /cmd_vel_navigation
                              |
                              v
现有 speed limiter -> /cmd_vel_gps -> FOD mode arbiter -> /cmd_vel -> M2
```

硬件与稳定工作区的对应关系：

- 保留 rolling GPS subgoal manager 和全局路线；
- 新规划器只替换/旁路 local decision，不绕过 `/cmd_vel_navigation`；
- 保留 GPS goal speed limiter、终点 stop latch、FOD 控制权仲裁和 M2 sanitizer；
- 保留 `TEB-Tuned-V1` 和独立公平调优固定 TEB 为强基线；
- TEB 轨迹可以作为候选之一或故障 fallback；
- 不直接发布底盘 `/cmd_vel`；
- 不需要新增传感器，但动态 tracker 必须在现有激光上重新标定。

## 6. 推荐执行顺序

### V3-00：Hermetic 工具和证据底座

先建立新的 V3 namespace、系统指南、纯离线 foundation contract 和 independent review。
冻结 workspace overlay、`ROS_PACKAGE_PATH`、package lookup、environment fingerprint、场景
reset、cloned initial state、日志 schema 和 consumption boundary。

本阶段只能纯离线；不创建仿真 release/budget，不启动 ROS/Gazebo。

### V3-01：反应式 actor 与反事实数据资格

建立 `cautious / assertive / hesitant` 等隐藏 actor policy，在 crossing、head-on、merge、
overtake/yield、corridor-crossing、occlusion-emergence 等 family 中，对相同初态的多条安全
自车候选做 cloned intervention。

先证明：

- actor response 随自车候选发生可辨识变化；
- response effect 大于仿真/跟踪噪声地板；
- 各候选具有非零、可审计 propensity；
- runtime policy 不读取 actor type、scene role 或 future truth；
- family split 而不是同模板换 seed。

失败则停止反事实主张，转为 uncertainty-calibrated non-interactive safe planner。

### V3-02：候选条件预测与校准

比较 constant velocity、非条件小模型、candidate-conditioned predictor 和可选层级交互解码器。
同时评价反应方向、collision-relevant error、NLL/Brier、95% occupancy coverage、tube area、
held-out actor family、OOD 和 p99 延迟。

### V3-03：无学习安全候选规划器

先用解析代价选择轨迹：

```text
progress/time + path deviation + jerk
+ calibrated occupancy risk + switch hysteresis
```

在这一阶段就与最强公平固定 TEB 比较。若解析规划器已经达到正式门，先冻结该结果，不得为了
强行加入 RL 而重开 test。

### V3-04：有条件启用学习排序

固定顺序：

1. supervised pairwise ranker；
2. offline contextual bandit；
3. 仅在长期价值缺口被证明后使用 IQL/CQL。

RL validation 若增益不足、跨 seed 不稳定、action support 不足或安全过滤器正常干预过高，
保留阴性结果并停止 RL，不影响 V3-02/03 的独立贡献。

### V3-05：一次性正式 held-out 评测

按 geometry/actor-policy family 隔离，方法在相同初态 paired。相对最强公平基线保持：

- point improvement 至少 `10%`；
- 单侧 95% lower 至少 `5%`；
- success/collision/emergency 非劣；
- clearance lower 不低于 `-0.05 m`；
- 正常 safety-filter intervention `< 5%`；
- planner p99 `< 40 ms`，端到端 p99 `< 80 ms`；
- 无未解释 crash、完整性失败、seed 替换或 test 泄漏。

任何数值都要在新的 V3 preregistration 中重新冻结；本交接中的建议不是执行合同。

## 7. 下一项具体工作

下一安全入口仅为：

> **V3-00 纯离线 foundation design closure。**

预期产物：

1. `docs/thesis_experiment/V3_SYSTEM_GUIDE.md`；
2. `config/thesis_experiments/v3/v3_00_foundation_contract.yaml`；
3. V3 package/artifact namespace 与接口草图；
4. hermetic environment/package-resolution closure 设计；
5. reactive actor 与 cloned rollout 的数据合同；
6. independent offline reviewer 与静态 fixtures；
7. 明确保持 `execution_authorized=false`、budget=0。

V3-00 完成前不得训练模型、建立 held-out test、启动 Gazebo、分配 fresh execution seeds 或修改
稳定实车工作区。

## 8. 已保存材料

完整理论报告：

```text
docs/future_research/v3_candidate/
  THEORETICAL_FEASIBILITY_AND_RESEARCH_ROADMAP.md
SHA-256 4ac25509d8fc07296de27ad9630bb64a0fb71b12fcfcab0784b69a21d169e74f
```

文献索引与校验：

```text
docs/future_research/v3_candidate/literature/README.md
SHA-256 c7fe27ef4be43f18c8400d78ff5902ba8230e4914217fc6f0d4ec603142aead6

docs/future_research/v3_candidate/literature/SHA256SUMS
SHA-256 ba22a9fe6b6acaaf94d675d0f63a49eb33e123c5807da57d8270116926be335c
```

文献包共 10 篇、约 17 MB，覆盖条件/反事实交互预测、联合预测规划、CQL/IQL、预测安全过滤、
多智能体共形可达和离线上下文 bandit。

## 9. 保存时的运行边界

- 未启动 ROS、Gazebo、move_base 或训练；
- 未连接实车、CAN 或在线 TEB 参数；
- `/home/robot/robot_ws` 仅只读参考，未修改；
- P1/I5/T12/P2--P4 历史证据、失败样本和 dirty worktree 均保留；
- V3 当前只有概念报告、文献包和交接，不存在阳性结果或执行权限。
