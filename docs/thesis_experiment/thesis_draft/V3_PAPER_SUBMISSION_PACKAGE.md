# V3-SCOPE 论文投稿准备包

日期：2026-08-06

## 建议题目

中文：

> 面向 Ackermann 移动机器人的候选条件交互预测与安全空间轨迹选择

英文：

> Candidate-Conditioned Interaction Prediction and Safe Spatial Trajectory
> Selection for Ackermann Mobile Robots

题目可以在副标题或摘要中写明 “comparison with TebLocalPlannerROS”，但不使用
“general TEB superiority”“real-world safety” 或 “reinforcement learning”，因为当前
actual-TEB 证据只覆盖三个冻结 single-reactive-actor Gazebo 模板，没有实车验证，且
V3-04 按支持度门关闭了训练。

## 摘要草稿

动态横穿场景中的局部导航需要同时处理车辆运动学、他者对自车动作的条件响应以及安全备份。
本文提出 V3-SCOPE，将决策动作从高维局部规划器参数改为预生成的 Ackermann 可行空间轨迹，
并在解析排序前依次执行候选条件交互预测、校准可达占据和硬安全验证。系统保留 certified
stop，禁止评估器隐藏 actor 类型进入运行时，并对每次候选接受、拒绝和执行建立可重放证据。
离线支持度审计未发现启用 contextual bandit 或 offline RL 的充分条件，因此正式方法保持为
零训练解析规划器。我们进一步建立工作区实际构建的
`TebLocalPlannerROS` 0.8.4 公平基线，使两方法共享 M2 Ackermann 车辆、执行器动力学、
速度上限、起始 gate、目标、命令边界、paired simulator seed 和 oracle actor track。在
三个冻结 single-reactive-actor Gazebo 模板、cautious/hesitant 两类行为的 30 个 fresh
paired blocks、60 个一次性 episodes 中，两方法均达到 100% 目标成功且 contact 为 0。
V3-SCOPE 将平均受限 gate-to-goal 时间由 7.7733 s 降至 6.7683 s，配对均值下降
12.9288%，单侧 97.5% paired-bootstrap lower 为 12.7253%；六个 scene×policy strata
方向全部为正。结果证明限定反应式横穿仿真中相对 actual TEB 的空间候选选择收益。进一步
的双 actor 扩展完成 5 单元预检和 40 单元 fresh 证据矩阵，45/45 动作身份绑定均通过，但
跨候选共同前缀 actor 坐标/速度差达到 0.06607 m/0.07208 m/s，超过预注册的
0.05 m/0.05 m/s 门。后续 fresh 单因素实验只将 actor 响应延迟到 12.55 s，完成新的
45/45 单元后把速度差降至 0.008597 m/s（下降 88.07%），但坐标差仍为 0.067472 m，
仅 5/8 区块通过坐标门。因此多人预测器与后续评测继续关闭。本文不构成感知鲁棒、
multi-actor/OOD 泛化、形式化安全、学习收益、部署或实车性能证据。

## 可写成贡献的三点

1. 一种 action-aligned 的局部规划架构：策略选择已满足 Ackermann 约束的轨迹，而不是输出
   会被大量投影的高维 TEB 参数。
2. 一条候选条件预测—校准占据—硬安全验证链，并以 evaluator-only hidden label firewall、
   certified stop 和逐决策 digest 保证可审计性。
3. 一项相对 actual `TebLocalPlannerROS` 的预注册 fresh paired-block 正式评测：60/60
   一次完成，点效应 12.9288%、单侧 97.5% lower 12.7253%，六个 scene×policy strata
   全为正，并完整公开公平 gate、净空和分析修复审计。
4. 一项 fresh 单因素反证实验：响应门控使共同前缀速度错位下降 88.07%，但坐标门仍失败，
   将多人扩展的复合失败分解为响应动力学与位置同步两个层次。

## 核心图表

- 主结果图：
  `artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_formal_result.svg`
- 总指标：
  `artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_formal_summary.csv`
- scene×actor-policy 分层：
  `artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_stratum_results.csv`
- 30-block 配对结果：
  `artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_block_results.csv`
- 声明边界：
  `artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_claims.yaml`

## 审稿时必须主动说明

- comparator 是实际构建的 TebLocalPlannerROS 0.8.4，但不是对所有 TEB 版本/配置的排名；
- 确认性总体只有三个冻结 single-reactive-actor crossing 模板；
- actor policy 是仿真行为模型，不是实测人群分布；
- 双方共享 Gazebo oracle actor track，因此没有 perception robustness 证据；
- actual TEB 的最小经验保守净空为 0.0192 m，零 contact 不等于通用安全；
- 训练步数为 0，论文贡献不是 RL；
- 保守净空无违规是经验仿真结果，不是形式化安全证明；
- 没有真实车辆、多人、独立复杂地图、OOD 或传感器退化的确认性结果；
- V2 和 V3-05 的阴性结果继续保留，不因 V3-09-F1 阳性而被覆盖。
- V3-14-S1 是完整阴性扩展，不得因 45/45 执行成功而写成 multi-actor 资格通过；失败的是
  反事实共同前缀 actor 配对门，而不是动作绑定或进程完整性。
- V3-16-S1 的速度机制改善必须与坐标门 0.067472 m 失败同时出现；不得写成多人支持、
  predictor 资格或性能通过。

## 投稿成熟度判断

当前材料已具备可复核算法原型、actual-TEB 公平基线、fresh confirmatory matrix、
预注册统计、负向历史、六分层结果、multi-actor 阴性边界、完整性审计和图表导出，足以形成一篇范围明确的
仿真方法论文或会议论文。若期刊接受以高保真仿真为主证据，当前结果可作为核心实验；
若要求更强外部效度，应补充下列独立研究。

若目标是更强的机器人期刊/主会完整论文，提交前建议另立新研究完成：

1. DWA/MPPI 等额外 paired baselines；
2. 独立静态几何、多 actor 与非横穿交互；
3. 定位/感知噪声与 distribution-shift 压力测试；
4. 独立硬件标定后的低速封闭场现场验证。

这些属于新的证据阶段，不应通过扩充或重跑 V3-09-F1/V3-11-F1 完成。当前阳性结果不需要
训练；只有在新 cloned-intervention 数据对替代 candidate 具有非零可审计支持、且新的
learning-value gate 通过后，才应考虑 contextual bandit 或 offline RL。
