# V3 候选方向文献包

下载日期：2026-07-24

本目录保存 V3 未来研究设计的本地只读参考文献。它不构成新的实验阶段、执行授权、
性能结论或部署资格。PDF 已通过文件类型检查，字节校验见
[`SHA256SUMS`](./SHA256SUMS)。

## 建议阅读顺序

1. 先读 04，理解为什么交互预测需要主动探索并观察他车对自车动作的反应。
2. 再读 01、02、03，比较 goal-conditioned、what-if 和博弈式联合预测/规划。
3. 读 08、09，明确预测安全过滤和概率可达集成立所需的模型、终端安全集与校准条件。
4. 最后读 10、06、07，决定问题是离线上下文 bandit 还是必须升级为时序离线 RL。
5. 05 用作“可学习模块 + 显式优化器”的结构化端到端参考，不把其结果直接外推到本项目。

## 文献与本项目用途

| 文件 | 论文 | 本项目用途 | 原始来源 |
|---|---|---|---|
| [`01_PRECOG_ICCV2019.pdf`](./01_PRECOG_ICCV2019.pdf) | PRECOG: PREdiction Conditioned on Goals in Visual Multi-Agent Settings | 条件多智能体预测；说明受控体目标可以进入预测条件 | [ICCV 2019](https://openaccess.thecvf.com/content_ICCV_2019/html/Rhinehart_PRECOG_PREdiction_Conditioned_on_Goals_in_Visual_Multi-Agent_Settings_ICCV_2019_paper.html) |
| [`02_What_If_Motion_Prediction_ECCV2020.pdf`](./02_What_If_Motion_Prediction_ECCV2020.pdf) | What-If Motion Prediction for Autonomous Driving | 可查询的反事实几何/社会上下文预测 | [arXiv 2008.10587](https://arxiv.org/abs/2008.10587) |
| [`03_GameFormer_ICCV2023.pdf`](./03_GameFormer_ICCV2023.pdf) | GameFormer | 交互预测与规划联合建模；区分 reactive 与 non-reactive 闭环评测 | [ICCV 2023](https://openaccess.thecvf.com/content/ICCV2023/html/Huang_GameFormer_Game-theoretic_Modeling_and_Learning_of_Transformer-based_Interactive_Prediction_and_ICCV_2023_paper.html) |
| [`04_Interaction_Aware_Prediction_Decision_2023.pdf`](./04_Interaction_Aware_Prediction_Decision_2023.pdf) | Learning Interaction-aware Motion Prediction Model for Decision-making in Autonomous Driving | 最直接的候选轨迹条件预测参考；其在线探索设计支持“当前日志不足以识别反应”的判断 | [arXiv 2302.03939](https://arxiv.org/abs/2302.03939) |
| [`05_DIPP_Integrated_Prediction_Planning.pdf`](./05_DIPP_Integrated_Prediction_Planning.pdf) | Differentiable Integrated Motion Prediction and Planning with Learnable Cost Function | 学习预测/代价与显式优化器组合的结构化端到端参考 | [arXiv 2207.10422](https://arxiv.org/abs/2207.10422) |
| [`06_CQL_NeurIPS2020.pdf`](./06_CQL_NeurIPS2020.pdf) | Conservative Q-Learning for Offline Reinforcement Learning | 离线 RL 分布偏移与保守价值估计 | [NeurIPS 2020](https://proceedings.neurips.cc/paper/2020/hash/0d2b2061826a5df3221116a5085a6052-Abstract.html) |
| [`07_IQL_ICLR2022.pdf`](./07_IQL_ICLR2022.pdf) | Offline Reinforcement Learning with Implicit Q-Learning | 避免显式评价数据集外动作的离线策略学习参考 | [ICLR 2022](https://openreview.net/forum?id=68n2s9ZJWF8) |
| [`08_Predictive_Safety_Filter.pdf`](./08_Predictive_Safety_Filter.pdf) | A Predictive Safety Filter for Learning-Based Control of Constrained Nonlinear Dynamical Systems | 用可行 backup trajectory、终端安全集和模型误差界过滤学习动作 | [Automatica / arXiv](https://arxiv.org/abs/1812.05506) |
| [`09_Multi_Agent_Reachability_Conformal.pdf`](./09_Multi_Agent_Reachability_Conformal.pdf) | Multi-Agent Reachability Calibration with Conformal Prediction | 把预测误差校准为动态可行占据管；明确概率保证与假设边界 | [arXiv 2304.00432](https://arxiv.org/abs/2304.00432) |
| [`10_PAC_Bayesian_Offline_Contextual_Bandits_ICML2023.pdf`](./10_PAC_Bayesian_Offline_Contextual_Bandits_ICML2023.pdf) | PAC-Bayesian Offline Contextual Bandits With Guarantees | 候选轨迹一次选择优先建模为离线上下文 bandit 的理论参考 | [ICML 2023](https://proceedings.mlr.press/v202/sakhi23a.html) |

## 使用边界

- 文献中的数据规模、车辆模型、安全保证和性能数字不能直接迁移为本项目结论。
- “条件预测”不自动等于因果反事实；必须另行满足干预覆盖、可交换性/无混杂或明确的结构假设。
- CQL/IQL 缓解数据集外动作估值，但不能从没有覆盖的数据中创造反事实证据。
- 共形或 RollingRC 校准给出的是相应假设下的边际/渐近误覆盖控制，不是无条件碰撞零风险。
- 所有本项目性能结论仍须来自独立预注册、fresh family/seed、强公平基线和一次性 held-out test。
