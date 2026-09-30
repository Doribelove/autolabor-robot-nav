# 项目主线与推荐论文结构

## 科学主线

1. T12/V2 揭示直接学习高维 TEB 参数存在动作投影、Ackermann 耦合和执行错位。
2. V3-SCOPE 把学习动作改成有限 Ackermann 可行候选，建立候选条件预测、可达占据、硬验证、
   certified stop 和解析排序；V3-11 有限定历史阳性，V3-21 的新鲜表征未复现一般 TEB 优势。
3. V4 首次把 Masked-PPO 用于候选选择，但三轮训练及 fresh 表征仅 0.0980%，学习收益未证明。
4. V5 建立部署期纯传感器 actor、asymmetric critic、解析残差先验和硬动作掩码；正式结果
   6.0936%，虽区间为正但未达到冻结 8% 实效门。
5. V6 只加入 privileged counterfactual trajectory-rank distillation（PCT）单因素，在冻结
   四类 fast sensor-level population 上达到 8.402444% 和正 97.5% 区间；该阳性不等于 Gazebo。
6. V6 早期三次 Gazebo 迁移预检失败。2026-08-10 新研究随后修复完整闭环、公平配置 TEB/DWA，
   完成两轮独立 formal；PCT 对 TEB、DWA 的预注册优势仍未建立。

## 推荐学位论文章节

- 第1章：研究背景、Ackermann 局部导航问题、研究问题与证据边界。
- 第2章：TEB/DWA、PPO/asymmetric critic、privileged learning、shielding 与候选轨迹相关工作。
- 第3章：从 T12/V2 到 V3 的动作对齐与候选规划骨架。
- 第4章：PCT-SCOPE-RL 方法——观测、候选、解析先验、PPO、PCT 标签、硬验证和控制边界。
- 第5章：实验完整性——开发/pilot/formal 隔离、失败记账、散列冻结、公平调参和统计门。
- 第6章：V4/V5 阴性与 V6 fast-population 阳性，作为学习单因素证据。
- 第7章：最新 Gazebo 四方法公平比较，两轮 formal 均为阴性。
- 第8章：消融、失败案例、外部效度与可证伪下一步。
- 第9章：结论；fast-population 阳性和 Gazebo superiority 阴性必须同时保留。
- 实车章节：明确标记 `pending_no_evidence`，不填任何虚构数字。

## 推荐题目方向

“面向 Ackermann 移动机器人的特权候选排序强化学习与可审计安全轨迹选择研究”比“PCT 全面优于
传统规划器”更符合证据。副标题可说明包含 fast sensor-level 证据和 Gazebo 两轮公平验证。
