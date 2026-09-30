# 当前论文更新整合指南

日期：2026-08-08

## 目标

本指南用于把另一台电脑上已经套用学校模板的 V3 论文初稿更新到当前 V3+V6 结果。仓库中的
整合技术稿不是学校格式的替代品；它是内容和证据的权威更新源。

## 推荐论文主线

建议题目：

> 面向 Ackermann 移动机器人的候选条件安全规划与仿真特权强化学习研究

论文主线应按问题演进组织，而不是按所有实验编号逐一罗列：

1. T12/V2 说明高维 TEB 参数动作存在投影和执行语义错位；
2. V3-SCOPE 把动作改为 Ackermann 可行候选轨迹，建立候选条件预测和硬安全骨架；
3. V3 正式结果提供限定的 actual-TEB Gazebo 机制证据，但新鲜 V3-21 没有复现普遍优势；
4. V5 建立传感器到候选 ID 的 Asymmetric PPO，正式结果未达到 8% 实效门；
5. V6 只增加 PCT 特权反事实候选排序蒸馏，得到冻结快速总体内 8.4024% 的正式阳性；
6. V6 Gazebo 迁移预检失败，揭示传感器屏蔽外部有效性缺口；
7. 实车实验保持空白占位，待后续单独研究填写。

## 对学校模板初稿的逐章修改

- 绪论：把“V3 是最终方法”改为“V3 提供安全候选骨架，V6 提供最终强化学习方法”。
- 相关工作：补充 PPO、Asymmetric Actor Critic、Learning by Cheating、safe RL shielding、
  sensor-level navigation RL；不要把这些基础机制写成本论文首创。
- 系统设计：保留 V3 候选生成/预测/硬验证，并新增 V6 sensor actor、解析残差、优势门、PCT。
- 方法章节：明确部署 actor 不读取特权信息，端到端边界是传感器到候选轨迹 ID。
- 实验章节：V3-11、V3-21、V5-03-F1、V6-02-F1 必须分别报告，不能跨合同合并统计。
- 讨论：同时解释 V3 历史阳性与新鲜未复现、V5 阴性与 V6 单因素阳性、快速总体阳性与 Gazebo
  迁移阴性为何可以并存。
- 实车章节：复制占位结构，只保留“待实车实验”，不得填写推测数据。
- 结论：主阳性写 V6 8.4024%，同时保留无 history 收益、窄走廊失败和 Gazebo 失败。

## 三个建议贡献

1. 提出 action-aligned 的 Ackermann 候选级局部导航架构，使学习动作与实际执行轨迹语义一致，
   并以候选条件预测、硬安全验证和 certified-stop 保留可解释回退。
2. 提出 PCT-SCOPE-RL：通过 asymmetric PPO 和特权反事实候选排序蒸馏训练同一传感器 actor，
   部署时仅用真实可得传感器，并由解析先验、优势门和硬掩码共同限制学习权限。
3. 建立包含 V5 阴性、V6 单因素阳性、场景消融与 Gazebo 迁移阴性的可复核实验链；在 80 个
   fresh blocks 上相对解析先验降低失效计入均时 8.4024%，双侧 97.5% CI 为
   [3.4840%,13.7135%]。

## 不得混淆的结果

- V3-11 的 12.9288% 是三个冻结单 actor Gazebo 模板、共享 oracle track 下的历史正式结果；
- V3-21 的 -0.1931% 表明新鲜 failure-inclusive 研究没有证明 V3 普遍优于 Actual TEB；
- V6 的 8.4024% 是四类 sensor-level fast population 相对解析先验的结果，不是 Gazebo/TEB 结果；
- V6 Gazebo 只有三次失败预检，没有资格或正式矩阵；
- V6/no-history 完全相同，不能声称时序记忆收益；
- 实车实验为 pending，不得出现“实车验证表明”等表述。

## 推荐使用顺序

1. 阅读 `README_FIRST.md`；
2. 阅读 `combined_claims_evidence_matrix.yaml`；
3. 阅读 `CURRENT_RESULT_SUMMARY.csv` 和数据字典；
4. 对照整合技术稿 PDF 与独立的 V6 章节；
5. 在学校模板副本中逐章迁移；
6. 使用 `CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md` 启动另一台电脑的 Codex；
7. 编译后逐条检查 forbidden claims 和实车占位。

