# 给另一台电脑 Codex 的论文更新提示词

将下面整段提示词发送给另一台电脑上的 Codex，并把其中的路径占位替换为实际路径。

---

你现在负责更新一篇已经套用学校模板的研究生毕业论文。项目根目录为
`<PROJECT_ROOT>`，学校模板论文初稿位于 `<SCHOOL_THESIS_DRAFT>`。请先只读检查两个目录，确认
论文格式、章节、参考文献系统和已有 V3 内容，然后复制初稿形成可回退的工作副本；不要覆盖唯一原稿。

权威论文更新包位于：

`<PROJECT_ROOT>/artifacts/v6/paper/v6_06_p1_thesis_transfer_package`

开始写作前必须完整阅读：

1. `README_FIRST.md`
2. `CURRENT_THESIS_INTEGRATION_GUIDE.md`
3. `combined_claims_evidence_matrix.yaml`
4. `DATA_DICTIONARY.md`
5. `CURRENT_RESULT_SUMMARY.csv`
6. `CODEX_HANDOFF_MACHINE_CONTEXT.md`
7. `chapters/CH09_PCT_SCOPE_RL_INTEGRATED.tex`
8. `chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex`

目标：在不改变学校模板格式的前提下，把原 V3 初稿更新为当前 V3+PCT-SCOPE-RL 论文。论文主线是：

- T12/V2 暴露高维 TEB 参数动作的投影和执行错位；
- V3-SCOPE 建立 Ackermann 可行候选、候选条件预测、可达占据、硬验证和解析排序骨架；
- V5 建立传感器到候选 ID 的 Asymmetric PPO，但 6.0936% 未达到 8% 门；
- V6 仅增加 PCT 特权反事实候选排序蒸馏，在 80 个 fresh blocks 上相对解析先验降低
  failure-inclusive restricted mean time 8.4024%，双侧 97.5% CI [3.4840%,13.7135%]；
- V6/no-history 完全相同，不能声称时序历史收益；
- 三次 V6 Gazebo 迁移预检失败，不能声称 Gazebo、Actual TEB、部署或实车有效；
- 实车实验章节保持明确占位，任何数值均不得编造。

请完成以下工作：

1. 更新中英文题目、摘要、关键词、绪论研究问题和贡献；
2. 在相关工作中加入 PPO、Asymmetric Actor Critic、Learning by Cheating、safe RL shielding、
   TEB 和 sensor-level navigation RL，只把本文的候选级组合与 PCT 排序蒸馏写成创新；
3. 保留 V3 候选条件预测和硬安全框架，把 V6 写成独立证据的学习决策层；
4. 新增完整的 PCT-SCOPE-RL 方法、训练合同、V5 阴性、V6 阳性、消融和 Gazebo 阴性章节；
5. 把 `figures/` 中可编辑图迁入学校模板图目录，把 `tables/` 和 `data/` 中的数值作为唯一数据源；
6. V3-11 与 V3-21 必须分开报告，不得把历史阳性和新鲜阴性合并；
7. 新增有效性威胁，明确 synthetic fast population、Gazebo 迁移失败、无实车和无 OOD；
8. 保留“实车实验设计与待补结果”章节，所有结果位置写“待实车实验”或 `--`；
9. 更新结论，使主强化学习结论限定为四类 sensor-level fast population；
10. 编译论文并检查交叉引用、图表编号、参考文献、字体、公式和学校版式。

严格禁止以下表述：

- “V6 已在 Gazebo 中证明有效”；
- “V6 优于 Actual TEB”；
- “已经证明形式化安全或实车安全”；
- “GRU 历史信息带来性能提升”；
- “方法解决了窄走廊让行或一般多 actor/OOD 导航”；
- “实车实验表明……”，除非用户后来提供了可复核的真实数据；
- 把 V3 的 Gazebo 阳性结果改写成 V6 的结果；
- 删除 V5、V3-21 或 Gazebo 的阴性结果。

允许的核心结论必须保持数字精确：

- V3-11：12.9288%，one-sided 97.5% lower 12.7253%，仅限三个冻结单 actor Gazebo 模板；
- V3-21：V3/Actual TEB 28/30 与 30/30，7.785/7.770 s，-0.1931%，95% CI
  [-19.8668%,12.9820%]，Actual TEB 优势未证明；
- V5：6.0936%，95% CI [1.7867%,11.0069%]，未达到 8% 门；
- V6：60/80 对解析 49/80，13.115625 s 对 14.318750 s，8.402444%，双侧 97.5% CI
  [3.484003%,13.713459%]；
- V6 对 V5：3.161052%，95% CI [0.493580%,6.576508%]；
- V6 正式快速总体中 contact=0、mask violation=0；这些不是形式化安全证明。

完成后交付：

- 更新后的学校模板源文件；
- 可打开的 PDF；
- `THESIS_UPDATE_CHANGELOG.md`，逐章说明修改；
- `CLAIMS_AUDIT.md`，逐项说明每个主要声明的数据来源和边界；
- `PENDING_REAL_ROBOT_FIELDS.md`，列出未来实车数据需要填写的位置；
- 编译错误、缺失引用或无法确认的学校格式问题清单。

不要为了让论文显得更强而扩大结论。若学校初稿与更新包有数字冲突，以
`combined_claims_evidence_matrix.yaml`、CSV 数据和哈希清单为准，并在 changelog 中记录替换。

---

