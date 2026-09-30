# 论文写作系统完整交接包：从这里开始

这是 2026-08-10 的外层权威交接包，面向另一套论文写作系统。它把历史 V3/V4/V5/V6 论文包与
最新完成的 PCT-SCOPE-RL、解析先验、Actual TEB、DWA 两轮 Gazebo 正式研究合并到同一证据入口。

## 最重要的更新

旧 V6-06 包在 2026-08-08 正确写明“V6 尚无 Gazebo 正式矩阵”。该陈述现在只具有历史意义：
2026-08-10 后续研究已经修复 Gazebo 链路、公平调优传统规划器并完成两轮全新正式矩阵。
新结果是合格阴性，不得把“后来完成 formal”误写成“后来获得优势”。

- R1：160 paired blocks / 640 episodes；未证明优于 TEB 或 DWA。
- R2：200 paired blocks / 800 episodes；仍未证明优于 TEB 或 DWA。
- R2 对 DWA 为 7.7754%，97.5% CI [2.5521%,12.6148%]，但低于冻结的 8% 点门，因此严格为阴性。
- R2 对 Actual TEB 为 -2.7288%，97.5% CI [-9.8083%,3.5914%]。

## 必读顺序

1. `00_READ_ME_FIRST.md`（本文件）
2. `claims_contract.yaml`
3. `canonical_results.json`
4. `data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`
5. `01_PROJECT_STORY_AND_PAPER_OUTLINE.md`
6. `02_METHOD_FAIRNESS_AND_EXPERIMENT_DESIGN.md`
7. `03_RESULTS_STATISTICS_AND_FAILURES.md`
8. `04_CLAIMS_LIMITATIONS_AND_NEXT_WORK.md`
9. `05_DATA_DICTIONARY.md`
10. `06_FIGURE_TABLE_CATALOG.md`
11. `07_SUPERSESSION_NOTICE.md`
12. `08_PROMPT_FOR_PAPER_WRITING_SYSTEM.md`

随后阅读 `benchmark/THESIS_RESULT_TRANSFER.md` 和 `chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex`。
历史论文主线、V3/V5/V6 数据、41 页整合稿和旧图表在 `upstream_v6_06_historical/`。

## 数据优先级

发生数字冲突时，依次以 `claims_contract.yaml`、`canonical_results.json`、新 R1/R2 assessment、
逐 episode CSV、旧历史包为准。不要用 Markdown 中的四舍五入值替代机器文件中的原始精度。

## 原始证据与体积策略

本包复制论文必需的 assessment、CSV、preregistration、release、review、配置、代码和模型摘要。
约 868 MB 的逐 episode Gazebo 日志不重复复制；`indices/full_raw_benchmark_evidence_index.csv`
给出每个原始文件的仓库路径、字节数和 SHA256，可在项目工作区逐项追溯。

本包不授权训练、仿真、调参、实车或在线 TEB 写入。另一系统只需写作、排版和声明审计。
