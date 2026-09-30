# 数据字典

本包只包含论文写作需要的派生数据，不用于重新选择模型或修改冻结结论。

## 顶层文件

- `CURRENT_RESULT_SUMMARY.csv`：V3/V5/V6/实车占位的统一研究级摘要。
- `combined_claims_evidence_matrix.yaml`：允许、未证明和禁止声明的权威边界。

## data/v3

- `v3_12_*`：V3-11 历史 actual-TEB 正式结果的总表、分层、区块与 claims。
- `method_summary.csv`、`comparison_summary.csv`、`stratum_results.csv`、`block_results.csv`：V3-21 四方法新鲜表征。
- `canonical_episodes.csv`：V3-21 的 120 个 failure-inclusive episode。
- `formal_characterization_assessment.yaml`：V3-21 终态结论。

## data/v5

- `formal_assessment.json`：V5 6.0936% 阴性实效门结论。
- `formal_episodes.csv`、`paired_blocks.csv`、`bootstrap_distribution.csv`：V5 逐回合、区块和 bootstrap 数据。

## data/v6

- `formal_assessment.json`：V6 正式主结论。
- `v6_02_f1_formal_episodes.csv`：320 个正式 episode。
- `v6_02_f1_method_summary.csv`、`v6_02_f1_family_summary.csv`、`v6_effect_summary.csv`：方法、场景和效应表。
- `v6_gazebo_preflight_summary.csv`：三次迁移负结果。
- `selection_report.json`：四训练身份验证选择。

## figures 与 tables

- `figures/v3`：5 张 V3 图，PDF/SVG/PNG。
- `figures/v6`：6 张 V6 图，PDF/SVG。
- `tables/v3`：5 个 V3 booktabs 三线表。V6 的主表已写入独立章节，原始数值以 CSV 为准。
