# 现有证据论文技术稿（2026-09-27）

本目录是在冻结历史证据的**副本**上整理的论文写作材料。先读
`FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.pdf`；需要修改正文时编辑同名 `.tex`、
`chapters/`、`tables/`，在本目录运行两次 `xelatex -interaction=nonstopmode
-halt-on-error FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.tex`。PDF 为 45 页 A4 技术稿，
不是学校学位论文模板成稿；作者、学校格式、致谢、规范参考文献和学校审查仍需按实际要求处理。
上级目录的 `FAM_TEB_EXISTING_EVIDENCE_THESIS_R01.tar.gz` 是便携交接档，
同名 `.sha256` 可在其所在目录用 `sha256sum -c` 校验。

## 可写入论文的结论

| 证据 | 结果与范围 | 写作判定 |
|---|---|---|
| V3-11 | 三个冻结单反应式 actor Gazebo 模板、oracle actor track；对 actual TEB 的配对时间下降 12.9288%，单侧 97.5% 下界 12.7253% | 仅限定历史阳性 |
| V3-21 | 新鲜四方法 30 块/120 回合；对 actual TEB 为 -0.1931%，双侧 95% CI [-19.8668%,12.9820%]，另有一项完整性门容差偏差 | 新鲜研究未证明优势；不得与 V3-11 合并 |
| V5 | 快速 sensor-level 总体下降 6.0936%，未达冻结 8% 实际效应门 | 阴性实际效应 |
| V6 | 四类冻结快速 sensor-level 总体 80 块/320 回合；对解析先验下降 8.402444%，双侧 97.5% CI [3.484003%,13.713459%] | 严格限定阳性，不外推 Gazebo |
| Gazebo R1/R2 | 六类冻结模板，160+200 个新鲜配对块、640+800 回合；R2 对 actual TEB -2.7288%，对 DWA +7.7754%，后者未过冻结 8% 点门 | 两轮均未证明 PCT 优于 actual TEB 或 DWA |
| N05 | 12 个开发配对块/24 回合；两臂均 2/12 成功、10/12 超时，处理终态距离平均多 0.5437 m，4 回合工程净距门失败 | 仅负向开发诊断，非正式结果 |

完整的允许/禁止声明以 `evidence/claims_contract.yaml` 和
`evidence/canonical_results.json` 为准。零接触不等于形式化安全；本文没有多 actor、
感知/OOD、部署或实车效果证据。N06 只完成了离线 release 审查，没有登记种子或运行回合，
故未写成论文结果。

## 目录与溯源

- `evidence/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810.tar.gz` 是 8 月外层权威交接包的逐字节副本，
  含正式回执、逐 episode CSV、合同、图表和旧 41 页稿。其原始 SHA256 为
  `c22a369561f1e9041d02fa76a564f3640cd7528a871293be9b163a374bdc4b29`。
- `evidence/` 另放常用正式 assessment/完整性 review、声明合同、规范结果和 N05 独立审查，
  方便逐项核对。逐回合源日志不重复拷贝；权威包的索引可追溯到工作区原件。
- `appendix_figures/FIGURE_ATLAS.pdf` 是 30 组已质检图的候选图册，
  `FIGURE_INDEX.md` 标记每图来源与限制。30 组不等于 30 个正式结果；其中六类场景图来自
  N05 开发回合。正文只直接采用了正式比较图与少量 N05 诊断图，避免用开发图充当正式证据。
- `EVIDENCE_MANIFEST.json` 列出包内关键文件的 SHA256；`VALIDATION.json` 记录数值和
  排版检查；`qc_pages/` 的五张 contact sheet 用于整稿视觉巡检。

## 套学校模板时

优先迁移摘要、绪论、方法、V3/V5/V6 结果、Gazebo 正式 R1/R2、局限与结论；保留
`chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex` 的完整阴性判定和失败计入规则。
N05 单列为开发诊断，不进入正式效应合并。实车章仍只写研究设计，所有结果字段保持空值。
图册只按 `FIGURE_INDEX.md` 选择与正文声明一致的图。学校模板、作者身份及学院规范并未提供，
因此本包不宣称已满足这些外部格式要求。
