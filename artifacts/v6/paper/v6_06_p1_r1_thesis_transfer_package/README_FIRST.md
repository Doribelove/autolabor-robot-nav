# README FIRST：当前 V3+V6 论文更新移交包

这是面向另一台电脑学校模板论文写作的自包含更新包。它不替代学校模板，但提供当前实验数据、
可编辑图、三线表、整合技术稿、声明边界、实车占位和 Codex 提示词。

## 最先打开

1. `FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf`：当前内容整合稿；
2. `CURRENT_THESIS_INTEGRATION_GUIDE.md`：如何修改旧 V3 初稿；
3. `CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md`：直接交给另一台电脑 Codex；
4. `combined_claims_evidence_matrix.yaml`：任何结果冲突时的权威声明边界；
5. `DATA_DICTIONARY.md`：数据和图表说明；
6. `chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex`：保持为空的实车章节。

## 当前核心结论

V6 PCT-SCOPE-RL 在四类冻结 sensor-level fast population 的 80 个 fresh paired blocks 中，
相对解析先验降低 failure-inclusive restricted mean time 8.402444%，双侧 97.5% CI 为
[3.484003%,13.713459%]。该结果不是 Gazebo、Actual TEB 或实车结论；三次 Gazebo 预检失败，
实车实验尚未执行。

## 校验

在本目录运行 `sha256sum -c terminal_content_manifest.sha256`。整合稿由冻结 V3 源派生，未修改
V3/V5/V6 上游证据。
