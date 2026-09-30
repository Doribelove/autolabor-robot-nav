# 论文更新材料包（2026-09-27）

这是使用**已有、冻结证据**更新论文的便携材料包。先读
[`paper/README.md`](paper/README.md)，再打开
[`paper/FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.pdf`](paper/FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.pdf)。
45 页 A4 技术稿及同名 `.tex`、`chapters/`、`tables/`、`figures/` 可以直接改写和编译；
编译需要 XeLaTeX、Noto Sans CJK SC 字体和相应 TeX 包。它尚未套学校模板。

## 可直接使用的内容

1. `paper/`：完整技术稿、源文件、正文所用图、30 页图册、声明边界、历史权威交接包、
   正式回执副本、N05 开发诊断、可复核哈希与视觉质检记录。技术稿 PDF SHA256 为
   `27ff1b36a0b7e6dc4f32c36a2afe535351d80df454f6a9d2e940346f0475de33`。
2. `figures_for_insertion/`：30 组**已验收版本**各自的 PNG、SVG 与来源 QC；
   [`LOCAL_FIGURE_INDEX.md`](figures_for_insertion/LOCAL_FIGURE_INDEX.md)提供相对路径和使用边界。
   其中 8 组含原生 Gazebo 截图，13 张不同原始帧在 `native_frames/`。
   六类 F06–F11 场景图来自 N05 开发回合，不能写成新的正式独立布局。
3. `formal_gazebo_evidence/`：R1/R2 预注册、发布、schedule、逐回合 CSV、assessment、
   完整性审查及可粘贴 LaTeX 表。R1 为 160 块/640 回合，R2 为 200 块/800 回合；
   两轮均未证明 PCT 优于 Actual TEB 或 DWA。N05 只作开发诊断，绝不合并进正式统计。
4. `project_context/`：历史论文写作与结果转移说明。

`PACKAGE_MANIFEST.json` 给出此 ZIP 内每个文件的 SHA256；解压后可独立核验。
旧 2026-08-10 权威交接包也在 `paper/evidence/`，保留其原始字节。

## 正文结论边界

V6 相对解析先验在四类冻结快速 sensor-level 总体下降 8.402444%，属于限定阳性。
后续 Gazebo R1/R2 共 1440 回合均未达到相对 Actual TEB 或 DWA 的预注册综合优势门。
V3-11 的限定历史阳性与 V3-21 新鲜未复现分别写，不得合并。N05 的 24 回合负结果
仅是开发诊断。本文没有一般规划器优越性、形式化安全、感知/OOD、部署或实车效果证据。

建议更新顺序：先按 `paper/README.md` 核对声明，再将正文套入学校模板，最后从
`figures_for_insertion/` 逐张选择与对应证据层级一致的图。学校模板、学校格式和导师要求
不在此包内，不能以本包代替最终提交审查。本包没有启动或授权任何新实验。
