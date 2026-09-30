# V3-22 论文材料与终态 PDF 导出交接

日期：2026-08-06

## 已完成工作

V3-22 将当前工程整理为可直接用于研究生学位论文的材料闭环：

- V3-18-A0：项目审计、系统目标/架构、实验库存、方法证据矩阵、采集 schema、缺口分析；
- V3-18-B0-R1：五个历史研究共 825 行 canonical episode dataset；
- V3-19--V3-21：四方法、多场景、30 fresh seeds、120 episodes 的资格、故障保留、端口修复、
  完整执行、统计和独立审查；
- V3-22-P1：5 张论文图各三种格式、5 个 booktabs 三线表、配置表、中英文结果叙述和 claims；
- V3-22-P2：32 页 A4 XeLaTeX 论文 PDF、机器结果摘要、build environment、evidence manifest
  和独立 paper review。

全程没有修改 `/home/robot/robot_ws`，没有硬件、CAN、串口、在线 TEB 参数写入或实车运动。
V3-22-P1/P2 均为纯离线派生；训练步数为 0。

## 系统目标与架构

系统目标是在动态交互中选择 Ackermann 可执行、可解释、可回退的局部轨迹。数据流为：

```text
感知/定位/全局路径/健康状态
→ immutable scene snapshot
→ Ackermann-feasible candidates + certified stop
→ candidate-conditioned predictor
→ calibrated reachable occupancy
→ hard verifier
→ analytic ranker
→ tracker
→ /cmd_vel_navigation
→ GPS limiter → FOD arbiter → M2 driver
```

机器可读架构：`artifacts/v3/thesis/v3_18_a0_project_audit/system_architecture.yaml`。

## 论文材料

目录：`artifacts/v3/thesis/v3_22_p1_paper_materials/`

- 5 图 × PDF/SVG/300-dpi PNG；
- 5 个 LaTeX booktabs 三线表；
- `RESULTS_NARRATIVE_ZH.md` 与 `RESULTS_NARRATIVE_EN.md`；
- `paper_claims.yaml`；
- materials manifest SHA：`658e906c3fae23d86e5cab5e65edbe6ca00cc74c1188b0404294f29c4fe973d0`；
- independent review SHA：`88547275a6627027517aa55d2128469bb30de1721fc162e92ab9d6d787be0959`。

## 终态论文

PDF：

```text
artifacts/v3/paper/v3_22_p2_thesis_export/FAM_TEB_V3_THESIS_DRAFT.pdf
```

终态属性：

```text
pages=32
paper=A4
encrypted=false
embedded_font_rows=15/15
latex_errors=0
undefined_references=0
figures=5
three_line_tables=5
```

冻结哈希：

- PDF SHA：`22f4e423a4211590c2d4936c8be4a7f5d6963da83083e0dd8f86b0197af58b7d`
- evidence manifest SHA：`14be2d50096d1ed5f50f5c3a0ed6138ee5089dd5e7c379834aea38b9bc2487e6`
- independent paper review SHA：`adb4317773484601dd3a1b2006098051521c611fdaa7073138991112a7a3d7a5`
- terminal freeze SHA：`8c0ddec2b3a42ea6af00dc1b6d3130ed8c4337f86fb5a8b0f551149248cb5dd9`

独立 review 状态：`terminal_thesis_pdf_independent_review_pass`。

## 复现入口

完整说明：`docs/thesis_experiment/THESIS_EXPERIMENT_REPRODUCTION.md`。

终态离线检查：

```bash
python3 src/tools/thesis_experiment/scripts/assess_v3_21_f1_characterization.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_21_f1_characterization.py --check
python3 src/tools/thesis_experiment/scripts/build_v3_22_p1_paper_materials.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_22_p1_paper_materials.py
python3 src/tools/thesis_experiment/scripts/build_v3_22_p2_thesis_export.py --check
python3 src/tools/thesis_experiment/scripts/review_v3_22_p2_thesis_export.py
sha256sum -c artifacts/v3/paper/v3_22_p2_thesis_export/terminal_freeze.sha256
```

## 最终论文声明

- 历史 V3-11：限定三个 single-reactive-actor Gazebo 模板、共享 oracle track 的正式阳性；
- 新鲜 V3-21：没有证明 failure-inclusive Actual TEB 时间优势；
- V3-21 相对去候选条件化的 49.454% 是探索性机制证据；
- 不能声称 V3 普遍优于 TEB、感知/OOD/多人泛化、形式安全、学习收益、部署或实车性能。

## 后续边界

当前论文整理目标已完成。允许的后续包括纯离线文字润色、学校模板迁移、参考文献格式化和冻结
证据复核。任何更强科学结论都需要新的预注册研究、fresh seeds/预算和独立审查；禁止修改或重跑
V3-19--V3-22 的冻结身份。实车环节仍需单独现场授权。

