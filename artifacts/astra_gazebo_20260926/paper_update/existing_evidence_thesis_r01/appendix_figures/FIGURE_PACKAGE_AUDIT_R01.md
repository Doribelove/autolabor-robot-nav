# Astra 论文图表阶段交付审计

本阶段已有 **30 个不同的已验收图组**，每组均绑定原始数据或原生画面、PNG/PDF/SVG，以及机器、实际视觉和论文尺寸质检记录。[图册 PDF](working_atlas_r12/FIGURE_ATLAS.pdf)为 30 页，SHA256 `63a656f6d59cce29c54cde51ec8a7c8ffd11bdd777db60e929b36d6fd61eaa96`；[中文索引](working_atlas_r12/FIGURE_INDEX.md)、[LaTeX 插图代码](working_atlas_r12/latex_figures.tex)与[机器审计清单](FIGURE_PACKAGE_AUDIT_R01.json)可直接复核。审计 JSON SHA256 `9ec5bbd73e1cc53f22bccf7a965b3b858328f91f44c51dce57f892ffd7cfa133`。

其中 8 组含真实 Gazebo 原生截图：Q01、NAV05、F06–F11，合计引用 13 张不同原始帧。F06–F11 分别覆盖六类核心任务；这六组取自既有 N05 development 回合，不能冒充六组新的正式独立布局。原始画面目录另登记 149 个 RAW 条目，包含 21 个机器失败条目和仍只完成总览或待查的帧；它们没有被自动升级为论文图。图册的 30 个图组均有独立来源和版式，拒收的中间修订版继续保留。

图册现有的 N04/N05、D04、M13 图主要是历史或新 development 诊断、离线几何与实现剖析。N05 是完整保留超时和工程净距失败的阴性开发结果。图组数量和版式质检达标，不等于训练、多种子正式对照、一般安全、shadow 或实车验证达标。后续新结果须另起图册版本，保留此版及其 SHA。

重建图册可在仓库根目录运行：

```bash
python3 artifacts/astra_gazebo_20260926/paper_update/build_working_atlas.py \
  --output artifacts/astra_gazebo_20260926/paper_update/working_atlas_NEW_VERSION
```

重新核对接受图、三种格式、原生帧和图册哈希时运行：

```bash
python3 artifacts/astra_gazebo_20260926/paper_update/audit_figure_package.py \
  --output artifacts/astra_gazebo_20260926/paper_update/FIGURE_PACKAGE_AUDIT_NEW_VERSION.json
```
