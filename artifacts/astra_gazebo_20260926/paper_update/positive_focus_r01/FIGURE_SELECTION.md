# 优先选图

先用能直接支持论文方法与已成立结果的图；把开发诊断图留给附录或不选。以下路径均相对于本包。

| 优先级 | 图 | 用途 |
|---|---|---|
| 主文 | `materials/paper/figures/v6/figure_v6_architecture.pdf` | 展示部署 actor 与训练期特权模块边界 |
| 主文 | `materials/paper/figures/v6/figure_v6_training.pdf` | 展示训练与推理流程 |
| 主文 | `materials/paper/figures/v6/figure_v6_formal_summary.pdf` | 展示快速传感器级学习实验的正式主结果 |
| 主文 | `materials/paper/figures/v6/figure_v6_effect_intervals.pdf` | 展示主效应及区间；图注必须标明快速总体 |
| 按需 | `materials/paper/figures/v6/figure_v6_family_ablation.pdf` | 展示收益集中在哪些冻结任务族 |
| 主文 | 技术稿单行人交互规划实验章内的结果表和 TikZ 时间图 | 展示候选条件安全规划方法相对固定 Actual TEB 的限定阳性 |
| 讨论或附录 | `materials/paper/figures/pct_gazebo/formal_effects_97_5_ci.png` | 交代后续 Gazebo 外部验证未达综合优势门 |

`materials/figures_for_insertion/` 的 30 组已验收图主要用于方法、场景和开发诊断。
其 `LOCAL_FIGURE_INDEX.md` 标明每图边界。开发诊断轨迹图与六类场景截图
均不是正式性能阳性证据；若正文篇幅紧张，先不用这些图。源文件路径中保留的版本码只用于
溯源，最终论文中的图题、图注与交叉引用应改为方法名和实验场景名。
