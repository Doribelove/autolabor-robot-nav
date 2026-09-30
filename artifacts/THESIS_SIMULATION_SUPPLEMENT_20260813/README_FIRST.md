# 论文导出后仿真补充材料包

本目录是上次论文主稿导出之后新增的仿真图片、轻量结果表和论文回填说明，适合整体复制到另一台电脑继续排版。它是**增量补充包**，不是完整论文工程，也不替代先前导出的主稿 PDF。

## 基线与范围

- 上次论文导出基线：`FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf`，本地导出时间为 2026-08-08。
- 本包收录时间范围：2026-08-10 至 2026-08-13。
- 本包收录三批后续内容：Gazebo 四方法正式对比、M2/Arena 场景展示图、Arena Hospital 四方法 Q1 资格实验。
- 本包不含 ROS/Gazebo 原始日志、开发缓存、模型资产、源 world、训练数据或完整论文源文件。

## 换电脑后的最快用法

1. 解压后保持目录结构不变，把整个文件夹放到论文工程旁边。
2. Word/WPS 排版优先使用 PNG；LaTeX 或需要无损缩放时，Gazebo 正式结果图可使用 SVG。
3. 先看 [图注与放置建议](FIGURE_CAPTIONS_AND_PLACEMENT.md)，再从下表推荐图中选取，不必把所有独立场景图都放入正文。
4. LaTeX 用户可复制 [LATEX_SNIPPETS.tex](LATEX_SNIPPETS.tex) 中的示例。
5. 传输后如需核验文件完整性，在本目录执行 `sha256sum -c SHA256SUMS.txt`。

## 推荐优先回填的图片

| 论文位置 | 推荐图片 | 用途 |
|---|---|---|
| 仿真平台与场景设计 | [M2 六场景总览](02_m2_scene_gallery/00_six_scene_overview.png) | 一张图概括六类冻结基准场景的空间和交互语义 |
| Arena 平台扩展环境 | [Arena 五环境总览](03_arena_scene_gallery/00_arena_environment_overview.png) | 展示住宅、书店、医院、工厂和仓库等高细节场景 |
| Gazebo 对比实验结果 | [两轮正式效应及区间](01_gazebo_formal_comparison/figures/png/formal_effects_97_5_ci.png) | 呈现 PCT 相对 Actual TEB/DWA 的效应及 97.5% 区间 |
| Gazebo R2 结果分析 | [R2 方法结果](01_gazebo_formal_comparison/figures/png/formal_r2_method_outcomes.png) | 对照失败计入时间与有效到达率 |
| Arena 资格实验 | [双口径失败计入时间](04_arena_q1_qualification/figures/failure_inclusive_time.png) | 同时展示原始到达和统一安全规范口径 |
| Arena 交互案例分析 | [Hospital 执行轨迹](04_arena_q1_qualification/figures/hospital_route_trajectories.png) | 展示四方法及闭环反应式代理轨迹 |

## 目录说明

| 目录 | 内容 | 证据性质 |
|---|---|---|
| `01_gazebo_formal_comparison/` | R1/R2 四方法正式图、PNG 兼容版、CSV 和完整性摘要 | 可报告冻结数值，但两轮均未建立对 TEB/DWA 的预注册总体优越性 |
| `02_m2_scene_gallery/` | M2 模型及六类论文场景 PNG | 静态展示图，不是正式 episode 回放 |
| `03_arena_scene_gallery/` | 五类 Arena 高细节环境 PNG | 暂停态平台展示，不是算法验证证据 |
| `04_arena_q1_qualification/` | Hospital 单路线 Q1 的两图、三表和机器可读摘要 | 单世界、3 个配对块的描述性资格结果，不是正式推断实验 |
| `05_notes_and_provenance/` | 详细结果解释、证据边界、来源署名、协议和封印摘要 | 用于核对论文措辞和追溯出处 |

## 必须保留的结果边界

### Gazebo R1/R2 正式对比

- R2 中 PCT-SCOPE-RL 为 180/200 有效到达，失败计入均时 14.8413 s。
- R2 相对 Actual TEB 的时间降低为 −2.73%，双侧 97.5% CI 为 [−9.81%, 3.59%]。
- R2 相对 DWA 的时间降低为 7.78%，双侧 97.5% CI 为 [2.55%, 12.61%]，但未达到预注册的 8% 点效应门限。
- R1 与 R2 的所有主比较均未通过全部预注册门限。因此可写“完成率和部分场景表现出优势或机制改善”，不可写“正式证明全面优于 TEB/DWA”。

详细数值和推荐结论见 [Gazebo R1/R2 回填说明](05_notes_and_provenance/GAZEBO_R1_R2_RESULT_TRANSFER.md)。

### Arena Hospital Q1

- 12/12 episode 的证据链、平台完整性和接触心跳均通过，四方法原始到达均为 3/3。
- 按冻结的保守净空规范，PCT 与解析先验均为 3/3 合格，Actual TEB 与 DWA 均为 1/3 合格。
- PCT 相对解析先验的安全口径均时描述性降低 25.246%，三个块方向一致。
- DWA 的原始到达均时为 10.038 s，快于 PCT 的 12.011 s；不能只展示安全封顶口径而隐去这一反例。
- 该结果仅覆盖 Arena Hospital 的一条路线和 3 个块，不包含显著性检验，不能表述为普遍 Arena 优势、形式安全、感知/OOD 泛化或部署结论。

完整双口径解释见 [Arena Q1 结果说明](05_notes_and_provenance/ARENA_Q1_RESULT_GUIDE.md)。

### 两组场景图库

- M2 图组的几何参照冻结的六类基准场景，但材质、标记、展示位置和相机视角为论文可读性进行了调整。
- Arena 图组是在 Gazebo 暂停态重新渲染的环境展示，未启动机器人、导航栈、学习策略或评价器。
- 场景图可以证明平台能够加载和展示这些环境，不能单独证明算法在这些环境中取得了性能或泛化效果。
- 使用 Arena 图时，请保留 [Arena 来源与署名说明](05_notes_and_provenance/ARENA_ASSET_AND_ATTRIBUTION_GUIDE.md) 中的来源边界。

## 格式说明

- 正式 Gazebo 图同时提供 `figures/svg/` 和 `figures/png/`。两者内容一致，PNG 是为 Word/WPS 和跨电脑兼容额外渲染的版本。
- M2、Arena 和 Q1 图以 PNG 提供，分辨率足以用于论文页面排版。
- `tables/` 中的 CSV 可直接用于重建表格；不要把 development 消融表当作正式主结果。
- `05_notes_and_provenance/` 中部分文档是从原仓库复制的追溯副本，其中的仓库内相对链接在另一台电脑上可能不可用，但正文、数值、散列和证据边界均保留。

## 与此前论文结果的关系

本包只封装“上次导出之后”的增量材料。此前 V6、V3 等已进入主稿的正式证据仍以原论文导出和各自冻结证据为准，不应与本包的 Gazebo R1/R2 或 Arena Q1 样本合并计算，也不应相互替代其结论边界。

