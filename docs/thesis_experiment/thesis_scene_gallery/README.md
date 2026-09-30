# 论文仿真场景图集

本目录提供两组可直接用于论文排版的 Gazebo 场景图：一组是 M2 机器人与六类典型导航场景，另一组是 Arena-Rosnav-3D 集成的住宅、书店、医院、工厂和小型仓库五类高细节环境。

> **使用边界：**这些图片是在暂停状态下对展示性 Gazebo 世界进行的静态截图。M2 图组的场景几何参照冻结的 PCT/TEB/DWA 六类基准场景，但材质、路线标记、动态参与者的展示位置和相机视角为论文可读性做了调整；Arena 图组只展示平台可加载的扩展环境。两组图片都不是正式实验 episode 的回放，不构成新的性能、安全性、泛化性或统计证据。

Arena 图组的选图、图注、来源署名、证据边界与安全复现说明见 [`ARENA_SCENE_GUIDE.md`](ARENA_SCENE_GUIDE.md)。

基于 hospital 世界完成的四方法 Q1 资格实验、双口径结果和轨迹图见 [`../arena_algorithm_comparison/ARENA_Q1_RESULTS.md`](../arena_algorithm_comparison/ARENA_Q1_RESULTS.md)。场景截图仍只用于环境展示，量化结论以该结果文档绑定的原始证据为准。

## Arena 高细节环境总览

![Arena-Rosnav-3D 五类三维仿真环境](arena_images/00_arena_environment_overview.png)

建议图注：

> **图 X.X  Arena-Rosnav-3D 集成的多类型三维仿真环境。**（a）住宅环境；（b）书店环境；（c）医院环境；（d）工厂环境；（e）小型仓库环境。该图为暂停态平台扩展场景示例，非本文冻结正式实验结果。

## 快速选图

| 图片 | 内容 | 分辨率 | 推荐用途 |
|---|---|---:|---|
| [`00_six_scene_overview.png`](images/00_six_scene_overview.png) | 六类场景组合图，已含 `(a)`–`(f)` 标签 | 3600 × 1240 | 实验场景设计、仿真环境章节的主图 |
| [`01_m2_model_overview.png`](images/01_m2_model_overview.png) | M2 模型、静态障碍及起终点的斜视图 | 2400 × 1488 | 系统平台、机器人模型或 Gazebo 环境介绍 |
| [`02_static_avoidance.png`](images/02_static_avoidance.png) | 静态障碍避让 | 2400 × 1122 | 静态场景单独说明 |
| [`03_single_crossing.png`](images/03_single_crossing.png) | 单个动态参与者横穿 | 2400 × 1122 | 动态交互场景说明 |
| [`04_partially_occluded_crossing.png`](images/04_partially_occluded_crossing.png) | 局部遮挡条件下的横穿 | 2400 × 1122 | 遮挡/有限可见性场景说明 |
| [`05_head_on_yield.png`](images/05_head_on_yield.png) | 迎面交互与让行 | 2400 × 1122 | 会车与速度协调场景说明 |
| [`06_sequential_interaction.png`](images/06_sequential_interaction.png) | 连续双参与者交互 | 2400 × 1122 | 多阶段决策场景说明 |
| [`07_width_change_corridor.png`](images/07_width_change_corridor.png) | 宽度变化走廊 | 2400 × 1122 | 结构约束与通道变化场景说明 |

## 六场景总览

![六类 Gazebo 仿真场景总览](images/00_six_scene_overview.png)

建议图注：

> **图 X.X  Gazebo 仿真测试场景。**（a）静态障碍避让；（b）单行人横穿；（c）局部遮挡横穿；（d）迎面交互与让行；（e）连续双行人交互；（f）宽度变化走廊。

适合放在“仿真环境与测试场景”小节。该图说明不同场景的空间结构和交互对象，不应配合正文表述为算法性能结果。

## 机器人与环境总览

![M2 机器人及静态障碍场景](images/01_m2_model_overview.png)

建议图注：

> **图 X.X  M2 移动机器人及静态障碍仿真环境。**蓝色模型为 M2 移动机器人，橙色立方体为静态障碍物，蓝色和绿色圆盘分别表示起点与目标点。

适合放在“实验平台”“机器人模型”或“Gazebo 仿真环境搭建”小节。

## 独立场景图与文字说明

### 静态障碍避让

![静态障碍避让场景](images/02_static_avoidance.png)

机器人需要在有限宽度的走廊中绕过位于主行进方向上的静态障碍物，并继续向目标点运动。该场景主要展示局部路径可行性和障碍物绕行约束。

### 单行人横穿

![单行人横穿场景](images/03_single_crossing.png)

红色圆柱表示动态参与者（行人代理），半透明红线表示其横穿方向。机器人与动态参与者的运动方向在道路中央相交，用于展示典型的时空冲突情形。

### 局部遮挡横穿

![局部遮挡横穿场景](images/04_partially_occluded_crossing.png)

深灰色块表示遮挡体，红色动态参与者位于遮挡体后方并沿横向路径接近道路。该构型用于说明有限可见条件下潜在冲突的出现过程。

### 迎面交互与让行

![迎面交互与让行场景](images/05_head_on_yield.png)

动态参与者沿与机器人相反的纵向方向运动，形成迎面交互。该场景可用于介绍狭窄通道中的速度协调、让行或保守通行策略。

### 连续双行人交互

![连续双行人交互场景](images/06_sequential_interaction.png)

两个动态参与者在不同纵向位置依次横穿机器人行进路线，用于展示连续冲突下的多阶段局部决策需求。

### 宽度变化走廊

![宽度变化走廊场景](images/07_width_change_corridor.png)

道路边界由宽通道过渡至窄通道，再恢复为宽通道，用于展示局部空间约束变化对候选轨迹和速度选择的影响。

## 颜色与符号约定

- 蓝色长方体：Gazebo 中的 M2 机器人模型。
- 蓝色圆盘：起点；绿色圆盘：目标点。
- 黄色虚线：用于帮助读者识别主行进方向的展示标记，不是算法输出轨迹。
- 橙色立方体：静态障碍物。
- 红色圆柱和红线：动态参与者及其展示性运动方向。
- 深灰色块：遮挡体；浅灰色边界：走廊墙体。

## 可直接使用的 LaTeX 示例

若论文工程能够从仓库根目录访问本图集，可使用：

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=\textwidth]{docs/thesis_experiment/thesis_scene_gallery/images/00_six_scene_overview.png}
  \caption{Gazebo仿真测试场景。（a）静态障碍避让；（b）单行人横穿；（c）局部遮挡横穿；（d）迎面交互与让行；（e）连续双行人交互；（f）宽度变化走廊。}
  \label{fig:gazebo_six_scenes}
\end{figure}
```

机器人模型总览图可使用：

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=0.88\textwidth]{docs/thesis_experiment/thesis_scene_gallery/images/01_m2_model_overview.png}
  \caption{M2移动机器人及静态障碍仿真环境。}
  \label{fig:m2_gazebo_overview}
\end{figure}
```

如果论文源文件不在仓库根目录，请按实际位置调整图片相对路径，或将 `images/` 复制到论文的图片资源目录。

## 文件结构

```text
thesis_scene_gallery/
├── README.md                  # M2 场景图集总说明
├── ARENA_SCENE_GUIDE.md       # Arena 图组、图注、来源与证据边界
├── arena_images/              # Arena 论文成品图
│   └── raw/                   # Arena Gazebo 渲染视口原图
├── images/                    # 论文可直接使用的 PNG
│   └── raw/                   # Gazebo 渲染视口的原始截图
├── source_worlds/             # 展示性、暂停态 Gazebo SDF 世界
└── scripts/
    ├── build_gallery_worlds.py
    ├── build_arena_overview.sh
    ├── build_overview.sh
    ├── capture_arena_gallery.sh
    ├── capture_gallery.sh
    ├── raise_x11_window.py
    └── sanitize_arena_world.py
```

## 复现截图

在本工作空间中执行：

```bash
cd /home/robot/robot_ws_base_rl
python3 docs/thesis_experiment/thesis_scene_gallery/scripts/build_gallery_worlds.py
DISPLAY=:0 bash docs/thesis_experiment/thesis_scene_gallery/scripts/capture_gallery.sh
```

截图脚本具有以下约束：

- 只 source ROS Noetic 和当前论文工作空间的 `devel/setup.bash`，不会 source 或修改稳定实车工作空间 `/home/robot/robot_ws`。
- 使用隔离的 ROS/Gazebo master 端口，并在开始前拒绝已有的 `rosmaster`、`gzserver` 或 `gzclient` 进程。
- 每个世界以 `paused:=true` 启动，只生成 M2 模型后截图；不启动 `move_base`、TEB、DWA、PPO、评估器或正式实验 runner。
- 每个场景截图后都会清理本次启动的进程；原始渲染视口保留在 `images/raw/`。
- 最后自动生成裁切后的独立图片和带中文子图标签的 `00_six_scene_overview.png`。

如只需重新拼接总览图、无需启动 Gazebo，可执行：

```bash
bash docs/thesis_experiment/thesis_scene_gallery/scripts/build_overview.sh
```

## 学术表述注意事项

这些图适合说明机器人模型、场景构型、交互对象和空间约束。动态参与者的位置是为清楚呈现场景语义而选取的静态展示位置，并非从某个正式试验时刻抽帧；路线标记也只是视觉辅助。论文中的成功率、时间、碰撞、统计显著性或算法优越性结论，应继续引用已经冻结并审查的正式结果表格和图，而不是从本图集推导。
