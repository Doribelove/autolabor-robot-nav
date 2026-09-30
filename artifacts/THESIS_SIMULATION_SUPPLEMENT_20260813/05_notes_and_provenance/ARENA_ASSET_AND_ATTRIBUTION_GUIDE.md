# Arena 高真实感仿真场景图集

本图集从当前工作空间内的 Arena-Rosnav-3D 场景资产重新渲染，提供住宅、书店、医院、工厂和小型仓库五类环境的论文插图。成品图位于 [`arena_images/`](arena_images/)，其中 `raw/` 保存 Gazebo 视口原始截图。

> **统一证据边界：**这些图片只用于说明仿真平台的场景丰富度和环境扩展能力。截图在 Gazebo 暂停态生成，没有启动机器人、导航栈、局部规划器、学习策略或实验评价器，也不是任何正式 episode 的回放。因此，它们不构成算法性能、安全性、泛化性、感知能力或部署有效性的证据。

## 快速选图

| 图片 | 场景内容 | 分辨率 | 推荐用途 |
|---|---|---:|---|
| [`00_arena_environment_overview.png`](arena_images/00_arena_environment_overview.png) | 五类 Arena 三维环境总览，含 `(a)`–`(e)` 标签 | 3600 × 1520 | 仿真平台或环境多样性章节主图 |
| [`01_residential.png`](arena_images/01_residential.png) | 家具密集型住宅内部 | 2400 × 1488 | 居住环境、室内结构与窄空间示例 |
| [`02_bookstore.png`](arena_images/02_bookstore.png) | 书架、展台和服务台组成的书店 | 2400 × 1488 | 货架型障碍、通道与公共空间示例 |
| [`03_hospital.png`](arena_images/03_hospital.png) | 病房、走廊和中央服务区 | 2400 × 1488 | 大型结构化室内环境示例 |
| [`04_factory.png`](arena_images/04_factory.png) | 输送线、货架、托盘和工业通道 | 2400 × 1488 | 工业环境与复杂静态障碍示例 |
| [`05_small_warehouse.png`](arena_images/05_small_warehouse.png) | 货架、堆垛区和装卸区域 | 2400 × 1488 | 仓储环境、规则通道与作业区示例 |

## 五场景总览

![Arena-Rosnav-3D 五类三维仿真环境](arena_images/00_arena_environment_overview.png)

建议图注：

> **图 X.X  Arena-Rosnav-3D 集成的多类型三维仿真环境。**（a）住宅环境；（b）书店环境；（c）医院环境；（d）工厂环境；（e）小型仓库环境。该图为暂停态平台扩展场景示例，非本文冻结正式实验结果。

可配套使用的正文表述：

> 为展示仿真平台对不同空间结构和视觉资产的兼容能力，本文补充给出了住宅、书店、医院、工厂及小型仓库五类三维环境。它们覆盖家具密集空间、规则货架通道、大型走廊网络和工业作业区等不同结构，但未纳入本文冻结的定量评价集合。

## 独立场景图

### 住宅环境

![住宅仿真环境](arena_images/01_residential.png)

该场景包含客厅、厨房、餐桌、隔断和室内家具，可用于展示高密度生活设施条件下的三维环境建模能力。

建议图题：**家具密集型住宅三维仿真环境。**

### 书店环境

![书店仿真环境](arena_images/02_bookstore.png)

该场景由展台、书架、服务台和开放通道构成，适合说明规则货架与开放区域并存的公共室内布局。

建议图题：**包含书架与展台的书店三维仿真环境。**

### 医院环境

![医院仿真环境](arena_images/03_hospital.png)

该场景具有连续走廊、多个房间、病床和中央服务区，能够直观呈现大尺度、多房间的结构化室内空间。

建议图题：**包含病房与走廊网络的医院三维仿真环境。**

### 工厂环境

![工厂仿真环境](arena_images/04_factory.png)

该场景包含输送线、货架、托盘、楼梯和工业设备，可用于展示多层次障碍、作业区边界及狭长通行区域。

建议图题：**包含输送线与工业货架的工厂三维仿真环境。**

### 小型仓库

![小型仓库仿真环境](arena_images/05_small_warehouse.png)

该场景包含规则货架、堆垛区、装卸区和地面作业标线，适合说明仓储类环境的规则结构和通道约束。

建议图题：**包含货架、堆垛区与装卸区的小型仓库仿真环境。**

## LaTeX 示例

```latex
\begin{figure}[htbp]
  \centering
  \includegraphics[width=\textwidth]{docs/thesis_experiment/thesis_scene_gallery/arena_images/00_arena_environment_overview.png}
  \caption{Arena-Rosnav-3D集成的多类型三维仿真环境：（a）住宅；（b）书店；（c）医院；（d）工厂；（e）小型仓库。该图为平台扩展场景示例，非本文冻结正式实验结果。}
  \label{fig:arena_environment_gallery}
\end{figure}
```

如版面允许，也可以选择一张独立图与本文正式实验场景图并列，但正文应明确区分“平台可加载的扩展环境”和“用于定量评价的冻结实验场景”。

## 来源与署名

本图集中的 PNG 是作者基于本地 world 文件于 2026-08-12 重新渲染的暂停态截图，不是直接复制的上游预览图。

- 场景集成与本地 world 来源：[Arena-Rosnav-3D](https://github.com/ignc-research/arena-rosnav-3D)。该仓库提供 GPL-2.0 许可文本并要求使用者引用 Arena-Rosnav 相关论文。
- 住宅、书店、医院和小型仓库的主要世界资产分别来自 AWS RoboMaker 的 [Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros1)、[Bookstore World](https://github.com/aws-robotics/aws-robomaker-bookstore-world/tree/ros1)、[Hospital World](https://github.com/aws-robotics/aws-robomaker-hospital-world/tree/ros1) 和 [Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros1)。正式发表时应同时保留对应项目的 `LICENSE`/`NOTICE` 与署名要求。
- 工厂环境包含来自 ARIAC、Open Robotics/OSRF 和 [Gazebo Model Database](https://github.com/osrf/gazebo_models) 的模型来源线索。不同模型可能采用不同许可，本图集不将全部内嵌资产笼统归入 Arena 仓库的 GPL-2.0。

推荐署名模板：

> 场景集成：Arena-Rosnav-3D；住宅、书店、医院及小型仓库的主要世界资产：AWS RoboMaker Worlds；其他 Gazebo/ARIAC 模型按各自模型元数据署名。图片由作者基于本地场景重新渲染。

## 可复现记录

- Arena 本地提交：`634bcb091a90b362087cdba5a9cd3856466d493c`
- 截图日期：2026-08-12（Asia/Shanghai）
- Gazebo：Classic 11.15.1
- 渲染方式：独立 `gzserver -u` 暂停态服务和 `gzclient`，隔离端口 `11745`
- 输出方式：截取 Gazebo 渲染视口后统一缩放至 2400 像素宽，并通过 FFmpeg 生成总览图

| 场景 | 源 world SHA-256 | 固定 GUI 相机 pose |
|---|---|---|
| 住宅 | `d2a2c4a478bb97bf66ffa9683ab2a2c5b9b3db634b75dc2eab9ead771068804a` | `-4.4965503 -0.4874 1.508684 0 0 0` |
| 书店 | `488b503b51511b56dba54a3398f665349b817e70579277b625aa95700ed3e2b6` | `-5.73 3.76 8.10 0 1.1316 -0.4398` |
| 医院 | `818a5eab5a9eedc43050d2fb3a2345d267dd32843c4139f29d58d06b396e2c95` | `-27 -41 39 0 0.68 0.60` |
| 工厂 | `79866ba67199601396e828b9269864e1994bbe58896704003b386add936d5e50` | `-15 -24 30 0 0.68 0.68` |
| 小型仓库 | `bef748d97c784056a242b2578554379386c3104a00e5693afc33872d932f6a7d` | `0 0 30 0 1.570796 0` |

其中 `factory.world` 和 `small_warehouse.world` 在截图前已经存在用户修改；表中的 SHA-256 绑定的是本次实际用于生成临时快照的精确字节。截图流程没有改写这两个源文件。

## 安全复现

重新生成全部 Arena 图片：

```bash
cd /home/robot/robot_ws_base_rl
DISPLAY=:0 bash docs/thesis_experiment/thesis_scene_gallery/scripts/capture_arena_gallery.sh
```

只重拍指定场景，例如书店：

```bash
DISPLAY=:0 bash docs/thesis_experiment/thesis_scene_gallery/scripts/capture_arena_gallery.sh bookstore
```

不启动 Gazebo、只重新拼接现有五张图：

```bash
bash docs/thesis_experiment/thesis_scene_gallery/scripts/build_arena_overview.sh
```

该流程会先通过 [`sanitize_arena_world.py`](scripts/sanitize_arena_world.py) 将源 world 转换为临时展示副本，移除运行时插件、传感器脚本和原 GUI 配置，再注入固定相机；截图结束后删除临时副本。它不会调用 Arena 的 `start_arena_gazebo.launch` 或 `generate_world.py`，因此不会触发任务生成器对源 world 的原地重写。

## 学术表述注意事项

- 可以写“平台集成了多类高细节三维环境”或“图中展示平台的场景扩展能力”。
- 不应写“算法已在五类环境中验证”“取得跨场景泛化”或“适用于医院/工厂部署”。
- 图片中没有本文算法轨迹、传感器输出或性能指标；不要把视觉丰富度等同于实验真实性或方法有效性。
- 正式实验结论仍应引用冻结的实验设计、日志、统计表格和评审过的结果图。
