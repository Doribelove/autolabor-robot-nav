# 三路环视与导航调参合并记录

2026-10-23。工作目录 `/home/slam/robot_j6m_ws_conbine`，本地分支
`robot_j6m_ws_conbine`。目录名称沿用用户指定的 `conbine` 拼写。
本目录是独立 Git 仓库，数据库在本目录 `.git`，与两个来源目录同级。

## 来源与保存方式

| 来源 | 原目录与分支 | 本仓库的完整源码快照 |
| --- | --- | --- |
| 三路环视 | `/home/slam/j6m-dev-b-surround`，`vision-bpu-surround` | `source-snapshots/vision-20261023`，`e5feee57ca6341ab97c70b5a1b18063cd82deac0` |
| 导航调参 | `/home/slam/robot_j6m_ws_navigation_tuning_20261023`，`feature/qt-navigation-tuning-20261023` | `source-snapshots/navigation-20261023`，`21c924df47b7660f6906bf35e56ce1024fe4a614` |

用户称当前视觉分支为 `vision_bpu_dev`；本机没有这个分支。已说明并按当前工作目录的
三路环视版本合并，没有选择父目录的 ZED 版本。
两源的原 HEAD 均为 `5d1028efa93cf34d2179b03384085c87a456402a`，重要成果在未提交及
未跟踪文件中。先在新仓库独立保存两份完整源码快照，再执行有两个父提交的合并；
未在源仓库提交、切分支、重置或清理，也未推送。

视觉来源清单 1873 个文件，导航来源 1779 个文件，包含已跟踪文件和未忽略的新增源码。
快照保留原可执行位和符号链接。逐文件 SHA256、大小、模式及原 Git 状态的前后检查均一致。
清单、原 tracked 补丁、变更文件恢复包和快照脚本在 `runtime/conbine_merge_20261023/`；
源码本身也完整保存在上述两个 Git 提交中。忽略的模型、镜像、日志、地图和私有运行资产
没有混入提交；运行时复用关系见下文。

## 合并内容

- 原 C++ Qt 的三路 GDC 透视视频、端口 0/1/2 切换、图像与同帧检测结果、材质分类及状态显示。
  模型仍是板端原 YOLO11-GAM 与五类材质分类 HBM，没有修改 backbone 或重新训练。
- 新增“导航调参”页，保留覆盖规划、转场 TEB、Hybrid A* 三组参数；修改经后端确认后保存，
  重启优先读取保存值，写入失败不误报成功。
- Qt 选区与覆盖后端统一拟合最小面积外接矩形，保留原导航、视觉、AI 和覆盖修复。
- 原有其他页面与分栏保留，按导航来源将覆盖基础参数迁至调参页；视觉页保留顶部三个端口按钮。

三方合并只在 `AGENTS.md`、`README.md` 和 GUI `CMakeLists.txt` 产生冲突。
文档保留两边历史并增加本目录说明；构建同时纳入 `surround_integration.cpp` 和
`navigation_tuning_page.cpp`。自动合并后的 MainWindow 已核对两种功能共存。
合并版增加独立启动入口与只读视频桥；窗口标题区分三路实景与导航离线调参。
事件日志中的参数保存提示改称“当前后端”。原界面部分说明仍沿用 J6M 词语，
本次隔离入口实际只写本目录预览配置，不能据此认定板端已更新。

## 当前 Qt 的运行范围

```text
J6M 三相机端口 0/1/2 → PYM NV12 → GDC → 原 BPU 检测与分类
  → 已有相机服务与 SSH 转发（AGX 127.0.0.1:18781）
  → 仅 GET 的 Unix socket 视频桥 → 隔离空间 127.0.0.1:18782 → 合并版 Qt

合并版 Qt 导航调参 → 私有 ROS 11543 → 新编译 CoverageGlobalPlanner / coverage_manager
  → 本目录 coverage.yaml、hybrid_tuning.json、transit_tuning.json
```

Qt、规划和参数后端运行在只有 `lo` 的独立网络/PID 命名空间，地图、TF 和零速 odom 是
合成预览数据。没有车辆 ROS 网络、CAN 或实车导航接入。该空间只挂载显示所需 GPU 节点，
不提供串口、相机、BPU 等设备；真实视频桥只允许 `GET /state`、`GET /frame/0/1/2`，
保持报文、模型身份、源端口、序号及年龄原值；Qt 仍执行原时效和图像结果配对校验。

**当前真实视频依赖已运行的 `j6m-b-surround-operator.service`。** 本次没有另启一套相机或
BPU 进程，也没有部署新的板端 release。合并版服务只拥有自己的 Qt、隔离后端和视频桥；
关闭合并版不会停止来源服务。关闭来源 Qt/服务后，其原回收逻辑会停止相机，合并版将显示
等待视频，不能把已经保存的历史帧继续当作实时画面。

本次入口不是独立的实车整栈部署入口。来源的 `optimized.sh`、`surround/operator_qt.sh`
保留原路径保护，不能绕过后在新目录执行。后续部署须将本分支的相机生命周期、导航服务
和板端发布路径配套适配。标定模板、`production_ready=false`、`motion_eligible=false`
保持原状；本次不是 BEV、目标地图坐标、检测精度或实车导航验收。

## 启动与参数位置

```bash
cd /home/slam/robot_j6m_ws_conbine
bash scripts/conbine.sh start
bash scripts/conbine.sh status
# 只关闭本合并版：
bash scripts/conbine.sh stop
```

服务名 `autolabor-conbine-preview.service`；Qt 可执行文件
`devel/lib/autolabor_operator_gui/autolabor_operator_gui_node`。
编译与测试入口分别为 `bash scripts/conbine.sh build`、`bash scripts/conbine.sh test`；
真实参数后端保存/重启自测为 `bash scripts/navigation_tuning_preview.sh test`。

相机桥与生命周期日志：`runtime/conbine_preview/`。
Qt、私有 ROS、后端日志及截图：`runtime/navigation_tuning_preview/`。
独立参数文件：

- `runtime/navigation_tuning_preview/coverage.yaml`
- `runtime/navigation_tuning_preview/ros-home/autolabor_navigation/hybrid_tuning.json`
- `runtime/navigation_tuning_preview/ros-home/autolabor_navigation/transit_tuning.json`
- `runtime/navigation_tuning_preview/qt-config/Autolabor/Autolabor Operator Console.conf`

这四个文件从导航来源当前保存值逐字节复制并核验哈希，之后只修改新副本。
导入记录为 `runtime/conbine_merge_20261023/tuning_settings_import.json`。
重启测试使用单独的 validation 目录，不覆盖这些用户参数。

构建只读复用 `/opt/ros/noetic`、`/home/slam/robot_ws/.deps` 和稳定工作区的生成消息依赖，
GUI、覆盖管理器与 CoverageGlobalPlanner 来自本目录的独立 Release 构建。
相机 LUT、镜头参数、HBM、执行库及 SDK 继续使用原相机会话已校验资产；位置及 SHA 见
[环视来源交接](team/handoffs/B/20261023_surround_operator_qt.txt)。没有升级全局依赖。

## 验证记录

`validation/conbine_merge_20261023/` 保存构建、测试与现场显示证据：

- Release 构建通过，Qt 与覆盖插件均在本目录编译。
- 339 项独立叶子测试全部通过，含 227 项覆盖/规划与 112 项 Qt 检查。
  Catkin 的 412 为含嵌套套件及 rostest 包装的汇总，不计作独立测试数。
- 10 项原视频协议检查、7 项新只读转发测试全部通过；覆盖跨端口、身份/年龄保持、
  源失联不复用旧帧、拒绝控制方法/未知路径和超长/截断报文。
- 7 项实际隔离后端的参数保存、失败拒绝及进程重启检查全部通过，见 `persistence.log`。
- 新 Qt 实例检查验证三路选择与导航调参共存，切换端口不会改变当前导航页签。
- `runtime_isolation.json` 为运行进程、网络命名空间和可见设备实测；
  `window_identity.json` 核对实际窗口与本目录 Qt 可执行文件身份。
- `qt_port*.png`、`qt_coverage.png`、`qt_teb.png`、`qt_hybrid.png` 用于逐页显示复核；
  不按截图文件名推断测试成功，以图中实际按钮和来源状态为准。
- 首次地图页面触发 RViz 时，隔离空间缺少 NVIDIA 图形设备路径，GLX 初始化失败后 Qt 退出。
  原始日志保留在 `first_gui_glx_failure/`。补齐 `/dev/nvidia0`、`/dev/nvidiactl`、
  `/dev/nvidia-modeset` 和 `/dev/nvgpu` 后，在同等隔离空间确认 NVIDIA OpenGL 4.6 可用；
  保留车辆设备隔离。Qt 或视频读取器异常退出现在也会向会话入口报告失败。
- 将预览区域库根目录绑定到同一合成地图的 `map_set`，沿用原地图身份校验，
  消除从导航来源继承的预览配置路径不匹配；没有降低真实地图或选区校验。
- 最终重新打开后，清扫页和综合页 RViz 均实际显示合成地图，区域库正常载入；
  三个视频按钮逐一核对图像、按钮高亮和端口状态一致，三组调参页正常显示。
  `final_qt.log` 不含前述渲染/区域库错误。停止本合并版后的 `owned_stop_check.json`
  确认私有视频桥与 socket 已回收，来源三相机会话身份不变且持续新鲜；随后已重新打开。

来源完整性记录为 `runtime/conbine_merge_20261023/source_preservation_check.json`。
原两个工作目录继续保留各自分支与全部未提交改动。
