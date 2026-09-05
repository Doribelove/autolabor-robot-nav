# detect_and_classify 实现与验证记录（2026-09-02）

> 2026-09-04 更新：经操作员明确要求，后续版本已增加兼容
> `/fod/detections` 的运动输出。它只发布稳定材质分类，并要求当前 RGB 源帧具备同 frame、
> 同尺寸、时间容差内的注册深度；两阶段置信度取较小值。下文的 recognition-only 描述和
> 验证结果保留为 2026-09-02 初始版本的历史记录。detect-and-classify 后端结果门限、
> 检测输入、后端发布、Qt 源帧过期显示和视觉伺服新鲜度门限按操作员
> 要求统一为 0.50 秒。

## 范围与安全边界

新增 Qt 显示名 `detect and classify`、内部标识 `detect_and_classify` 的第三个视觉
后端。测试只启动过隔离的本机 ROS master、ZED、视觉节点和离屏 Qt；没有启动 J6M
控制栈、CAN、`move_base`、底盘或清扫机构，也没有部署、提交、推送或刷写 J6M。
活动 `config/dual_host.env` 在测试前后均保持 `NVIDIA_FOD_BACKEND=locateanything`。

新后端在实车验收前固定 `motion_eligible=false`。为保持 J6M 已部署消息的 MD5 和既有
两个后端兼容，新增独立 UI 消息 `/fod/vision/results`；新后端在旧
`/fod/detections` 上只发布空数组和不受旧控制器支持的 task，控制链 fail-closed。

## 实际架构与模型契约

- 现有后端内部标识为 `yolo` 和 `locateanything`；Qt 切换脚本执行完整托管冷重启，
  同时只存在一个 `/fod_detector`，不会把三个后端一起放入 Orin 显存。
- ZED 应用话题：`/fod_camera/image_raw`、`/fod_camera/depth_registered`、
  `/fod_camera/camera_info`。
- ZED 原生注册点云：`/zed2/zed_node/point_cloud/cloud_registered`。
- RGB/depth 光学 frame：`zed2_left_camera_optical_frame`；新后端只查询 RGB 源
  时间戳的 `map`，失败后查询同一时间戳的 `odom`。
- 生产 ZED launch 当前 `publish_tf=false`、`publish_map_tf=false`，且物理安装外参
  尚未测量验收。因此实景目标世界坐标允许为 `N/A`，不得使用测试 TF 代替。
- 自定义 Ultralytics：`/home/slam/yolo11/yolo11_GAM/ultralytics/__init__.py`，
  版本 8.4.7。
- detector：`/home/slam/yolo11/detect_classify/detect/trash_yolo11s_gam/best.pt`，
  SHA256 `711b6bb4b4debebcf993f033f23e7e641a02dd279254779f8dafed11b6a79233`，
  task `detect`，类别严格为 `{0: trash}`，含 1 个 GAM 层。
- classifier：`/home/slam/yolo11/detect_classify/classify/material_yolo11s_cls/best.pt`，
  SHA256 `d0cce9310e184e8acd7a6142face16d39aadc9a6e5405b18694346f2315899e9`，
  task `classify`，顺序严格为
  `metal, plastic, paper, glass, kitchen_waste`，输出 5 维概率。
- 同一进程先加载/冒烟测试 classifier，再加载 detector；两者各加载一次、预热后
  常驻，退出时先 join 推理线程再释放模型。没有启用双 worker 回退。

Jetson 运行环境：Orin，compute capability 8.7，CUDA toolkit/PyTorch CUDA 11.4，
PyTorch `2.0.0+nv23.05`，cuDNN 8600，OpenCV 4.10.0，NumPy 1.24.4，
`lap` 0.5.13。查询时 CUDA 统一内存总量 61.31 GiB、空闲 46.52 GiB。

## 运行路径

相机回调只把 ROS Image 引用、采集时间和 frame ID 放入容量为 1 的覆盖式最新帧槽。
独立线程执行：ground ROI（默认关闭）→ BoT-SORT/GMC 单类检测 → 内存 20% context
crop → 清晰度门 → 最多 8 个 crop 的分类 batch → object_id 概率投票 → 同源时间戳
深度/CameraInfo/TF → 结构化结果。实时路径没有 JPG 写入。

深度由注册深度和相机内参重建框内局部三维候选，过滤 NaN/Inf/范围外点和离群值，
按深度一致性与像素连通性分簇，再以中心/底边一致性、覆盖率、紧致度和离散度选簇；
评分不使用绝对“最近”距离。平面无法分离、样本稀疏和贴地纸屑无法可靠分离时无效。

object 表保存 object/track ID、世界位置、bbox、时间、最多 5 次概率、稳定材质、状态
和深度有效性。一对一匈牙利代价以世界距离为主，结合时间、尺寸、外观和可用分类概率；
严格的短时外观/IoU 条件只用于源时刻 TF 暂缺时的 track break，不会让当前深度或世界
坐标变为有效。`CONFIRMED` 必须满足至少 3 次稳定投票。

## 验证结果

### 构建与自动测试

- `./scripts/build_workspace.sh` Release 全工作区编译通过，包括新消息 C++/Python 生成
  和 Qt 链接。
- catkin 汇总：840 tests，0 errors，0 failures，0 skipped。
- 两阶段新增核心/契约测试初验为 19/19；本次缓存优化后扩展为 25/25。Qt 契约：
  48/48；双机/切换契约：30/30。
- `git diff --check`、Python compile、bash 语法、launch XML 均通过。
- `health_check.sh --static` 通过；只读检查发现当前配置存在原有运动授权标记，但托管
  服务为 inactive，本次没有启动或使用该授权。
- 三种 `switch_fod_backend.sh --check-only` 均通过，配置文件未改变。

### 300 帧实景 ZED

ZED 2 序列号 23748636 在 5000M USB 3.0 上打开。采集到 300 个
`/fod/vision/results`：

| 指标 | 结果 |
| --- | ---: |
| 实际结果 FPS | 7.53 |
| detector P50 / P95 | 63.43 / 68.63 ms |
| 总处理 P50 / P95 | 65.71 / 71.79 ms |
| RGB 采集到发布年龄 P50 / P95 | 176.75 / 208.69 ms |
| RGB / depth / CameraInfo 时间差 | 300/300 为 0 ms |
| RGB、depth 发布率 | 14.91 Hz / 14.91 Hz |
| 原生注册点云发布率 | 9.89 Hz |
| backend/payload backend 错误 | 0 / 0 |
| 推理错误 | 0 |

CameraInfo 在 ZED alias 上约 44.14 Hz（同一节点的多个相机信息发布路径），但被选中的
300 帧全部与 RGB 精确同 stamp。结果源年龄最大 251.27 ms；该次测试使用 300 ms 结果
门限，后续按操作员要求将检测输入、后端、Qt 显示和视觉伺服门限统一调整为 500 ms。

镜头场景没有被 detector 判定为垃圾，因此该组 300 帧的 classifier 阶段为 0 ms、目标
数为 0；这不能作为实景分类成功依据。自节点启动以来的最终累计计数为 received 1976、
processed 996、覆盖丢帧 11、150 ms 输入年龄门丢弃 777。槽容量始终为 1，没有队列
积压；严格输入年龄门会舍弃 ZED 管线已经偏老的帧，但输出 FPS 仍高于配置 minimum 5。

300 帧末尾的进程内 PyTorch CUDA allocator：当前 allocated 95.72 MiB、reserved
358.00 MiB，生命周期峰值 allocated 235.19 MiB、reserved 358.00 MiB。这是 PyTorch
进程分配器指标，不冒充整个 Jetson 的独占显存。

### 分类阶段补充基准

使用检测验证集 `glass111.jpg`，模型只加载一次，对同一内存 crop 做 50 次独立 batch=1
基准（这不是生产投票循环）：classifier P50/P95 15.07/15.94 ms，detector
P50/P95 44.04/46.90 ms，detect+classify P50/P95 60.19/62.64 ms。第一次 BoT-SORT
初始化造成总耗时最大 392.68 ms；预热后的分类均为 glass，分类置信度中位数 1.00。

### 完整检测、投票、深度和重关联补测

由于实景没有目标，另以真实验证图 `glass111.jpg` 作为 RGB，并只在测试进程中发布同
stamp 的合成注册深度与测试 `map -> test_camera_optical` TF。该数据不代表实车距离或
外参：

- 真实权重输出 trash 检测 D=0.9414、材质 glass C=1.00；
- 有效簇深度中位数 1.50 m，结果世界 frame 为 `map`，同步差 0 ms；
- 首段 track 1 / object 1；64 个空白推理结果后重现为 track 2（首帧还出现 fallback
  track 1000001），object 始终为 1；
- 重关联后 classifier 调用为 0，证明稳定概率历史继承到 object_id；
- 随后连续 24 个带目标但深度全 NaN 的结果全部 depth/world 无效，没有沿用前段 1.50 m；
- 新 backend 结构化结果 backend_id 全部正确，旧控制话题检测数组始终为空。

### object_id 深度/TF 缓存优化补测

随后按现场操作要求把深度和 TF 的高开销步骤改为 object 级有界采样：5 个有效深度
聚类值经 MAD 剔除异常值后取内点平均；锁定后默认每 12 个推理帧复核一次。TF 在同一
RGB 源帧只查询一次并供全部框共享，每个 object 收集 3 个有效世界点后停止继续查询；
连续 10 个需要 TF 的源帧失败后退避 2 秒再重试。缓存全部绑定 `object_id`，track ID
变化时随重关联继承。未执行新鲜复核的帧仍显示 `depth:N/A`，不把缓存相机深度伪装成
当前帧测量。

隔离本机 master 上只启动 ZED 和新视觉节点，现场画面稳定检测到两个目标（模型输出为
metal 与 paper；本记录不把模型类别当成人工真值）。运行快照如下：

| 指标 | 结果 |
| --- | ---: |
| 已处理结果帧 | 1472 |
| 活动 / 深度锁定 object | 2 / 2 |
| 实际逐框深度聚类 | 254 次 |
| 因稳定缓存跳过逐框聚类 | 2692 次（约 91.4%） |
| 36 帧抽样中的新鲜深度帧 | 3（严格每 12 帧一次） |
| 两目标新鲜深度示例 | 0.807 m / 0.837 m |
| 最近 300 帧 detector P50 / P95 | 45.892 / 53.865 ms |
| 最近 300 帧总处理 P50 / P95 | 48.421 / 58.489 ms |
| 采集到发布年龄 P50 / P95 | 154.116 / 197.737 ms |
| 诊断快照实际 FPS | 8.226 |
| 推理错误 / 深度同步缺失 | 0 / 0 |

生产配置没有可用的源时刻 `map/odom -> ZED optical` TF，节点正确保持世界坐标无效。
该阶段累计 91 个失败查询帧，触发 9 次退避；每个查询帧即使有两个框也只进行一轮 TF
查询，不再按框重复。随后仅在隔离 master 中临时发布明确标记为测试用途的 identity
`map -> zed2_left_camera_optical_frame` 静态 TF：总共 3 次成功源帧查询同时为两个
object 生成 6 个世界样本并让二者锁定。继续运行 177 帧后，TF 查询帧和成功数仍保持
`94 / 3` 不变，证明锁定后没有继续查询。测试 TF 随后关闭，整个 ZED/视觉 launch 正常
退出，日志确认推理线程已 join、两模型已释放；无 ROS/相机/TF 残留进程。

### 原后端与 Qt

- 原 YOLO11-GAM 在隔离 master 冷启动成功，固定自定义运行库、best6 哈希和五类正确；
  `/fod/vision/results` 只有旧结果适配器一个发布者，停止后无残留进程。
- LocateAnything-3B 冷启动成功，固定模型根/manifest，长期 worker 只创建一次，仍为
  `motion_eligible=false`；停止后 worker 无残留。
- 新后端两次启动/停止均记录“推理线程已 join、两模型已释放”。
- Qt Release 可执行程序以 `configured_vision_backend=detect_and_classify` 离屏启动，
  正确订阅 `/fod_camera/image_raw` 与 `/fod/vision/results`，并正常退出。
- 没有在真实托管双机栈中点击执行三次切换，因为安全脚本要求新鲜的停车/覆盖/
  move_base 状态，而本次明确不启动车辆栈。已完成三种 check-only、三后端隔离冷启动
  （新后端含实景）以及 Qt 切换契约测试，不把它们写成实车切换验收。

## 配置、话题与使用入口

- 主配置：`src/application/autolabor_fod_vision/config/detect_and_classify.yaml`；
  BoT-SORT 配置：`config/botsort_detect_and_classify.yaml`。两者均位于原 FOD 配置体系内，
  没有新增第二套全局配置。
- 输入：`/fod_camera/image_raw`、`/fod_camera/depth_registered`、
  `/fod_camera/camera_info`，以及源时间戳的 `/tf`、`/tf_static`。已查明但不直接订阅的
  ZED 注册点云为 `/zed2/zed_node/point_cloud/cloud_registered`；节点从注册深度和内参在
  框内重建局部三维点并聚类。
- UI 结构化输出：`/fod/vision/results`；Qt 原图输入仍为
  `/fod_camera/image_raw`。诊断与可选调试图为 `/diagnostics`、`/fod/debug/image`。
  兼容输出 `/fod/detections` 对本后端固定为空，不向旧运动链提供目标。
- 不改配置的安全契约检查：
  `./scripts/switch_fod_backend.sh --backend detect_and_classify --check-only`。
- 现场正常启动仍使用 `./scripts/start_dual_host.sh --start`，不附加视觉运动授权；待 Qt
  就绪、车辆静止且切换安全门状态新鲜后，在视觉页右侧选择 `detect and classify` 并点击
  “应用选择并完整冷重启”。切换脚本会停止旧后端、原子更新配置并冷启动新后端；不要绕过
  该脚本直接并行启动多个 detector。

## 本功能相关文件

- 消息：`src/application/autolabor_fod_msgs/msg/FodVisionDetection.msg`、
  `FodVisionDetectionArray.msg` 及该包 `CMakeLists.txt`。
- 两阶段实现：`src/application/autolabor_fod_vision/scripts/fod_detect_and_classify_node.py`、
  `scripts/fod_vision_result_adapter_node.py`、`src/autolabor_fod_vision/two_stage.py`、
  `src/autolabor_fod_vision/two_stage_runtime.py`。
- 配置与启动：该包的 `config/detect_and_classify.yaml`、
  `config/botsort_detect_and_classify.yaml`、`launch/perception.launch`、
  `launch/zed_fod_detection.launch`、`CMakeLists.txt`、`requirements-yolo.txt`。
- 模型切换/预检：工作区 `scripts/switch_fod_backend.sh`、`scripts/load_config.sh`、
  `scripts/nvidia_ui.sh`、`scripts/health_check.sh`、`config/dual_host.env.example`。
- Qt：`src/application/autolabor_operator_gui/include/autolabor_operator_gui/main_window.h`、
  `src/main_window.cpp`、该包测试与 README。
- 测试与文档：FOD 包的 `test/test_two_stage.py`、
  `test/test_two_stage_runtime_contract.py`，Qt/双机契约测试，以及本验证记录、顶层 README
  和 `docs/ARCHITECTURE.md`。工作树中原有的其他未提交改动均予以保留。

## 已知限制与后续现场项

1. 测量并验收真实 `base_link` 到 ZED 光学 frame 的安装外参，以及源时刻
   `map/odom -> camera` TF 链；完成前实景世界坐标可为 `N/A`。
2. 在真实地面放置五种材质样本，保持车辆静止，补做每类检测/分类、纸屑深度拒绝、
   多目标一对一关联和 Qt 目视验收。
3. 在完整但禁止运动的托管双机栈中，由操作员实际点击三种模型反复切换并核对界面；
   安全门通过前不应绕过脚本。
4. 300 帧实景中的 expired 计数较高，主要来源是当时使用的 150 ms 输入年龄门和 ZED
   管线延迟；检测输入、后端发布、Qt 显示和视觉伺服门限现统一为 500 ms，后续仍应优化 ZED
   发布/线程调度并重新测量。
5. 新后端不得接入车辆运动控制，直到以上现场项目完成并获得显式实车验收。
