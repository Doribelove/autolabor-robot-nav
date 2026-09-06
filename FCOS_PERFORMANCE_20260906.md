# FCOS 常驻推理性能优化（2026-09-06）

## 结果与当前状态

已更新 NVIDIA 候选 Qt/桥接和 J6M 私有 BPU worker，并完成真实相机测试。
旧桥强制 **0.5 Hz**，新桥默认上限 **30 Hz**：15 Hz 相机实测 **14.90 Hz**；
30 Hz 独立 RGB 相机实测 **28.22 Hz**，结果源帧年龄平均 **84.57 ms**、P95 **101.37 ms**。
保存图像的跨机连续基准为 **35.01 次/秒**，不是实时相机或完整导航的帧率。

本次开始时，原版和候选导航服务已经 inactive，J6M 刚重启、旧 BPU 进程不存在。
本次只运行隔离相机/Qt 测试（本机 master `127.0.0.1:11571`），未启动导航、CAN、
定位或深度链，未发送初始位姿、目标、非零速度或恢复旧目标。12:39 交付核对：
测试 Qt、相机、桥和远端 worker 均退出，BPU ratio=0、users 为空；两个导航服务仍 inactive。
不存在可据此宣称的当前 LOCALIZED 或运行态 watchdog 授权状态。

保留 `MOTION_ENABLED=true`、`FOD_MOTION_ENABLED=false`、最终线速上限 1.60 m/s 和既有
`runtime/motion_authorized.ok`，没有改动任何运动保护。历史已经完成的现场初值不应重复索要；
本轮没有启动需要初值的新导航会话。

## 原因和实施

1. 旧 `qt_bpu_bridge.py` 每次发送后固定等待至 `sent + 2.0`，最多 0.5 Hz。
   现采用最新帧事件唤醒，按预处理开始时间限频，默认 30 Hz，可配置 0.5–30 Hz。
   保留一帧在途，不积压、不重复识别旧帧。
2. 旧 worker 每帧启动 `hrt_model_exec`、加载模型、写输入、导出并读取 15 个浮点张量。
   新增 C++ 常驻会话直接调用现有私有 DNN/UCP API，一次加载并复用输入/输出内存，
   做正确的 cache clean/invalidate、输出反量化和 padding 去除。稳态不落盘张量。
3. 解码先筛除不可能达到原阈值的背景，再计算精确 score；框坐标计算和逐类贪心 NMS
   向量化。模型、输入尺寸、原阈值、稳定排序、类别、NMS 语义均保留。
4. NVIDIA 复用 OpenCV/NV12 缓冲。新增 FC03 传输只省略已逐字节验证为常量的黑色
   letterbox 区域，J6M 原样重建；不是有损压缩，也没有降低模型分辨率。
   640×360 输入时每帧传输从 1,204,248 减为 677,400 字节（含包头），减少 43.75%。
   非标准 padding 或无收益时自动用兼容的 FC01 全量协议。
5. Qt 图像单独以 33 ms 定时刷新；其余导航诊断仍为原 250 ms，不把全部诊断提升到视频频率。
   状态显示实际完成频率、上限、往返时间和源帧年龄。截图存盘改为每 5 秒，日志继续有界轮转。

`hbDNNInferV2`、任务提交/等待和缓存操作根据本机厂商头文件实现，使用现有私有 runtime，
没有升级系统库、写 sysfs、改变频率或刷写固件。接口资料参见
[D-Robotics 模型推理接口](https://toolchain.d-robotics.cc/guide/ucp/runtime/bpu_sdk_api/function_interface/model_inference.html)。

保持模型 SHA256：`1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa`。
冷启动模型加载仍需约 **392 ms**，首帧较慢；不能把稳态 85 ms 当作相机/Qt/模型冷启动总时长。

## 性能证据

以下测量均在同一对 NVIDIA/J6M 上完成。相机测试为 depth=NONE 的独立预览，
没有运行正式 YOLO、深度或完整导航负载。

| 测量 | 原 CLI / 旧桥 | 新常驻实现 |
| --- | --- | --- |
| 桥的固定频率上限 | 0.5 Hz（每 2 秒一次） | 默认 30 Hz，可调 |
| 保存图像、不限频跨机基准 | 0.80 次/秒，10 帧 | 35.01 次/秒，300 帧 |
| 基准稳态跨机往返均值 | 1,241.72 ms | 26.44 ms，P95 32.64 ms |
| 基准 worker 耗时均值 | 1,213.85 ms | 11.82 ms |
| 基准 vendor infer 均值 | 45.61 ms（逐帧冷进程） | 4.31 ms（常驻预热后） |
| 15 Hz 相机，60 秒 | 旧桥最多 0.5 Hz | 14.90 Hz，源帧年龄均值 113.60 ms |
| 30 Hz 相机，60 秒 | 未做同场景旧链对照 | 28.22 Hz，源帧年龄均值 84.57 ms |

15 Hz 行是在本轮常驻引擎的较早优化阶段实测；30 Hz 行使用最终裁边传输、缓存预处理和
NMS 优化。30 Hz 相机实际输入 29.17 Hz，60 秒输出 1,693 个结果，没有重复或乱序帧。
源帧年龄是 NVIDIA 相机消息时间戳到结果订阅回调的时间，不是物理曝光到屏幕、ToDesk 或
导航响应时延。往返时间只覆盖桥发送请求到收到结果；两者不能混用。

60 秒资源采样：J6M 整机 CPU 平均 10.31%，BPU 平均 11.15%、最高 17%；
worker RSS 稳定在 74,304 KiB，可用内存最低 6,403,648 KiB。
保留低优先级和 Python/BLAS 单线程限制；厂商 runtime 自身有后台线程，实测 worker 总线程数 20。
温度仅保存 sysfs 原始值，未验证该平台的单位，不作摄氏温度结论。

**不以 BPU 100% 为优化目标。** 本模型暖态单次约 4.3 ms，30 帧/秒只需约 13% 的
计算时间；实测占用与之相符。当前单相机供帧约 29 Hz，识别约 28 Hz，已经接近新帧输入速率。
重复算同一帧、堆积旧帧或额外跑压力负载只会抬高占用，不能提高现场新帧识别频率。

尝试过 zlib 无损压缩，但实测 CPU 代价使基准降至约 22.8 Hz，未保留该实现；
最终采用无编解码负担的恒定 padding 省略。失败尝试日志保留，不能与最终性能混用。

主要原始记录：

- [原 CLI 基准](experiments/j6m_bpu_20260905/live/bench_20260906_122123_782269/summary.json)
  和 [最终常驻基准](experiments/j6m_bpu_20260905/live/bench_20260906_123254_581097/summary.json)。
- [15 Hz 相机观测](experiments/j6m_bpu_20260905/logs/performance_live_20260906_122642.json)、
  [最终 30 Hz 相机观测](experiments/j6m_bpu_20260905/logs/performance_live_20260906_123502.json)、
  [最终资源采样](experiments/j6m_bpu_20260905/logs/performance_final_resources_20260906.json)。
- [最终 Qt 生命周期验收](experiments/j6m_bpu_20260905/logs/qt_integration_20260906_123610.json)：
  最小化后 7 秒帧计数 773→773、远端 worker 消失；恢复后新 worker 继续。
  停止桥后相机 3 秒仍有 90 帧，随后经测试入口停止整个相机/Qt 会话。
- [Qt 实际显示截图](experiments/j6m_bpu_20260905/logs/qt_integration_20260906_123610_live.png)：
  显示常驻引擎、实际频率及帧龄；测试场景随现场变化，不作为精度数据集。

## 修改文件和部署

实验工具目录为 `experiments/j6m_bpu_20260905/tools/`：

- 新增 `fcos_resident.cpp`、`resident_fcos.py`、`build_resident_j6m.sh`。
- 更新 `bpu_preview_worker.py`、`qt_bpu_bridge.py`、`live_protocol.py`、`decode_fcos_demo.py`、
  `start_qt_bridge.sh`、`qt_preview_session.py`；实验根目录的 `live_camera.launch`、
  `qt_preview.launch` 增加独立测试用的相机帧率参数。
- 新增 `benchmark_fcos.py`、`validate_resident_fcos.py`、`measure_preview.py`、
  `sample_bpu_resources.py`、`test_cropped_protocol.py`；更新 `check_qt_integration.py`、
  `test_fcos_demo.py`、`test_qt_bpu_bridge.py`、`test_preview_lifecycle.py`。
- Qt 修改为 `src/application/autolabor_operator_gui/src/main_window.cpp` 及其
  `include/autolabor_operator_gui/main_window.h` 中的独立预览定时器；候选本机构建已更新。
- 本记录及根/实验 README、AGENTS 和旧持续记录的最新状态索引已更新；不删除历史交接。

J6M 已同步 worker/协议/解码/包装器及所需私有头文件，并在 J6M 原生编译
`/map/robot_j6m_optimized_20260905/bpu_lab_20260905/bin/libfcos_resident.so`。
只读取已有 Ubuntu 编译器/sysroot；缺少的编译器辅助库复制到实验私有 `build/compiler_lib`，
未写导航 chroot 或系统目录。所有运行均经候选 `scripts/optimized.sh run`。

**没有发布导航 release、没有切换生产检测后端。** 候选远端链接
`/map/robot_j6m_optimized_20260905/rootfs/opt/autolabor/dual_host/current` 仍为
`/opt/autolabor/dual_host/releases/20260905_160700/install`。
不要误用原版 `/map/autolabor_runtime/rootfs` 下的另一个 `current` 判断候选部署。

远端常驻库 SHA256：`3f3534566e73f19bf10963131c8cbff206aa7a6f98ff1f65dd45b2b13e4fa8aa`。
[远端部署校验](experiments/j6m_bpu_20260905/logs/performance_deployment_sha256_20260906.log)
与[本机源码校验](experiments/j6m_bpu_20260905/logs/performance_local_sha256_20260906.log)匹配。
修改前实验文件副本在两端实验目录 `performance_before_20260906/`。
原工程 1,287 个文件清单复核未变，见
[原目录完整性记录](experiments/j6m_bpu_20260905/logs/performance_original_integrity_20260906.log)。

## 验证与边界

- [43 项实验单元测试](experiments/j6m_bpu_20260905/logs/performance_unit_tests_final_20260906.log)通过：
  包括限频、协议长度/坏包、奇偶尺寸 NV12 字节一致、预处理缓存和旧解码/NMS 等价性。
- [180 次张量比较](experiments/j6m_bpu_20260905/logs/performance_tensor_equivalence_20260906.log)
  float32 逐元素完全一致（2 个固定输入 × 15 个输出 × 6 次）；最终 300 帧基准逐帧核对旧 golden
  检测类别、置信度和坐标通过。这不是在新数据集上的 mAP 精度验收。
- [Qt 构建](experiments/j6m_bpu_20260905/logs/performance_qt_build_20260906.log)和
  [63 项 Qt 测试](experiments/j6m_bpu_20260905/logs/performance_qt_tests_20260906.log)通过
  （52 项 Python 契约 + 11 项 C++ 区域存储测试）。
- Shell 语法、Python AST、差异空白检查通过；
  [全项目 static 检查](experiments/j6m_bpu_20260905/logs/performance_static_20260906.log)通过。
  static 所显示的 952 项是现存测试 XML 汇总，不能写成此次重新执行全部 952 项。
- 保留独占锁、帧龄/身份验证、一帧在途、I/O/推理超时、EOF 清理、磁盘余量、日志轮转、
  可见性停算、导航伴随会话归属检查、故障退出；不自动重启异常或导航。
- FCOS 仍只是通用物体显示，不替代正式垃圾/五材质后端，不产生运动控制消息。
  未验收完整导航并行的新性能、GPU 节省量、实车导航/制动或新常驻引擎多小时连续运行。
- J6M 本轮重启后墙钟回到 2025-06-26；未为相机实验改时钟。频率/往返使用单调时间，
  相机源帧年龄在 NVIDIA 同机计算，不受该板端墙钟影响。未来完整启动仍需现有时间同步流程。

## 使用和回退

以下命令在候选目录执行。导航伴随入口不会启动导航或相机，要求候选导航/Qt 已通过其
正常入口运行并能验证归属；**本轮没有执行这一步完整启动**。

```bash
./scripts/optimized.sh run bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh --max-fps 30
```

无时长参数默认持续运行；只写 `--max-fps` 也保持持续模式。无参数同样为常驻引擎、上限 30 Hz。
本次**没有修改导航相机的 15 Hz 配置**：该输入下新桥最多处理约 15 个不同新帧/秒。
30 Hz 结果来自独立相机测试，不能声称正常导航相机已被提升到 30 Hz。

只在两个导航栈均停止时，复现独立 30 Hz RGB/Qt 测试：

```bash
./scripts/optimized.sh run bash experiments/j6m_bpu_20260905/tools/start_qt_preview.sh --seconds 180 --camera-fps 30 --max-fps 30
./scripts/optimized.sh run bash experiments/j6m_bpu_20260905/tools/start_qt_preview.sh --stop
```

伴随模式回退到保留的 CLI 引擎：先停止自己的桥并等待其正常退出，再启动低频 CLI；
不删除私有 socket、不杀未知进程，不需要回滚导航 release。

```bash
./scripts/optimized.sh run bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh --stop
# 确认当前桥已退出后：
./scripts/optimized.sh run bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh --engine cli --max-fps 0.5
```

最终停止状态证据：[本机服务](experiments/j6m_bpu_20260905/logs/performance_final_local_state_20260906.log)、
[候选 release / J6M BPU](experiments/j6m_bpu_20260905/logs/performance_final_remote_state_20260906.log)。
