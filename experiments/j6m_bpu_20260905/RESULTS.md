# J6M BPU 迁移阶段结果 — 2026-09-05

状态：**完成隔离 BPU 实机试跑和原模型 ONNX 导出验证；尚未完成生产迁移，未测得 NVIDIA GPU 节省量。**
用户已确认“完全静止”。本轮没有打开相机、启动导航、发布 ROS 检测/控制消息或发送运动目标。
19:27 收尾检查：原版和候选服务均 inactive / MainPID=0，J6M 11311 无监听，
BPU 无用户、利用率为 0，实验推理进程已退出。之前相机近距离遮挡、轮速脉冲来源和定位质量问题
仍未查清；确认静止不代表这些问题已经修复，不自动重启完整栈。

## 本轮新增内容与隔离

- 本机实验目录：`/home/slam/robot_j6m_ws_optimized_20260905/experiments/j6m_bpu_20260905`。
- J6M 实验目录：`/map/robot_j6m_optimized_20260905/bpu_lab_20260905`。
- 新增模型头检查、CPU ONNX 导出/数值对照、NV12 输入准备、离线 FCOS 解码及其测试。
- 官方 FCOS 权重、现有权重的导出副本、运行库、ONNX Runtime、输出和日志均在实验目录。
  NVIDIA 命令经 `scripts/optimized.sh run`；原项目与共享 Python 环境保持只读。
- 没有修改生产节点、launch、配置、后端选择、系统库、驱动、固件、服务或调频。
  没有刷板、安装编译工具链或修改 shell 启动配置。
- 两端实验目录均未发现软链接；远端实验根路径 `realpath` 与预期相同。
  本机读取原权重、验证图像和 Ultralytics 源码属于只读依赖，不把日志或导出写回这些位置。
- 原项目源码/脚本/配置/部署/manifest/地图的 **1287 个文件及链接无变化**。
  J6M 原配置、管理脚本、地图和发布链接与前次清单逐字节一致。
- J6M 原发布仍为 `20260905_122153`，候选发布仍为 `20260905_160700`。
  实验模型未放进任一导航 release，默认仍为 `detect_and_classify`。

证据：`logs/original_integrity_before.json`、`logs/original_integrity_after.json`、
`logs/remote_original_after.txt`、`logs/model_hashes_final.txt`、
`logs/local_symlinks_final.txt`、`logs/final_state_retry1.log`。
原项目完整性清单覆盖上述区域，并不等于逐字节审计整台主机。

## 已验证的 BPU 路径

参考模型来自 [OpenExplorer FCOS EfficientNet-B3](https://huggingface.co/OpenExplorer/fcos_efficientnetb3)。
这是 NASH_M / J6M 的 80 类通用检测模型，不能直接替代现有单类垃圾检测与五材质分类。
模型 SHA-256：

```text
1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa
```

模型头记录 HBDK `4.11.11`。板端原示例库为 `4.0.20.post0.dev202404300720+12b98ed`；
实际试跑使用从已有 SDK 复制的项目私有 HBRT `4.9.2` / DNN `3.14.5`，
原 `/usr/hobot/lib` 和 BPU 驱动 `2.2.19` 均未改。
私有库来源：
`/home/slam/horizen-ch/Acore/test/qualitytest/source/complex_scene_test/hrt_model_exec/j6p/`。
目录名含 j6p，但模型头确认目标是 NASH_M，且已在当前 J6M 真实执行。
不同编译/运行版本组合通过了以下有限测试，**不等于确认完整兼容性或长期稳定性**；
量化工具链就绪后仍需做 golden-output 比对和版本配套验证。

实际 HBM 接口不是模型卡中的一个 BCHW RGB 张量：

| 项目 | 实测接口 |
|---|---|
| 输入 Y | uint8，`1×896×896×1`，802816 bytes |
| 输入 UV | uint8，`1×448×448×2`，401408 bytes |
| 显式 stride | `802816,896,1,1;401408,896,2,1` |
| 输出 | 15 个带 SCALE 量化信息的 int32 NCHW 张量 |
| 输出顺序 | 5 层 80 类分类、5 层 4 维 LTRB、5 层 centerness |
| 特征尺寸 | `112、56、28、14、7` |

`hrt_model_exec` 反量化并去除 padding 后，15 个输出均能按预期尺寸读取为有限 float32。
内核 BPU task_time 也记录到执行任务，不是仅检查了模型头或做 CPU 模拟。

### 计时：仅离线工具基准，不能作为迁移前后 A/B

| 测试 | 结果 | 测量范围 |
|---|---:|---|
| bus 示例单次 infer | 46.750 ms | 工具报告的本次推理；另有 387.048 ms 模型加载 |
| bus 同输入 50 次 perf | 平均 3.919 ms | 单核、单线程；总帧延迟 195.964 ms |
| 上述 perf 工具吞吐 | 253.722 FPS | 不含相机、网络、前后处理、分类和深度关联 |
| 已保存的 ZED 图像单次 infer | 43.901 ms | 另有 389.887 ms 模型加载；不是实时视频 |

这里直接保留工具原始统计；没有独立记录逐帧分位数和工具内部热身过程，
也没有完成长时间负载/内存评测。不能把 253 FPS 称为机器人端到端帧率，
不能用它与原 YOLO 的不同模型/不同链路计时计算加速倍数。

离线查看使用明确标注的演示前后处理：等比缩放、左上对齐黑色 padding、
OpenCV BT.601 limited-range NV12；分数为 sigmoid(class)×sigmoid(centerness)，
LTRB×stride，按类 NMS，阈值 0.2。尚未核对 HEAL 的完整实现和训练预处理，
不得把这个演示解码器作为生产精度基准。

- bus 示例返回公交车和行人等合理框，也有低置信的重复框/可疑标签。
- 已保存的 `640×360` ZED 画面被近距离白色物体大幅占据，返回了
  `toilet 0.346 / sink 0.240 / book 0.208` 等演示标签。没有有效垃圾标注，
  不能据此评价业务精度；更不能把通用类别改名为材质类别来接管生产。
- 没有实时相机订阅、跨机流式通信或深度匹配测试。

证据：`logs/j6m_runtime_probe.log`、`logs/j6m_fcos_model_info.log`、
`logs/j6m_fcos_bus_infer.log`、`logs/j6m_fcos_perf_50.log`、
`logs/j6m_fcos_zed_saved_infer.log`、`results/fcos_bus/`、`results/fcos_zed_saved/`。
两组结果均保存原始输出、解码 JSON、标注图和输入来源 SHA。

## 原业务模型：导出与浮点数值一致性

没有改变网络或删除 GAM 层。两份原权重的 SHA 在导出前后和收尾时一致。
导出固定 batch=1、opset=13、FP32、无 NMS、无 simplify、无动态维度。

| 导出目录 | 输入 NCHW | 输出 | CPU FP32 对照 |
|---|---|---|---|
| `exports/detector/` | `1×3×1024×1024` | `1×5×21504` | 5 张验证图通过 |
| `exports/detector_zed/` | `1×3×576×1024` | `1×5×12096` | 同 5 张验证图通过 |
| `exports/classifier/` | `1×3×224×224` | `1×5` | 每材质 4 张，共 20 张通过 |

`detector_zed` 保留当前单张 `640×360` ZED 图像在 `imgsz=1024`、rect letterbox 下的
实际输入几何；方形导出是备用检查产物，不会静默把生产输入改成正方形。
分类类别顺序仍为 `metal, plastic, paper, glass, kitchen_waste`。

- 三份 ONNX 均通过完整 checker，无自定义 ONNX 域；检测保留 1 个 GAM 层。
- 30 项逐输入对照全部通过。检测框最大差值约 `0.00104 px`，
  检测分数最大差值约 `1.97e-6`；20 个分类 top-1 与原模型全部一致，
  分类概率最大差值约 `4.77e-7`。
- 这是**同一预处理张量下原 PyTorch FP32 与 ONNX FP32 的一致性**，
  不是分类准确率 100%，不是完整数据集 mAP，也不是生产 CUDA FP16、INT8 或 BPU 精度验收。
- 尚未验证分类 batch=8、跟踪投票、重新分类频率、生产帧龄/深度配对。
- 标准 ONNX 算子不保证全 BPU 执行；GAM 中的 MatMul/Softmax 等必须经过目标芯片
  checker、量化及编译报告验证，出现 CPU fallback 时需实测其开销。

数值对照使用项目私有 ONNX Runtime `1.16.3`，仅 CPUExecutionProvider，
Torch/ORT 计算线程各为 2，未初始化 CUDA，30 项共 36.59 s。
首次方形导出被 Ultralytics 内部设置改为 8 线程，约 9.75 s；已在**导出进程内**修正为 2，
后续导出及对照确认生效，没有改共享 Ultralytics 文件。
首次数值对照因实验脚本 numpy.bool_ 无法 JSON 序列化退出；修复后完整重跑通过，失败日志保留。
6 份 Python 文件语法检查通过，演示解码器 7 项单元测试通过。

证据：各导出目录 `export_manifest.json`、`results/onnx_equivalence.json`、
`logs/onnx_equivalence.log`、`logs/onnx_equivalence_retry1.log`、
`logs/fcos_decoder_tests_final.log`、`logs/python_syntax.log`。
export_manifest 中的 numerical_equivalence_tested=false 是导出时点记录，
之后的验证以独立 `onnx_equivalence.json` 为准；BPU 支持状态仍未知。

## GPU 节省量与剩余工作

此前静止运行快照：NVIDIA 全机 CPU 均值 33.91%、GPU 均值 58.27%，
J6M 全机 CPU 均值 16.25%。详见上层 `STATIONARY_FOLLOWUP_20260905.md`。
这些值包含桌面、ToDesk 和诊断订阅，不是 YOLO 独占负载。
本轮导航保持停止，只有离线原型测试，**不存在同条件迁移后的整机数据**；
CPU、内存、GPU 的优化前后对比暂不能交付，不能把停掉整套导航后的空闲当成迁移收益。
ZED SDK/深度仍需留在 NVIDIA，迁移检测也不意味着 GPU 归零。

当前实质阻塞：已检查的本机 SDK、现有 Python 环境及 Docker 镜像中未找到
可用 OpenExplorer host 量化编译环境。当前两台机器人计算机都是 ARM64；
[官方环境文档](https://doc.oe.horizon.auto/3.2.0/guide/env_install.html) 描述 x86 开发端，
并提供 CPU Docker。可以在独立 x86 开发机完成 PTQ，不必为此升级机器人 CUDA/驱动。
Docker Hub 元数据查询两次遇到连接/TLS 超时，没有拉取镜像或更改网络设置。
需要用户提供可用 x86 开发机的现有连接方式，以及 OpenExplorer 开发包/镜像路径（如有）。

获得环境后的顺序：

1. 在独立编译目录锁定 OE/HBDK 与 NASH_M 版本，运行目标芯片模型检查；
   优先编译原检测+材质模型，不先更换类别或删除 GAM。
2. 从训练集选取有代表性的校准数据；验证/测试集不用于校准。保留原预处理、类别、阈值。
   比较浮点/量化模型的垃圾召回、误检、每材质混淆矩阵和小目标表现。
3. 比较量化中间模型与板端 HBM golden outputs，检查 BPU/CPU 算子分配、
   内存、持续延迟、温度和与导航竞争的影响。参考
   [官方 PTQ 上板流程](https://doc.oe.horizon.auto/3.2.0/guide/faststart/ptq_quickstart.html)。
4. 通过后才实现生产跨机适配：限频最新 RGB 帧、保留原始时间戳/序号、返回小体积框和类别；
   深度缓冲及配对、跟踪/分类投票留在原链路。禁止跨机传完整深度/点云，禁止陈旧结果复用。
5. 现场输入问题排清后，完全静止做同场景、同帧率、同界面负载 A/B；
   测断连、延迟超限、重连和单后端所有权，再决定默认切换。运动不在本轮授权内。

## 复现入口与回退

本实验还没有生产 BPU 导航启动入口；不能给 `NVIDIA_FOD_BACKEND` 填一个尚不存在的后端。
无运动离线检查（不打开相机、不接触 CAN）：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
./scripts/optimized.sh run env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /home/slam/robot_ws/.venv/fod_yolo/bin/python3 -m unittest discover \
  -s experiments/j6m_bpu_20260905/tools -p 'test_*.py'
```

重做板端短基准前，先确认没有其他 BPU 任务，且不与导航负载测试混跑。
这是已有输入的 50 次一次性测试，结束自动退出；日志追加至本实验独立文件：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
./scripts/optimized.sh run bash -c '
set -euo pipefail
ssh -o BatchMode=yes -o ConnectTimeout=5 root@192.168.10.100 bash -s <<"REMOTE" \
  2>&1 | tee -a experiments/j6m_bpu_20260905/logs/manual_perf.log
set -euo pipefail
lab=/map/robot_j6m_optimized_20260905/bpu_lab_20260905
test "$(realpath -e "$lab")" = "$lab"
test -z "$(ss -H -ltn sport = :11311)"
if pgrep -x hrt_model_exec; then exit 10; fi
cat /sys/devices/system/bpu/users
cd "$lab"
export LD_LIBRARY_PATH="$lab/vendor/runtime_4_9_2/lib:/usr/hobot/lib"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
nice -n 10 timeout --signal=TERM --kill-after=5s 20s \
  vendor/runtime_4_9_2/bin/hrt_model_exec perf \
  --model_file "$lab/models/fcos_nash_m/model.hbm" \
  --input_file "$lab/inputs/bus/y.bin,$lab/inputs/bus/uv.bin" \
  --input_stride="802816,896,1,1;401408,896,2,1" \
  --core_id=1 --thread_num=1 --frame_count=50
REMOTE
'
```

导出、输入准备和解码脚本拒绝覆盖已有结果；保留本批证据，不删除结果来强行重跑。
推理库仅由上述进程级 LD_LIBRARY_PATH 引入，无常驻服务，无需系统回滚。
原版和候选发布均未被本实验切换，也不需要反向复制任何文件。
恢复导航应遵守上层 [启动与切回说明](../../OPTIMIZATION_REPORT.md#启动测试与切回原版)
及其最新安全停止状态；不能为“回退”而直接启动原版的运动配置。

模型仓库的 LICENSE、LICENSE.zh 与 NOTICE 已保留并复制至远端实验模型目录。
LICENSE.en 下载遇到超时；不影响已保留的完整中文许可文本，未擅自改成其他许可证。
参考权重只用于内部实验，不对外发布本目录中的模型和厂商运行库。
