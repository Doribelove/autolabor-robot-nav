# 0914：J6M 板端续接与 BPU 执行层适配

本次已经通过网线访问 J6M，在新的私有目录编译、部署并实际调用 BPU。
完成的是通用执行层、单模型通信桥和正式模型的 CPU 输入输出边界；
正式 YOLO11-GAM 检测与五材质分类仍没有 INT8/HBM，正式后端仍在 AGX。
本记录接续 [上一阶段 PTQ 准备记录](J6M_BPU_MIGRATION_20261010.md)，
上一阶段“无法连接、没有板端部署”描述的是当时状态。

开发工作树仍为 `/home/slam/robot_j6m_ws_navigation_20260907`、分支 `0914`，
HEAD `5d1028efa93cf34d2179b03384085c87a456402a`。已有未提交修改是保护基线，
本次新增代码集中于 `distribution/j6m_bpu_navigation`，未提交或推送。

## 网线连接与当前硬件

- 本次实际连线是 AGX 内置 `eth0`，MAC `4C:BB:47:4C:29:6A`、1 Gb/s carrier。
  原保存的 `autolabor-agx-mid360` profile 仍为 `192.168.1.50/24`。
- 在该接口仅增加运行时地址 `192.168.10.50/24`，恢复到 J6M `192.168.10.100`
  的 ping 和已有密钥 SSH；SSH 是网线上运行的登录协议。
  没有保存新的 NetworkManager profile，也没有改变 Wi-Fi 默认路由。
  该临时地址可能在重启、重连或 profile 重新应用后消失。
- 配置中绑定身份的 ASIX J6M USB 网卡、WCH MID360 USB 网卡当前均未枚举。
  没有把内置口冒认为专用 USB 网卡，也没有放宽 MAC/USB 身份匹配。
- 因此 `scripts/network_check.sh` 仍返回 1：两条专用以太网角色未解析，MID360
  连通性未通过；板端到 AGX ping 和 Wi-Fi 默认路由检查通过。
  **BPU 实验连接恢复，不代表完整导航双网验收通过。**
- 连接恢复时还核对了 FT4232H interface 02 的 J6M 串口，未更改板端 IP/持久启动配置。
  此后推理测试使用网线。执行现有 `sync_j6m_time.sh` 校正板端运行时系统时间，
  SSH 中点时差约 -2 ms；未刷机、改 RTC 或配置持久时钟服务。

## 板端部署范围与版本

新目录：`/map/robot_j6m_navigation_20260907/bpu_production_20261010`。
所有候选源码、构建依赖副本和生成库均在该目录。既有 FCOS/CenterPoint 的模型、
适配库和私有运行库作为只读依赖；未安装新全局包、挂载 chroot 或覆盖已有 release。

最终验证库：`build_v5/libj6m_tensor.so`。

| 项目 | 已核对值 |
|---|---|
| DNN / HBRT | `3.14.5_(4.9.2 HBRT)`，执行器要求版本精确一致 |
| UCP | 3.14.5 |
| BPU 驱动日志 | 2.2.19 |
| 执行器源码 SHA256 | `e772e116e707dbce2dd420074bec6fe58faa6765178995cfdf2ffe46944c180b` |
| 最终库 SHA256 | `5c99bafb0531fac9f8e84c49c8c2d0a6d8e852e5c2217e62750dce93b4a14e3d` |
| 导航 `current` | `20260914_095005/install`，未切换；映射后的安装目录与 setup 文件存在 |
| 收尾 `/map` 可用空间 | 876,851,200 bytes，约 836 MiB；未删除旧发布 |

现有私有 DNN 头文件与其他版本公开头文件的结构不同，尤其是 tensor stride、
sysMem 和 UCP 调度 ABI。构建工具校验三份私有头文件与三份运行库 SHA，
借用既有 ARM64 gcc9/sysroot 编译，仅在新目录写入编译器依赖副本。
厂商头文件、运行库和 HBM 不进入源码审阅包。

## 新适配层

| 文件 | 作用与边界 |
|---|---|
| `native/tensor_executor.cpp` | 私有 DNN/UCP C 接口，常驻模型/设备内存；按实际 stride 写输入、读输出，轴相关反量化，调度 BPU core 0 |
| `tools/build_tensor_executor.sh` | 限定候选路径，锁定依赖哈希，拒绝覆盖已有库，使用 `-Wall -Wextra -Werror` |
| `tools/native_tensor.py` | 加载前后验证 HBM SHA/模型名；输入 shape/dtype/有限值检查；输出数组由调用者持有 |
| `tools/tensor_wire.py` | 有长度上限、超时的二进制帧；拒绝重复 JSON 键、非法张量、NaN 和截断数据 |
| `tools/tensor_stdio.py` | SSH 管道内单模型常驻 worker；厂商 stdout 转 stderr，复用既有 BPU 独占锁 |
| `tools/tensor_transport.py` | AGX 客户端；严格 SSH 主机验证，回包校验模型/源帧身份、序号、时间戳和帧龄；失败关闭本次 worker |
| `tools/production_io.py` | 复用固定定制 Ultralytics 前处理，单类 YOLO NMS/坐标还原，固定五材质概率顺序 |
| `tools/verify_*.py` / `test/test_tensor_bridge.py` | 实机执行层对照、通信检查、正式 ONNX 输入输出对照及边界测试 |

执行器目前接受未量化 F32 featuremap 或 U8 参考输入，支持 F32 输出与 SCALE
整数输出，最多32个输入/输出 tensor。它**不支持**既有 CenterPoint 的量化 S8/S32
输入和36个输出组合；CenterPoint 本次继续使用原适配器。
张量、模型总内存和 wire payload 都有上限；超时或运行库错误会使会话不可继续使用。
设备任务状态不确定时不释放其缓冲区，交由退出进程后的驱动清理。

所有回包明确标记 `production_ready=false`、`motion_eligible=false`，没有 ROS
控制或检测 topic 发布者。这里传输的是单模型原始 tensor，尚未实现两模型垃圾流水线、
小体积检测结果服务、重连管理或正式 `detect_and_classify` 插件。
不能据此切换正式后端，也不能拿 COCO FCOS 的输出替代垃圾材质类别。

## 实际验证结果

| 验证 | 结果 | 能证明的范围 |
|---|---|---|
| 最终 native 库编译 | 成功，依赖哈希通过 | 当前板端 ABI 下可构建、加载 |
| 新执行器与原 FCOS 适配器对照 | 3组合成 NV12 × 15个输出，最大绝对误差 0；输出所有权检查通过 | BPU 执行、stride 和反量化与已有参考一致 |
| AGX → SSH → J6M BPU → AGX | 3帧往返 100.85 / 86.64 / 85.22 ms；每帧15个有限输出 | 模型/源帧身份与500 ms帧龄检查、通信关闭有效 |
| 上述3帧设备推理部分 | 4.91 / 4.30 / 4.27 ms | 参考 FCOS 的短时测量，非正式垃圾模型性能 |
| 既有 CenterPoint 板端检查 | 1000个合成点、326个 pillar、36个有限输出；网络约45.76 ms | 当前运行库仍能实际执行既有模型 |
| 正式双模型前处理 | 40份检测 + 50份分类训练图，重新处理与90份固定校准张量逐元素一致 | 已选择输入上的 CPU 前处理一致性 |
| 正式 FP32 ONNX 后处理 | 3份检测与原 NMS/scale_boxes 误差0；五类各1份分类输出通过，未二次 softmax | CPU FP32 输入输出边界，未验收 INT8 |
| 软件边界测试 | 22项通过：原 PTQ 9项 + 新桥13项 | 长度/超时/张量、身份/帧龄、SSH注入、类别/形状等边界 |
| 原板端状态保护 | 9项原模型/库/配置/链接哈希一致；测试 worker 均退出 | 候选目录与原导航发布分开，未留下常驻推理进程 |

合成数据没有识别准确率意义；3帧不是持续吞吐或与导航竞争的性能测试。
当前没有 ZED、MID360、CAN 等完整传感器/底盘链路验收，也没有 GPU/BPU 业务 A/B。
通用输出解包比原专用 FCOS 解包慢；正式输入是约6.75 MiB F32 图像，
多 crop 通信与 CPU 后处理成本必须计入总延迟，不能只报告4 ms网络推理。

原始证据在 `runtime/j6m_bpu_board_20261010/`：
`build_v5.log`、`reference_compare_v5.{log,json}`、`tensor_round_trip_v5.{log,json}`、
`centerpoint_smoke.{log,json}`、`production_io.{log,json}`、
`distribution_tests_final.log`、`board_preservation.json`、`time_sync.log`、
`network_state.json` 和 `network_check.log`。
公开源码包包含 [数值与证据哈希摘要](../distribution/j6m_bpu_navigation/board_validation_summary.json)，
原始现场数据和厂商依赖保持私有。

首次构建因警告被 `-Werror` 拒绝，修正后使用新 build 目录；未覆盖旧库。
首次 FP32 检查缺 ONNX Runtime，随后只读使用已有私有 ARM64 包，没有全局安装。
实际 FP32 sigmoid 产生约 `-5.96e-8` 的零附近舍入，现仅接受 `1e-6` 以内误差并裁剪至
概率区间；真实越界仍拒绝。原失败日志保留，最终通过记录单独命名。

## 正式垃圾模型仍需完成

1. 提供可访问的 **x86_64 OpenExplorer SDK/容器**，记录精确 OE/HBDK 版本及镜像标识。
   已有私有交接包包含两份固定 ONNX 与90份初步训练校准张量。本机与 J6M 都为 ARM64，
   当前没有可用 `hb_compile`。官方也提供 CPU 工具链容器，不要求为了 PTQ 使用 GPU；
   具体版本须与板端私有 HBRT 4.9.2 配套确认，不能直接用最新编译器推断兼容。
2. 编译两模型并审查 GAM 的 MatMul/Softmax 算子落点、CPU fallback 与量化报告。
   HBM 的实际 tensor 名称、dtype、layout、stride、scale 若改变，须明确适配，
   不能伪造原 FP32 契约使接口检查通过。
3. 使用独立标注集验收垃圾召回/误检、五材质混淆矩阵、小目标与量化误差，
   同输入对齐 HBM golden outputs；当前90份校准样本不等于独立准确率测试集。
4. 接入常驻两模型、最多8个 context crop，保留原 BoT-SORT/投票、源帧深度配对
   20 ms容差、500 ms帧龄限制及所有权。先实现观察后端和断连/延迟测试。
5. 完整传感器接入后，测量同场景 GPU/CPU/BPU 与导航竞争，再决定正式后端切换。
   CenterPoint 到 TEB 的原禁止条件继续保留，需独立 MID360 精度及控制接口验收。

本次没有启停完整导航、发送目标/速度、修改运动授权、刷机或切换正式 GPU 后端。
AGX 导航 service 收尾为 inactive / MainPID 0；该状态不代表实车导航已通过。
项目源码包可供地平线审阅，尚未核实 Labs 投稿接口或账号，未对外上传。

官方依据（查阅日期 2026-10-10）：

- [OpenExplorer 开发环境前提](https://doc.oe.horizon.auto/3.8.1/guide/env_install/pre-installation_preparation.html)
- [SDK 安装与 J6 CPU 容器](https://doc.oe.horizon.auto/3.8.1/guide/env_install/software_installation.html)
- [DNN/UCP 板端部署接口](https://doc.oe.horizon.auto/3.8.1/guide/model_deployment/board_deployment/runtime_dev.html)
- [用户指定的 Labs 平台](https://oe.horizon.auto/labs)
