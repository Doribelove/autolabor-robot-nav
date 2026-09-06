# 独立优化候选版交付记录 — 2026-09-05

最新：18:22 静止续测因左轮小幅非零反馈安全中止，双机栈已完整停止，原版也停止。
最终速度指令均为零；相机存在近距离遮挡。新的相机别名修复已本机构建、通过 949 项测试，
未重新启动实机，J6M 发布未变。现场确认前不自动重启。
本轮证据、资源数据和未完成项见 [静止续测记录](STATIONARY_FOLLOWUP_20260905.md)。

17:55 补充：修复了当前 GNOME 桌面侧栏不接收鼠标点击的问题，导航与 ToDesk 进程未重启，
导航窗口已恢复、机器人仍零速。详见 [桌面故障处理记录](DESKTOP_FIX_20260905.md)。

## 结论与当前限制

候选版已建立、完成 NVIDIA 构建和 J6M 原生构建部署，并通过项目测试、隔离测试、
真实数据回放、双机自定义消息通信和静止 ZED 采集测试。
**2026-09-05 16:59 已按用户“不要移动，可以启动导航”的补充要求启动候选完整导航栈。**
MID360 已恢复供电/链路，使用既有网络配置；原版服务保持停止。
候选 `MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`，没有创建或复制运动授权标记，
没有发送导航目标、初始位姿或执行移动测试。避障、急停、失联停车机制未关闭。
17:55 交接时曾保留候选导航和 Qt 运行；18:22 后已经停止，以上方最新续测记录为准。

地图实际显示 `READY`。17:08:57 起系统收到 Qt 操作员提交的初始位姿（不是助手发布），
全局/局部 costmap 和 `move_base/status` 随后开始发布。17:10 前多次 ICP 质量拒绝，
17:11 已能进入 `LOCALIZED`，但最后 30 秒 600 条状态中仍有 39 条 DEGRADED、
41 条 ALIGNING，另外 520 条为 LOCALIZED；末次 RMSE 0.3471 m，接近原设 0.35 m 门限。
没有修改定位算法/质量门限，运动放行继续关闭。所选地图与当前场景/位姿的一致性
仍需现场核对，不能把短时定位成功等同于稳定运行。
**已通过全链路启动和静止数据检查，尚未完成全局定位稳定性、避障效果和实车运动导航验收。**

首阶段 MID360 不在线时只进行了独立端口的回放和相机诊断、未启动车控；下文保留这些
早期基线证据。当前完整导航的实机证据见下一节，不再以早期网络缺失作为阻塞条件。

原版源码、脚本、配置、部署、manifest 和地图的 1287 个文件/链接复核未变；
原 J6M 配置、管理脚本、地图链接及发布链接前后校验一致。未升级依赖、驱动，
未修改系统服务或 NetworkManager 配置，未刷写固件，未提交/推送 Git。

## 补充：完整导航静止实测（17:05–17:12）

正常隔离入口启动成功，`check --static` 和 `check --runtime` 均通过。
完整栈包含真实 MID360、双 LD19、CAN/M2、ZED、两阶段视觉、FAST-LIO、地图、
定位门、move_base、覆盖管理、速度仲裁和 Qt。J6M master 的 `/proc/PID/root` 属于候选
chroot，`current` 仍为候选 `20260905_160700`；原服务 inactive。

`scripts/observe_stationary_navigation.py` 仅订阅，连续观测 30.10 秒，不发布指令或目标：
以下主表为 17:05 尚未设置初始位姿时的窗口。

| 实测项 | 结果 |
|---|---|
| MID360 点云 / IMU | 300 / 5992 条，10.00 / 200.01 Hz |
| FAST-LIO 里程计 / 点数 | 300 / 299 条，均约 10.00 Hz；里程计分量有限 |
| 前 / 后 LD19 | 299 / 300 帧，9.97 / 10.00 Hz |
| 融合 `/scan` | 299 帧，10.00 Hz；`mid360+dual_ld19`；1440 格中每帧至少 305 个有效距离；最大消息时间戳年龄约 0.113 秒 |
| ZED RGB / 深度、FOD 输出 | 445 / 446 / 240 条，14.85 / 14.86 / 7.98 Hz；只验证消息连续，不代表深度或分类精度 |
| 最终 `/cmd_vel` | 1478 条，49.30 Hz，全部六个速度分量为零；唯一发布者 watchdog |
| 上游 `/cmd_vel_safe` | 600 条，20.00 Hz，全部零速；唯一发布者 FOD 仲裁 |
| 左 / 右轮实测反馈 | 1256 / 1223 条，最大绝对轮速均为 0；底盘 `/odom` 位置也保持零 |
| 底盘状态 | 持续回报；四个急停状态位均 false；没有新增 control_timeout / controller_monitor 事件。未触发急停或失联测试 |
| GUI 和任务状态 | 地图 `READY`（1311×670，0.1 m/格）；coverage inactive；定位等待操作员初始位姿 |

17:08:57 后 Qt 操作员提交并调整了初始位姿：已测得全局 costmap 1311×670、
局部 costmap 200×200，分辨率均为 0.1 m；`move_base/status` 约 5 Hz，目标列表为空。
额外 30 秒观测中最终速度指令 1458 条、左右轮反馈 1252/1229 条，仍全部为零。
末次 ICP 为 overlap 0.9620、inliers 3522、RMSE 0.3595，超过 0.35 门限而处于 LOST；
之后调整初始位姿仍有质量拒绝。检查确认候选定位 C++ 和 YAML 与原版逐字节一致。
已保存 `stationary_navigation_after_pose.json`、`stationary_after_pose_state.log`；
被动观察报告的空 `faults` 仅表示所采样的数据连续性/零速检查无异常，不代表全局定位通过。

17:11 定位进入 LOCALIZED 后再次独立采样 30 秒：600 条定位状态中 LOCALIZED 520 条、
DEGRADED 39 条、ALIGNING 41 条，末次 overlap 0.9754、RMSE 0.3471 m、inliers 3694。
因此只记录“能定位但质量边缘、有状态波动”，不判定定位稳定性通过。
此窗 1452 条最终速度指令及左右轮 1268/1247 条反馈仍全零，未发生本次测试指令导致的运动。
证据为 `stationary_navigation_localized.json`；文件名表示该窗口从 LOCALIZED 开始，
不意味着窗口内每一帧都是 LOCALIZED。静止时保持全部运动保护，不进一步进行移动或放宽门限。

底盘启动瞬间记录过一次 `ControlTimeout (0x24)`，随后正常零速流建立；采样窗口没有复现。
ZED 初始化仍有自标定失败警告，提示纹理、亮度或近距离物体条件，未复现崩溃。
不能因此开放运动放行，亦不能声称已经验证真实障碍回避或相机深度精度。

静止资源快照（`top` 第二帧，即 10 秒区间；不是原版/新版 A/B）：NVIDIA 全机 CPU
约 28.1%、可用内存 44809 MiB，J6M 全机 CPU 约 11.9%、可用内存 5614 MiB；
两机 swap 使用均为 0。J6M FAST-LIO 为单核口径 16.6% CPU、181 MiB RSS，
显示增强节点 1.2% CPU、12.4 MiB RSS。NVIDIA 相机约 1.0 GiB、视觉进程约 3.4 GiB RSS。
31 个整机 GPU 采样为 11–89%、均值 54.61%；同时包含桌面和远程桌面等负载。
资源采样发生在设置初始位姿之前，当时全局定位/costmap/任务尚未初始化，
不能将这些数值当作运行导航任务的资源上限或优化收益。
17:11 补充的 10 秒窗口中，NVIDIA 全机 CPU 约 30.0%、可用内存 44770 MiB；
J6M 全机 CPU 约 13.5%、可用内存 5584 MiB，swap 仍为 0；记录在
`stationary_navigation_localized_{nvidia,j6m}_top.log`。这仍是静止、有诊断订阅者和桌面负载的快照，
不是原版/新版或运动任务的性能对比。

证据：`validation/stationary_navigation_start.log`、`stationary_static_health.log`、
`stationary_runtime_health.log`、`stationary_navigation_observation.json`、
`stationary_navigation_{nvidia,j6m}_top.log`、`stationary_full_navigation_tegrastats.log`。
本次原版 1287 项复核仍无变化；`remote_original_stationary_after.txt` 与远端原基线完全一致，
候选三个共享基础库挂载仍为只读。新增被动观察脚本和本节记录只在候选中。

## 位置与运行基线

| 项目 | 原版（保留不动） | 候选版 |
|---|---|---|
| NVIDIA 工作区 | `/home/slam/robot_j6m_ws` | `/home/slam/robot_j6m_ws_optimized_20260905` |
| J6M 运行目录 | `/map/autolabor_runtime` | `/map/robot_j6m_optimized_20260905` |
| J6M 发布 | `20260905_122153` | `20260905_160700` |
| 用户级运行单元名 | `autolabor-dual-host.service` | `autolabor-optimized-20260905.service` |
| 主地图 | `map_20260829_221335_gimp_cleaned` | 独立复制，`latest` 为副本内相对链接 |

实测平台：NVIDIA ARM64 Ubuntu 20.04.6、ROS Noetic、12 CPU、约 62.8 GB RAM；
J6M Debian 12 宿主内运行 Ubuntu 20.04/ROS Noetic chroot，6 CPU、6971 MiB RAM。
首阶段停止状态时 J6M 可用内存约 6270 MiB，`/map` 剩余约 5.0 GiB。候选本地目录约 6.4 GiB。
这些为现场快照，不是满负载导航性能。

当前实际视觉后端为 `detect_and_classify`，不是历史 README 中的 LocateAnything。
保留两阶段检测/材质分类、YOLO11-GAM、可选 LocateAnything、ASR、Qt 和 RViz。
ASR 静态检查中 CUDA/Whisper 及三档本地模型 SHA 验证通过，但没有枚举到物理麦克风。

双机网络：NVIDIA eth1 `192.168.10.50` ↔ J6M `192.168.10.100`；
MID360 为 `192.168.1.112`，预期 NVIDIA 专用口 `192.168.1.50`。
J6M 链路已验证双向 ping 和 ROS 动态 TCP 通信。USB 角色解析保留按永久 MAC、USB ID/序列号
识别的现有容错逻辑，没有按旧文档硬编码 eth 编号。CAN、前后 LD19、ZED 设备在 NVIDIA。

## 架构、启动流程和资源归属

```text
NVIDIA MID360 → /gateway/livox/{lidar,imu} → J6M 单份 relay
                                                ├─ FAST-LIO → Odometry/点云
                                                ├─ 已知地图 ICP → map 定位
NVIDIA 双 LD19 ──────────────────────────────────┴─ 避障融合 → 唯一 /scan
J6M map_server → move_base/Navfn/Hybrid A*/TEB → 覆盖管理/定位门/FOD 仲裁
                                                → /cmd_vel_safe
NVIDIA ZED/CUDA/识别 → FOD 消息 ──────────────────┘
NVIDIA 最终 watchdog → /cmd_vel → M2/CAN → 底盘
NVIDIA Qt/RViz ← 地图、诊断、增强显示点云和新增轻量点数
```

正常入口依次：独占/归属检查 → 网络与静态健康检查 → J6M 时钟/模型/地图合同检查与同步
→ J6M master/waiter → NVIDIA 传感器网关和最终 watchdog → J6M 定位导航
→ NVIDIA 相机/识别/Qt → 运行健康检查。完整监督器保留 required 节点退出联动、
有界停止、PID 启动时间验证和孤儿进程归属检查。运行时的原有跨机时钟同步步骤保留；
首阶段仅诊断/回放未执行，16:59 完整启动按既有流程同步了 J6M 时钟。

没有迁移职责或增加 GPU/BPU 推理后端：ZED、CUDA 推理仍在 NVIDIA，定位和导航仍在 J6M。
没有修改 CPU 亲和性、系统调度、功耗模式、驱动、模型精度、ROS 安全超时或避障参数。
现有单份 Livox relay 已避免重复跨机原始输入；未再引入另一份接收链。

## 修改清单（仅候选版）

1. **运行和输出隔离**：新增 `scripts/optimized.sh`，用已安装的 bubblewrap 将根文件系统
   只读挂载，只有候选目录及必要设备入口可写。ROS、Qt/XDG、CUDA、YOLO、临时文件、
   LocateAnything worker 的缓存/日志均重定向到副本。模型和已有依赖仍只读复用。
2. **生命周期隔离**：独立服务名、PID/令牌、日志、部署锁；新启动拒绝占用原 master。
   停止时先验证 J6M master 的 `/proc/PID/root`，只有属于候选 chroot 才取消目标、
   调用暂停服务或清理 ROS 注册。通信失败时跳过共享图修改，仍执行有归属证据的本地停止。
3. **部署工作流**：修复从其他当前目录调用部署时相对 rsync 路径错误；`--help` 和非法参数
   在 SSH/写操作前返回；复用候选专用原生 build cache，每次仍生成新发布、验证后原子切换。
   加入只读基础环境准备，重启后无需改动原 chroot 即可重新挂载候选依赖。
   两阶段实测日志还暴露 Albumentations 导入时的联网版本查询/TLS 超时；通过该已安装版本
   支持的 `NO_ALBUMENTATIONS_UPDATE=1` 跳过非必要更新查询，不改推理、依赖或安全机制。
4. **有证据的工具 bug**：卸载脚本识别真实的 `PID:start_ticks` 记录，拒绝卸载仍有活进程
   的 chroot；回退脚本改用 chroot 内路径并原子替换 `current`，避免生成宿主路径链接。
   远端生命周期脚本拒绝把运行根覆盖为原目录。
5. **点云显示分支**：增强点云不再先整帧复制再清空、逐点插入；按行复制有效数据，
   每帧只构建一次 TF 旋转矩阵。无显示订阅者时跳过输出组装，仍检查时间/TF/布局并发布诊断。
   修复组织点云行重叠、`row_step=0` 等错误布局可能静默复制错误点的情况，保持原报错透传逻辑。
6. **减少跨机无效数据**：FAST-LIO 增加 `/fast_lio/body_point_count`（UInt64）。
   集成 Qt 原来为点数/到达频率接收完整 `/cloud_registered_body`，现在仅订阅计数；
   保留原完整点云发布和独立 GUI 的旧订阅默认值，RViz 的显示数据不变。
7. **最小清理**：移除 FAST-LIO CMake 中完全重复的一行 C++14 编译选项。
   未删除无法证明无用的 GPS、仿真、诊断、硬件适配或恢复逻辑。复制时跳过原构建产物、
   rosbag、旧日志、Git 历史和 Python 缓存，不等于删除原数据。

运动、安全和覆盖算法保持原样。仅候选运动放行配置因用户要求静止改为双 false。
导航配置仍为前进 0.80、倒车 0.30 m/s、角速度 0.60 rad/s；
watchdog 原硬上限仍为 1.70 m/s、1.00 rad/s，失联超时 0.25 s。
**它们不是本轮获批的测试速度**；Qt 还可能保存更高的操作员设定。
实际运动前应依用户确认在副本中约束导航、Qt 和最终硬限速，并重新验收所有安全门。

## 性能对比与解释

测量为单节点基准，不是整机导航 A/B。采用原 bag 的同一帧 6471 点 MID360 点云及
720 格 LD19 扫描，输入 10 Hz 和 100 Hz；有/无订阅者各三轮，新旧交错、每轮 10 秒，
另有热身，保持系统现有频率策略。每端 24 轮。CPU 为单核百分比，内存为每轮峰值 RSS 的中位数。
NVIDIA 基线为优化前源码本地编译产物，J6M 基线为原实际部署二进制；均在候选环境执行。

| 位置/负载 | CPU 原→新（单核 %） | 峰值 RSS 原→新（MiB） |
|---|---:|---:|
| J6M 10 Hz，有查看器 | 0.80 → 0.80 | 16.00 → 15.88 |
| J6M 10 Hz，无查看器 | 0.60 → 0.40 | 15.06 → 14.50 |
| J6M 100 Hz，有查看器 | 6.50 → 5.60 | 16.19 → 16.13 |
| J6M 100 Hz，无查看器 | 4.60 → 2.60 | 15.06 → 14.56 |
| NVIDIA 10 Hz，有查看器 | 2.00 → 2.20 | 14.66 → 12.59 |
| NVIDIA 10 Hz，无查看器 | 1.40 → 0.90 | 12.23 → 13.94 |
| NVIDIA 100 Hz，有查看器 | 17.50 → 16.10 | 12.68 → 12.46 |
| NVIDIA 100 Hz，无查看器 | 9.80 → 4.30 | 12.61 → 11.84 |

J6M 100 Hz 无查看器的 CPU 中位数下降约 43.5%，但这是十倍压力输入。
生产 10 Hz 有查看器时没有可宣称的 CPU 收益；NVIDIA 对应测试甚至有约 0.20 个百分点上浮。
内存差异很小且有波动，不能声称整机内存明显降低。显示增强节点生产归属是 J6M，
NVIDIA 一列用于本地回归及交叉检查。

所有有订阅者的轮次接收数量均与发送数量相同；每轮抽取的输出帧在清除时间/序列号后
新旧序列化 SHA256 一致。无查看器诊断、重新订阅及异常布局另由 ROS 测试覆盖。

原 440 秒实车 bag 测得 `/cloud_registered_body` 约 10.002 Hz，前五帧平均序列化
大小 310419 B，据此估算旧 Qt 诊断订阅约 **3.10 MB/s** 纯消息载荷；新计数仅
8 B/帧，约 80 B/s。此为取样估算，不是网卡实测带宽，也不包含 RViz 仍需的增强点云流。
FAST-LIO 内部点云生成并未取消，因此不宣称相应计算量也被消除。

GPU/加速器未做参数优化。真实 ZED 采集时保存了整机 tegrastats 和相机进程采样，
相机 RSS 峰值约 1052 MiB；该轮还包含初始化、静态依赖检查和桌面后台负载，
不能据整机 CPU/GPU 曲线归因或给出导航前后收益。原生增量部署一次耗时 105 秒，
没有同条件的原部署计时，不能据此给出部署加速倍数。

## 验证记录

| 验证 | 结果/证据 |
|---|---|
| NVIDIA 23 包 Release 构建 | 成功；`validation/build_candidate.log` |
| J6M 原生构建、安装依赖、launch/MD5 | 成功；`validation/deploy_final.log`、`j6m_health_final.log` |
| 项目测试 | 945 项，0 错误/失败/跳过；`all_tests_final.log`、`affected_tests_final.log` |
| 隔离/运行资源/停止归属/参数/回退/PID 回归 | 10 项通过；`isolation_tests_final.log` |
| FAST-LIO 真实原始数据回放 | 15 秒、147 帧点云、147 组里程计；新旧点数一致，里程计分量最大差 0；新 147 条计数逐帧匹配；`fastlio_replay.json` |
| Qt 运行订阅检查 | 同一 GUI 旧模式订阅完整点云，新模式只订阅计数，均存活；禁用的仅为测试 RViz 渲染，不是避障；`stationary_io.json` |
| 真实 ZED USB/图像 | USB 5000M；RGB 和深度各 231 帧、640×360、约 14.97 Hz，连续至少 15 秒；有自标定警告，不能判定深度精度合格；`stationary_io.json`、`stationary_zed.log` |
| 真实相机+当前两阶段视觉 | 28 秒测试中输出 108 条 FOD 消息，检测/分类各加载一次，任务类型 detect/classify；`stationary_vision_io.json`。另有相机自标定警告，见下方限制；消息输出不代表识别准确性或运动有效性 |
| 双机自定义消息往返 | 独立 11476 master，FOD 数组 NVIDIA→J6M、Livox CustomMsg 返回，5/5；RTT 中位数 0.934 ms；`cross_host_client.json` |
| 原版完整性 | 1287 项无变化；远端前后清单相同；`original_integrity_final.json`、`remote_original_before/after.txt` |
| 回退链接 | 对候选当前版本执行原子重指向后安装态检查通过；未切原发布；`rollback_link_validation.log` |
| 首阶段静态/网络检查 | 当时无运动标记、MID360 未在线；保留 `local_health_final.log`、`network_check_final.log` 作为历史证据 |
| 本次完整静止导航 | 双运动放行 false；静态/运行检查通过，MID360+双 LD19+FAST-LIO+视觉+底盘反馈持续、输出零速；costmap 已发布、定位可进入 LOCALIZED 但有状态波动。稳定性和运动验收未完成 |

测试代码及 JSON/日志均在 `scripts/` 和 `validation/`。第一次全量测试中的两项入口合同
失败已保留原日志，修正为检查候选入口与不写网络配置后重跑通过，没有删除失败记录或放宽安全测试。
`cloud_before.json` 只是早期基准脚本的“旧对旧”校准，不应作为优化后结果。

## 未修复问题与尚未验证场景

- 历史日志包含 ZED `-11` 崩溃；更早记录有 `-6` 退出并联动关闭整栈。
  本轮短时相机采集未复现；没有证据确定原因，未修改 required/退出联动。
- 两轮相机短测及本次完整静止运行都记录了 ZED 自标定失败警告（画面纹理/亮度/近物条件提示），
  但仍输出图像和识别消息。候选保持零速放行关闭；不能把有数据等同于深度精度或传感器健康。
  需现场检查遮挡、照明、镜头/标定状态和实物深度误差后才能用于导航/FOD 运动。
- 覆盖任务在扫掠异常终态时，先返回 blocked、未复核有向完成，可能重试远处旧入口而触发
  4 m 局部恢复包络拒绝。原文档、日志和状态机支持该问题，但本轮未改变运动决策；
  保留恢复距离、碰撞和曲率保护，后续应先加针对性回放/仿真，再受限实车验证。
- 原 bag 有 TF 查询失败/退避及 CAN 控制超时、急停类消息。未证明它们由 CPU/GPU 争抢导致，
  不能据此改调度或放宽超时。FAST-LIO 极大的历史队列上限也仅列为后续内存峰值测量对象。
- 本次已测真实 MID360/LD19 连续性、唯一 `/scan`、零速链路与轮速反馈；
  已测到初始全局定位成功和质量状态波动；定位稳定性、地图重定位、动态障碍、失联停车、定位失效停车、物理急停、点到点导航、
  覆盖换行/恢复、FOD 运动仍未实测，不因用户只要求静止启动而补做运动测试。
- 静止底盘遥测的 `battery_current` 出现约 4.29e9 的原始值，现代码按 uint32 解码。
  尚未确认协议单位/有符号编码，不将其解释为真实安培值，也未在运行中修改驱动；需独立核对协议。
- 没有进行 YOLO 两阶段/CLIP/LocateAnything 长时间推理、语音采集、GUI+RViz 长时间
  全栈运行、低电量和热稳态性能对比。保留模型与合同，不能把模块测试当作这些场景验收。
- 没有执行整机重启测试；基础只读挂载的准备脚本可重复执行，仍需现场验证冷启动全链路。

## 隔离审计与依赖边界

副本源码无指向原工作区的目录链接。`src/CMakeLists.txt` 指向系统 catkin 文件，ASR Python
链接指向系统解释器，均只读；地图 `latest` 指向副本内目录。原 `.git`、构建树、旧日志/bag
未复制；ASR 模型及虚拟环境、CLIP Python/词表、Qt 用户配置已单独复制，未与原输出目录共用。
CLIP 已在实际 FOD Python 环境中从候选路径导入成功（`clip_runtime_import.log`）。
非必要版本联网查询已通过 mock 网络入口验证为零次调用（`offline_import_check.log`）。

保留的外部只读依赖包括 `/opt/ros/noetic`、`/usr/local/zed`、
`/home/slam/robot_ws/.deps`、`/home/slam/robot_ws/.venv/fod_yolo`、
`/home/slam/yolo11` 的固定权重/运行源和 `/home/slam/LocateAnything` 的固定模型/manifest。
**这是独立项目/运行数据版本，不是可脱离这些依赖的完整系统镜像**。
没有运行安装/升级脚本；若第三方代码尝试写外部绝对路径，隔离层会拒绝，不能解除隔离来绕过。

J6M 候选 `/usr`、`/opt/ros`、`/opt/autolabor/ros` 是原基础库的只读 bind mount，
三处均实测 `ro`。`/etc` 独立复制；`/run`、`/tmp`、ROS_HOME、地图、FAST-LIO 的 Log/PCD、
日志和发布链接都指向候选运行根。不会把完整原 `/opt/autolabor/dual_host` 共享给副本。
设备和 proc/sys 接口用于运行及诊断，不是完整安全容器；不要经 `run` 执行安装、固件或系统管理命令。

本机服务仅在启动候选完整栈时按原机制创建独立的临时用户级单元；本轮没有创建/修改原
系统服务。首阶段卸载了工作负载挂载；本次候选正在运行，候选设备/运行数据挂载和基础
只读挂载均保留，之后通过候选正常 stop 流程收尾，不卸载正在使用的 chroot。

## 启动、测试与切回原版

仅使用本目录的隔离入口，不直接 source 原工作区后混合运行：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
./scripts/optimized.sh build
./scripts/optimized.sh deploy
./scripts/optimized.sh check --static
./scripts/optimized.sh status
```

当前运动放行双 false，`check --static` 可通过，无需也不得伪造运动授权标记。
用户已明确授权静止启动，MID360 已在线；不需要先确认运动区域才能做静止数据检查。
如果以后要实际移动，再确认现场区域/人员/观察员、最高线速度和角速度、可立即操作的急停，
验证定位、传感器/避障、通信并在副本约束所有速度边界后，按既有授权流程重新验收。
不要在本次静止会话中把 `motion_enabled` 改回 true，不要发送导航/覆盖/FOD 运动目标。

静止启动命令如下（当前已启动，不需重复执行）；使用前仍须确认原版已停止且无双重底盘占用：

```bash
/home/slam/robot_j6m_ws_optimized_20260905/scripts/optimized.sh start \
  --map-set /home/slam/robot_j6m_ws_optimized_20260905/global_maps/map_sets/latest
```

`--authorize-fod-motion` 仅适用于另外明确批准该次 FOD 运动；不是默认测试参数。
任何定位丢失、传感器异常、通信中断或非预期运动，应立即使用物理急停并执行有界停止，
保留日志排查；不得关闭避障、急停或失联停车。断联时 SSH/软件停止不是物理急停的替代品。

切回原版：

```bash
/home/slam/robot_j6m_ws_optimized_20260905/scripts/optimized.sh stop
/home/slam/robot_j6m_ws_optimized_20260905/scripts/optimized.sh status
# 核验候选节点、CAN 占用、两机控制进程均已退出，现场确认安全后才启动原版：
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start \
  --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest
```

不要同时运行两套系统。原版 `current` 和原配置没有被替换，不需要把候选文件反向复制回去。
候选停止遇到归属不明进程或失联会报告，不能用广泛 `pkill`/`killall` 清场。
候选内部 `deploy/j6m/rollback.sh` 只切候选发布，不是切回原项目的方法；早期候选发布不含
全部新合同，当前推荐保留 `20260905_160700`，不能把任意历史候选当作可混用的生产版本。

增量部署缓存固定为 `20260905_155827`。当前同步不删除远端源码：以后若删除/迁移文件，
应选择新的 `J6M_BUILD_CACHE_ID` 做干净的独立构建，避免旧文件残留；不要清理原项目或原发布。
新日志位于本机 `log/`、`validation/` 和远端 `/map/robot_j6m_optimized_20260905/logs/`。
