# 双机项目终端交接

更新时间：2026-08-27（Asia/Shanghai）

工作区：`/home/slam/robot_j6m_ws`

这份文档面向下一位现场操作人员。正常情况下只需要一条命令，不要再分别手工启动两台机器。

## 一键启动

先给交换机、J6M 和 MID360 供电，确认 ASIX USB 网卡经扩展坞 RJ45 与 J6M
均接入交换机，MID360 专用 USB 网卡直连 MID360，且三段链路灯均已亮。

在 NVIDIA 主机打开一个终端：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_dual_host.sh
```

启动器会依次完成：

1. 检查并清理不完整的双机残留进程；
2. 检查 NVIDIA↔J6M、NVIDIA↔MID360 两条专网；
3. 同步 J6M 时钟；
4. 启动 J6M ROS Master/waiter；
5. 启动 NVIDIA MID360、CAN/M2 和速度看门狗；
6. 等待 J6M FAST-LIO、避障融合、move_base/TEB、FOD 仲裁；
7. 启动 ZED、YOLO11、Qt 与嵌入式 RViz；
8. 运行最终运行态健康检查；节点、主机归属、参数和 topic 所有权异常仍会阻止启动，
   暂时缺少传感器消息只打印 `WARN` 并让服务保持运行。

看到以下文字才表示完整启动成功：

```text
Dual-host project is ready and managed by autolabor-dual-host.service.
```

此时终端可以直接关闭。完整进程组由用户级 `autolabor-dual-host.service` 托管，不再依赖启动它的终端或图形桌面会话；任一关键进程退出时，服务会同步停止 NVIDIA 与 J6M，避免 FAST-LIO 在 Livox 断流后继续运行。

启动器内部使用 `--runtime --allow-missing-data` 区分“ROS 图已正常建立”和“实时数据
已经到达”。需要严格验收数据链时仍执行 `./scripts/health_check.sh --runtime`；该命令
会把任何缺流项打印为 `FAIL` 并返回非零，但不会自行停止已经运行的服务。

每次启动前，脚本会自动识别并清理由本项目运行令牌或严格工作区特征确认的旧进程，所以正常残留不再要求重启主机。它不会猜测进程归属，也不会为了抢占 CAN 串口杀死无关程序。

## 常用命令

```bash
# 查看完整运行状态
./scripts/start_dual_host.sh --status

# 强制执行一次完整冷重启
./scripts/start_dual_host.sh --restart

# 同步停止两台机器
./scripts/start_dual_host.sh --stop

# 前台诊断模式（终端需保持打开）
./scripts/start_dual_host.sh --foreground
```

也可以使用原关停入口：

```bash
./scripts/stop_dual_host.sh
```

不要只重启 `nvidia_gateway.sh`。Livox 断流而 J6M `/laserMapping` 未同步重置，曾导致 FAST-LIO 位姿发散到数千米。

### 实验性静态地图模式

默认命令仍启动无图 FAST-LIO 基线，不加载历史地图。只有操作员明确选择地图集时才启用
三维已知地图定位加二维导航：

```bash
./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest
```

该模式每次冷启动都停在 `WAITING_INITIAL_POSE`，不会复用旧位姿。MID360 点云和
`/Odometry` 都新鲜后，操作员才可在二维地图上给出车辆真实位置附近的
`/initialpose`；连续两次 ICP 质量检查通过并达到 `LOCALIZED` 后，导航速度门才放行。
定位退化、丢失或状态超时会立即输出零速并取消当前 move_base 目标，重新定位后不会
自动恢复旧目标。map_server 只提供二维地图，实时避障仍使用融合后的 `/scan`，不使用
AMCL。

## 当前机器分工

| 模块 | NVIDIA | J6M |
|---|---:|---:|
| ROS Master |  | 是 |
| MID360 物理驱动 | 是 |  |
| FAST-LIO |  | 是 |
| MID360/LD19 避障融合 |  | 是 |
| move_base + TEB |  | 是 |
| USB-CAN/M2 | 是 |  |
| ZED、CUDA、YOLO11 | 是 |  |
| FOD 速度仲裁 |  | 是 |
| 最终速度看门狗 | 是 |  |
| Qt/RViz、AI/MCP、可切换 Whisper ASR | 是 |  |

主要数据链：

```text
MID360 -> NVIDIA Livox driver -> J6M relay -> FAST-LIO
FAST-LIO -> /Odometry + registered cloud -> /scan -> move_base
Qt/RViz 普通目标 -> /move_base_simple/goal -> /navigation_pause 目标门
                 -> sweeper-simple-* /move_base/goal
Qt AI 显式目标   -> /navigation_goal/action_request -> /navigation_pause
                 -> /move_base/goal（同 ID 回显/状态闭环）
Qt AI 精确取消   -> /navigation_goal/cancel_request -> J6M /move_base/cancel
未转发安全回执   <- /navigation_goal/cancel_ack（只证明该 ID 从未进入 move_base）
Qt/AI 覆盖启动   -> /coverage/start_batch（客户端 operation ID 原样成为 batch ID）
覆盖精确取消     -> /coverage/cancel_batch（旧/foreign ID 不影响当前批次）
ZED -> YOLO11 -> /fod/detections -> J6M FOD arbiter
move_base -> /cmd_vel_navigation -> /cmd_vel_safe
          -> NVIDIA watchdog -> /cmd_vel -> M2/CAN
```

## 当前实机配置

- NVIDIA↔J6M：ASIX USB 网卡经扩展坞 RJ45 和交换机连接 J6M；NVIDIA 为 `192.168.10.50/24`，J6M `eth0=192.168.10.100/24`。
- NVIDIA↔MID360：MID360 专用 USB 网卡为 `192.168.1.50/24`，MID360 为 `192.168.1.112`；USB 网卡接口名以 MAC 识别结果为准。
- Wi-Fi `wlan0` 保持默认路由，机器人网口不接管互联网路由。
- CAN：`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0`，已确认。
- ZED 2：序列号 `23748636`，彩色图、深度和 YOLO 检测实测约 `15 Hz`。
- MID360 点云、`/Odometry`、`/scan` 实测约 `10 Hz`，IMU 约 `200 Hz`。

接口名称可能在重启后变化；启动器会按下列永久 MAC 自动找回接口并修正 NetworkManager
配置。MAC 未命中时还会核对配置中的 USB VID:PID + serial，只有唯一精确匹配才会接受：

```text
6C:1F:F7:C4:96:B8  ASIX 千兆网卡         -> 扩展坞 RJ45/交换机/J6M
50:54:7B:E3:C9:10  MID360 专用 USB 网卡  -> MID360
```

J6M 串口控制台当前是 FT4232H 第 3 路，波特率 `921600`。诊断时使用稳定的
`/dev/serial/by-path/...` 路径，不依赖可能变化的 `ttyUSBn`：

```bash
screen /dev/serial/by-path/platform-3610000.xhci-usb-0:4.1.1:1.2-port0 921600
```

退出 screen：先按 `Ctrl-A`，再按 `K`，最后输入 `y`。

网络异常时先运行：

```bash
./scripts/network_check.sh
ip -br address
nmcli -t -f NAME,TYPE,DEVICE connection show
```

若启动器明确报告 `MID360 USB Ethernet adapter is not connected`，说明 MAC 为
`50:54:7B:E3:C9:10` 的物理适配器尚未被系统枚举；重新插接或供电后直接再次执行一键启动，不需要重启主机。

需要查看托管启动日志时：

```bash
journalctl _SYSTEMD_USER_UNIT=autolabor-dual-host.service -n 120 --no-pager
./scripts/stop_dual_host.sh --list-orphans
```

## Qt 验收标准

Qt 标题应为“Autolabor 无人车操作与诊断台”，并能看到：

- 顶部 ROS、CAN、FAST-LIO、点云、IMU、避障雷达、导航、控制模式、相机、
  YOLO11 和录包状态卡；
- “综合、FAST-LIO、测试、视觉、清扫、AI语音控制、日志”全部页签；
- “AI语音控制”页的语音输入、AI 语义解析、AI 控制三门和智能语音模式每次启动都默认为
  关闭；既保留“开始录音/停止并识别”的手动模式，也可另行确认后启用持续监听、VAD 自动
  断句模式；页面显示 ASR 模型/设备/阶段、监听状态、句数、待处理数、录音时长、识别耗时、
  识别文本、云端往返和逐步计划；
- AI 语义解析关闭时 ASR 文本只能本地显示，AI 控制关闭时只允许计划预览；手工文本
  不需要语音授权，但同样不能绕过解析和控制授权；
- “清扫”页具有独立全局地图、车辆、计划/实走轨迹和侧栏状态；无 `--map-set` 时
  框定、参数和开始按钮置灰；静态图模式可多点框定、撤销、随时取消和确认闭合；
- 规划成功后可用中文或英文命名并“保存为已知清扫区”；相同实际地图再次启动时可从
  “选择已保存区域 / 管理队列”载入、排队、移除或删除，所有有状态操作都有确认框；
- 区域库保存在对应 `map_sets/<map-set>/coverage_regions/<source_mode>/regions.json`，按
  规范化 map-set 目录严格隔离并用 J6M 实际 `/map` 的完整 SHA-256 复核；只保存多边形
  定义，不保存 plan ID、轨迹或自动运动状态；地图摘要或 MapDisplay 未就绪时相关按钮
  必须禁用；
- 多区域队列由 J6M 一次接收并逐区即时规划；完成/部分完成继续，失败停止，“跳过当前
  区域”和“取消整批”语义分开，批次换区间 `/coverage/active` 仍保持任务所有权；
- 综合和清扫地图都用亮绿色 Polygon 显示 move_base 实时发布的车辆安全轮廓；当前
  基础 footprint 为 `1.04 × 0.70 m`，含 `0.10 m` padding 后约为 `1.24 × 0.90 m`，
  并随 `base_link` 移动；静态地图缺少初始位姿或新鲜里程计/TF 时暂不显示是正常状态；
- 清扫地图另外显示 `/Odometry` 最近 `120` 个有效位姿采样，侧栏显示当前全局/里程计
  位姿及最近 `10 s` 样本数、累计距离和年龄；有效清扫宽度默认
  `1.00 m`，重叠 `15%` 时车道中心距 `0.85 m`；速度输入默认 `0.80 m/s`、覆盖任务最大
  `1.60 m/s`，白色侧栏/工具栏区域必须使用黑色文字；
- 清扫“规划参数”还包含倒车速度、最大转弯角速度、线/角加速度及换向/分段附加时间；
  每次修改立即写入 NVIDIA 当前用户的 Qt `QSettings`，下次启动沿用，但不写进地图区域库；
  全覆盖排序以覆盖完整度为硬优先级，再用静态图 A* 距离和上述运动约束构造秒级时间，
  对候选角度运行受限 beam search 联合选择首线、后续顺序和方向。TEB 原始混合权重 cost
  不是秒，不直接参与离线排序；执行仍由 Navfn + 在线 TEB 最终裁决；
- 清扫侧栏分别显示覆盖导航状态、去重后的覆盖面积估算、路线宽度/间距/最小半径、
  VCU/TEB 在线运动学核对、MID360 + 前后 LD19 障碍感知状态和“清扫机构未接入”，
  不得把 `SWEEPING` 当作刷盘已运行；
- 覆盖任务活动时目标门拒绝普通 `/move_base_simple/goal`，Qt 相对目标、普通取消和清扫页
  `SetGoal` 都不能抢占覆盖管理器的分段 action goal；
- 中央嵌入式 RViz 网格、MID360 点云、动态 aligned scan 和避障扫描；
- 路线图例明确区分青色覆盖条带、蓝色全局参考路线、红色当前局部轨迹、绿色覆盖执行
  记录；没有活动 move_base 目标时蓝/红路线必须清空；
- 静态地图冷启动时，综合页 RViz 自动以 `map` 为视角目标显示完整二维图，并显示
  “① 显示整张地图 / ② 设置初始位姿”；第二个按钮只选择 `SetInitialPose` 工具，
  仍必须由操作员在车辆真实位置按住并沿车头方向拖动；
- ICP 达到 `LOCALIZED` 后应自动进入“③ 跟随车辆”视角，也可手动点击；Fixed Frame
  保持 `map`，Target Frame 为 `base_link`，局部代价地图随车居中；
- 同一控制条的“④ 显示静态三维先验”默认关闭；点击后按需显示
  `/fast_lio_localization/prior_map` 并切换为可旋转视角，再次点击或进入初始位姿工具
  会恢复二维全图；
- 右侧 ZED/YOLO11 实时画面；
- 局部坐标、速度、FAST-LIO 健康分及事件日志；
- 综合页 `Δ前向 / Δ左向 / ΔYaw` 相对目标输入。未授权运动时不要点击发送。

GPS/WGS84 目标页、GNSS/航向卡和 RabbitMQ 远程页已经从本室内版本完整移除，
对应桥接进程也不再随任何启动器运行。

FAST-LIO 健康度读法：

- `健康（≥85）`：里程计/点云约 10 Hz、IMU 约 200 Hz，数据新鲜，TF 连通，
  位姿连续且静止漂移正常；
- `注意（65–84）`：频率、延迟、协方差、单帧跳变或漂移有一项开始偏离；
- `异常（<65 或关键链路故障）`：停止下发新目标，按页面“判定依据”逐项检查；
- 静止漂移要等待车辆持续静止 5 秒后才有效；运动中显示“采集中”是正常的；
- 内部协方差不是绝对真值误差，最终精度仍要用已知点或闭环复位测试。

录包卡默认灰色“未录制”，需要时由操作员手动开始。

## AI 语音与 ASR

ASR 使用 NVIDIA 本机 OpenAI Whisper `small / medium / large`，默认 `medium`，其中
`large` 对应 `large-v3`；原始音频不上传云端。首次准备在
NVIDIA 执行：

```bash
cd /home/slam/robot_j6m_ws
bash ./scripts/install_whisper_asr.sh
```

安装器创建 `runtime/asr/venv`，固定 Whisper commit，并将校验通过的三套 checkpoint
原子落到 `runtime/asr/models/{small.pt,medium.pt,large-v3.pt}`。运行时不允许下载。
实际配置位于权限必须为
`0600` 的 `src/sweeper_mcp/config/sweeper_mcp.yaml`；其中 `input_device: auto`（空值
等价）会只读枚举实体 ALSA capture 端点并优先选择 USB 麦克风，也可显式填写稳定设备
标识。设备枚举不打开麦克风，实际采集仍受 Qt 语音授权控制。

自动发现会排除 Jetson APE 和 Pulse monitor/null 端点，并优先选择实体 USB capture。
当前已验证 AB17X USB Audio 可被只读枚举为稳定 ALSA card ID `Audio`；连接存在时会选择
`plughw:CARD=Audio,DEV=0`，枚举过程不打开设备。麦克风拔出时 ASR fail-closed 为
`UNAVAILABLE`，不会猜用 APE、monitor 或临时 card 编号。

确认语音授权后有两种互斥流程：手动模式仍为“开始录音 → 停止并识别”；智能模式还需点击
“启用智能语音”并确认，之后持续监听并在约 `0.8 s` 静音后自动形成一句、使用当前所选模型
识别。它不输出边说边更新的 partial token；每句需完成断句和本地批量推理后才可能请求云端。
解析门关闭时文字只在 Qt 本地显示；解析门开启后智能句子进入有界 FIFO 串行处理；控制门
关闭仍只展示计划。授权变化会清理未提交旧句，关闭智能模式、撤销语音授权、Qt 心跳超时或
worker 异常都会停止采集、清空队列并丢弃旧 session/capture 的迟到结果。ASR worker 没有
ROS、云端密钥或 MCP 控制能力，任何模式都不能绕过项目原有安全门。

本链的源码、模型、CUDA venv、Qt 和 AI 节点全部属于 NVIDIA。只修改 ASR/AI/Qt 时执行
本机构建和静态检查，不运行 `deploy_j6m.sh`，也不切换 J6M `current`；只有另行修改 J6M
包、J6M launch/config 或两端共享消息时才走 J6M release 流程。

AI 的地图坐标导航不再把锁存 `/map` 当作两秒动态话题。NVIDIA MCP 后端长期缓存一次有效
OccupancyGrid，使用与 J6M 覆盖管理器相同的完整地图摘要确认身份；每个任意 x/y/yaw 目标
仍须通过新鲜 `LOCALIZED`、新鲜 CoverageStatus、摘要一致、地图原点旋转、范围和占用检查，
且发布前复核地图未切换。“地图坐标原点”固定为 `(0,0,0°)`；其他未保存名称不得由模型
猜测坐标。

AI 不再从 `/move_base/goal` 的“最近目标”猜 GoalID。每次导航由 NVIDIA 生成唯一
`sweeper-ai-<uuid>`，J6M 桥校验并按本机时间转发；只有同 ID 的 action 回显、唯一状态和
完整目标位姿均一致才算接受。闭环缺失或 `LOST` 会进入持续精确撤销，GoalID 在安全终态
确认前不会释放，J6M 的 AI 心跳租约也会在 NVIDIA 后端失联时继续撤销。因此 Qt 中若显示
“撤销确认中”，不能把它理解为目标已经停止，也不能继续发下一条 AI 导航。

AI 覆盖批次在 NVIDIA 后端还有独立提交锁：从首次 CoverageStatus/区域库预检、预分配
`coverage-batch-<uuid>`、调用 `/coverage/start_batch`，一直覆盖到响应丢失后的 exact
tombstone/取消收敛。第二个并发 start 或 cancel 在这段窗口内不会进入 ROS 服务，因此已被
J6M 接受的 A 不会被 B 的拒绝补偿清空 `_ai_batch_id`。J6M 端对失败后保留的 owner 也采用
单事务清理并绑定清理开始时的 move_base generation 和精确 GoalHandle；旧 A 的重复取消、
状态定时器或迟到线程不能操作随后启动的 B。

## 安全状态

当前交付配置为：

```text
MOTION_ENABLED=true
FOD_MOTION_ENABLED=false
```

`runtime/motion_authorized.ok` 已由此前现场确认流程创建并保留，不要在部署、重启或收尾时
静默删除。主运动门为 true 不等于车辆必然执行：定位未完成、传感器缺流、CAN/急停异常、
指令过期或节点所有权异常时，现有链路仍应保持零速。

只有车辆架空、人员远离车轮、实体急停可用，并明确要做低速运动测试时，才允许按项目原安全流程临时授权：

```bash
./scripts/authorize_motion.sh --confirm-elevated-estop
```

此前实测出现过 `/cmd_vel` 非零但左右轮速仍为零。下次运动测试必须优先检查 M2 控制模式、VCU 急停/制动输入与 CAN 下行帧，不能把非零 `/cmd_vel` 当作底盘已执行。

## 前后 LD19 与避障融合

前后物理口已完成确认，当前实际配置为：

```text
DUAL_LIDAR_PORTS_CONFIRMED=true
front = /dev/serial/by-path/platform-3610000.xhci-usb-0:4.4:1.0-port0
rear  = /dev/serial/by-path/platform-3610000.xhci-usb-0:4.3:1.0-port0
```

系统级 `/dev/autolabor/lidar_front`、`lidar_rear` 别名目前没有生成，且本次会话没有
免密 sudo，未重载 udev；运行配置直接使用上述稳定物理路径，因此不依赖易变的
`ttyUSBn`。2026-08-23 实际冷启动验收结果为：

```text
/dual_lidar/front/scan_raw ~= 10 Hz
/dual_lidar/rear/scan_raw  ~= 10 Hz
/dual_lidar/scan           ~= 10 Hz
/mid360/scan               ~= 10 Hz
/scan                      ~= 10 Hz
/avoidance/source_mode = mid360+dual_ld19
/avoidance/dual_lidar_active = true
```

普通导航保留 MID360 单源降级能力；覆盖任务要求 `/scan` 新鲜且前后 LD19 确实参与，
任一条件丢失会拒绝启动/恢复，执行中丢失则暂停并取消当前目标，要求人工恢复。

## Jetson/ZED 注意事项

本次开机后曾出现 `/dev/nvhost-vic` 错误变为 `root:root 0600`，导致 ZED 节点段错误。正确状态应为：

```bash
stat -c '%a %U %G %n' /dev/nvhost-vic
# 期望：660 root video /dev/nvhost-vic
```

一键启动器会在启动 ZED 前检查访问权限并拒绝带故障启动。系统已有 `/etc/udev/rules.d/99-tegra-devices.rules`，其预期权限就是 `root:video 0660`。若重启后复发，需要管理员重新应用 udev 权限并重启 `nvargus-daemon`，再执行完整冷启动。

2026-08-23 又确认了一类独立故障：ZED `2b03:f780/f781` 在 udev 稳定前完成冷枚举，
usbfs 和 hidraw 节点停在 `root:root 0600`，同时视频链路只协商到 `480M`。项目提供：

```bash
./scripts/install_zed_udev.sh       # 首次安装，需管理员密码
./scripts/zed_camera_check.sh --wait 0
```

安装器通过 `autolabor-zed-coldplug.service` 在以后开机首轮 udev 稳定后、
NetworkManager 启动前加载 ASIX/WCH、FTDI、CH341、UVC 驱动并重新应用本车设备规则；
MID360 专用 WCH 网卡在精确 USB ID+序列号匹配且仍无载波时只重置一次。USB 速率
仍必须由正确的 USB 3.x 端口、线缆和插头接触保证。若检查显示 `480M`，翻面重插
相机端 Type-C 或改接原生 USB 3.x 口，直到显示 `5000M` 或更高。ROS 中仅有
`/zed2/zed_node` 不表示相机已打开，必须实际收到 `/fod_camera/image_raw` 和
`/fod_camera/depth_registered`。

补充：本机 ZED SDK 4 已实测在 f780 为 `5000M`、f781 usbfs 可访问但内核未生成
hidraw 时仍会报告 `Camera Available`。因此 hidraw 只做可选诊断，不是启动硬门；
最终仍以 ROS 彩色图和注册深度首帧为准。

## 日志与诊断

每次一键启动会创建：

```text
/home/slam/robot_j6m_ws/log/dual_host_launcher_YYYYMMDD_HHMMSS/
├── j6m_ssh.log
└── nvidia.log
```

NVIDIA 子模块详细日志仍位于：

```text
/home/slam/robot_j6m_ws/log/nvidia_gateway_YYYYMMDD_HHMMSS/
/home/slam/robot_j6m_ws/log/nvidia_ui_YYYYMMDD_HHMMSS/
```

J6M 详细日志位于：

```text
/map/autolabor_runtime/logs/dual_host/YYYYMMDD_HHMMSS/
```

手工验收命令：

```bash
./scripts/health_check.sh --runtime
rostopic hz /gateway/livox/lidar
rostopic hz /Odometry
rostopic hz /cloud_registered_body
rostopic hz /livox/imu
rostopic hz /scan
rostopic hz /fod_camera/image_raw
rostopic hz /fod/detections
rostopic echo /nvidia_cmd_vel_watchdog/status
```

## 2026-08-24 覆盖换道实车检查点

- 最新工作树增加了覆盖生命周期取消与总代价首段选择。`PLANNING/READY/PREPARING/ACTIVE`
  均可取消；
  完成或取消后会清除锁存区域、计划/实走轨迹和计划 ID，可直接第二次框定。前后端均有
  请求代际保护，旧规划/跟踪回调不能污染下一轮任务。本地 Release 全量构建通过；覆盖
  54 项、Qt 37 项针对性测试通过，`health_check.sh --static` 汇总 507 项、0 错误、0 失败。
- 首条线改为比较完整开放路线：在转弯友好跨行顺序及其反序的所有循环起点中，用动态
  规划联合选择条带方向，代价包含首段入场、全部条带和所有转场连接。这是候选族内总代价
  最优，不是任意排列或实际 Navfn 路径的全局最优证明。
- 以上版本已部署至 J6M release
  `/opt/autolabor/dual_host/releases/20260824_193746/install` 并原子切换 `current`；J6M 静态
  健康检查和 `latest` 静态地图双机冷启动运行检查通过，时钟中点偏差 `-2 ms`。Qt 实际
  运行本机 `devel` 新二进制，地图显示状态为 `READY;width=733;height=444;resolution=0.1`；
  清扫页黑字白底、默认 `1.00 m / 15% / 0.80 m/s`、连续两次“框定→取消框定”均已目视
  验证。未发送初始位姿、覆盖计划或导航目标，`/cmd_vel_safe` 与 `/cmd_vel` 连续采样均为零。
- Qt 所有 `QMessageBox` 已显式改为浅色背景、深色正文及深色按钮文字，避免全局浅色字体
  落到系统浅色弹框后看不见。Qt 合约测试共 38 项通过；本机重编译并冷启动后实际打开
  “确认进入视觉行驶模式”弹框，标题、正文和 `Yes/No` 均清晰可见，随后选择默认“否”，
  视觉控制保持 `DISABLED` 且 `/cmd_vel` 仍为零。第一次冷启动曾在 RViz 初始化阶段发生一次
  `pure virtual method called`，监督器按设计完整清理；第二次完整冷启动及运行健康检查通过，
  若以后复现应保留当次 `nvidia_ui_*/gui.log` 继续定位。
- 当前仍为 `WAITING_INITIAL_POSE`。VCU 连续上报 `gamepad_emergency=true`，因此 Qt 正确显示
  “底盘执行门未就绪”并禁用覆盖启动；在解释或解除该实体安全状态前不得开始运动验收。
- 当前分支为 `0824`。此前实车检查基线的底盘驱动 28 项、覆盖模块 46 项针对性测试通过，
  当时 `health_check.sh --static` 汇总 494 项、0 错误、0 失败、0 跳过。
- 前一版 J6M 基线 release 为 `/opt/autolabor/dual_host/releases/20260824_170341/install`。
- 原换道卡死的直接原因已由时间线确认：上一扫掠 goal 成功后，新转场 goal 已提交，约
  `0.15 s` 后 M2 八项状态查询的整批回包空窗超过 `3 s`，覆盖安全门遂取消 goal 并锁成
  人工恢复。手柄是否物理连接与该次误暂停无关。
- NVIDIA M2 驱动改为单请求定时调度，安全状态优先，避免每秒把八个查询同时塞给 VCU。
  运动中 30 秒采到 52 个组合状态，平均间隔 `0.564 s`、最大 `1.993 s`，没有超过 `3 s`；
  原安全阈值没有放宽。
- 实车计划共 10 段。现场已观察到第 2 段 `ENFORCED_SWEEP` 完成后于
  `1787564587.543842472` 原子切到第 3 段 `POINT_TO_POINT_NAVFN_TRANSIT`；第一次无可行路时
  等待 10 秒，随后重规划成功，并进入第 4 段扫掠。后续受阻段也会在有限重试后标记并继续
  后续转场，没有重现“上一条结束后永久卡住”。
- 本轮按低速测试要求把活动 TEB 目标临时钳到 `0.3 m/s`，连续 459 个 `/cmd_vel` 样本最大
  值为 `0.300 m/s`；源码默认速度仍按产品需求保留 `0.80 m/s`。
- 后半轮现场出现多人完全封路、小孩遮挡雷达及操作员遥控接管。定位点数不足、不可达条带
  和 `gamepad/remote emergency` 安全暂停均属预期行为，不纳入算法缺陷；当前任务保持暂停，
  下次应在清场、雷达无遮挡、遥控接管解除后重新完整跑一轮。

## 2026-08-23 当前运行检查点（历史）

- 双机栈已用静态地图 `map_20260822_slice_selfcrop_swept_final` 冷启动，Qt 二维地图状态为
  `READY;width=733;height=444;resolution=0.1`；当前为 `WAITING_INITIAL_POSE`，必须由操作员
  按车辆真实位置设置初始位姿。
- J6M `current` 指向 release `20260823_211926`，远端 ARM64 安装空间包含项目内
  `teb_local_planner`、覆盖规划器和静态局部代价地图配置，并通过远端静态健康检查。
- 定位完成后的同轮只读实测曾确认：增强局部点云约 10 Hz、对齐扫描约 0.5 Hz、
  `/Odometry` 约 10 Hz、局部代价地图更新约 1.8 Hz；最终冷启动未复用该初始位姿。
- 局部静态代价地图现由 `StaticLayer + ObstacleLayer + InflationLayer` 组成；静态地图未知区
  在 TEB 可行性检查中按障碍处理，覆盖 Navfn 回退也禁止穿越未知区。TEB 静态地图模式
  的全局计划前视距离限制为 `8 m`，未来轨迹仅越出滚动局部代价地图窗口（footprint cost
  `-3`）不再被误判为实体碰撞，当前位置越界仍保持 fail-closed。
- 覆盖清扫无动作的另外两个触发点已修复：单帧无效 `/scan` 在 `0.5 s` 新鲜窗口内不会
  锁存永久暂停，持续丢失仍会暂停；普通 `/move_base_simple/goal` 先经过
  `/navigation_pause` 仲裁，覆盖任务活动或安全暂停时不会抢占覆盖 action goal。
- Qt 增加局部跟车视角、动态对齐扫描、路线图例和覆盖/清扫状态；无活动目标时会隐藏并
  清空 TEB 蓝/红路线，防止把历史缓存误认为当前规划。
- 对时脚本改为复用一条 SSH 会话的纳秒采样与中点校验，本次发布实测偏差 `-3 ms`、往返
  `5 ms`；覆盖状态已确认 `avoidance_ready=true`。
- 本地构建、静态健康检查、J6M 远端静态检查和冷启动运行健康检查通过；全量结果为
  `479` 项测试、0 错误、0 失败。没有发送导航目标或非零速度，最终 `/cmd_vel_navigation`、
  `/cmd_vel_safe`、`/cmd_vel` 各连续 3 个样本均为零。

## 2026-08-23 较早停机检查点（历史）

- 已使用 `./scripts/start_dual_host.sh --stop` 同步停止双机栈；
  `autolabor-dual-host.service` 为 `inactive`，J6M 栈未运行，运动授权标记保留。
- J6M `current` 仍指向 release `20260822_233820`；已部署版本通过远端静态检查。
- 本地和 J6M 地图均保留为
  `map_20260822_slice_selfcrop_swept_final`。二维切片为 `Z=-0.4±0.2 m`，
  已加入随车辆位姿变化的实时自点裁剪和轨迹扫掠 footprint 裁剪；
  原始 PCD SHA256 保持不变，融合结果与扫掠区域无占据格重叠。
- 自点裁剪、静态地图融合、定位门、导航与 Qt 合约测试共
  `441` 项通过，0 错误、0 失败；全量本地构建成功。
- Qt 黑屏根因已定位为同一进程同时存在两个嵌入式 RViz/Ogre
  `VisualizationFrame`。源码已改为“综合”和“清扫”共用唯一 RViz 画布，
  切换页签时移动同一个 frame；合约测试和编译已通过，但该 NVIDIA
  GUI 最新改动尚未进行实机重启验收，也无需部署到 J6M。
- 工作树保留了本轮全部 tracked/untracked 进度，未提交、未清理。明日先以
  `./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest` 冷启动，
  重点验证综合/清扫页连续切换、二维/三维显示、地图自适应和
  `/coverage/clicked_point`；不发送导航目标或非零速度。

## 2026-08-22 最后检查点

- NVIDIA Release 构建通过；测试汇总 427 项，0 错误、0 失败。
- J6M `current` 已切换到 `20260822_182346` 并通过远端静态健康检查。
- 静态地图 `latest` 与 J6M maps/current 已切换到
  `map_20260822_slice_zm040_hw020_final`；MID360 融合二维切片为
  `Z=-0.4±0.2 m`，并保留每格至少 20 帧观测过滤。
- 实验性静态地图模式要求新鲜点云/里程计和连续两次 ICP；失锁会零速并取消旧目标。
- 默认无图模式保持不变；静态地图模式启动后仍必须由操作员按真实位置发送初值。

## 2026-08-16 历史检查点

- 完整双机运行态健康检查通过：262 项测试，0 失败。
- Qt、嵌入式 RViz、MID360 点云、ZED 画面和 YOLO 检测均已现场目视确认。
- `/cmd_vel` 与左右轮速均为 `0.0`，四类急停状态均为 `false`。
- 静止采样 60 帧：`x=-0.0359~-0.0241 m`、`y=-0.0486~-0.0338 m`、`z=0.0038~0.0058 m`。
- Qt 已改为室内 FAST-LIO 健康页和局部相对目标；GPS 目标与 RabbitMQ 模块已移除。
- 运动授权已撤销，当前仅允许观察、诊断和无运动导航链验证。

## 2026-08-28 时间最短覆盖规划重构（已部署，待运动验收）

- 已在分支 `0828` 以提交 `8b347f6` 保存重构前完整工作树；本节所述重构位于该提交之后
  的工作树，尚未另行提交。
- 覆盖规划从“优先靠近第一条清扫线、固定弓字顺序”改为以完整任务预计耗时为目标：
  候选扫描角先比较覆盖完整性，再比较从当前位姿入场、全部清扫线和所有转场的秒数；
  对优选角度用确定性有界 beam search 联合选择第一条线、后续线序和每条线的行驶方向。
- 时间模型包含前进/倒车速度、线/角加速度、最大转弯角速度、最小转弯半径、换向惩罚
  和路段交接惩罚。静态地图上的转场距离由禁穿未知区和占据区的八邻域 A* 估算；原始
  TEB 目标值混合了不同量纲的加权残差，不能当作秒数，因此不直接并入离线排序。
- 实际执行架构保持不变：每次转场仍交给 `move_base` 的全局规划器和在线 TEB，沿线再
  使用覆盖执行约束。动态障碍、局部同伦选择和控制跟踪会影响真实耗时，所以当前结果是
  有界搜索下的最短预计时间，不是对动态环境的全局数学最优证明。
- Qt 清扫页“规划参数”新增倒车速度、最大转弯角速度、最大线/角加速度、换向惩罚和
  路段交接惩罚；与清扫宽度、重叠率、前进速度、允许倒车一起用 `QSettings` 在 NVIDIA
  当前用户下即时持久化。修改后的数值会成为下次启动默认值；已有预览就绪时参数锁定，
  取消并重新规划后才能应用新值。
- `PlanCoverage`、`StartCoverage`、`StartCoverageBatch` 和 `CoverageStatus` 接口均已扩展。
  NVIDIA 与 J6M 已成套构建，四个接口 MD5 逐项一致；J6M `current` 已切换到 release
  `20260828_164358`，远端静态健康检查通过，时钟中点偏差 `-2 ms`。
- 使用 fused 地图集 `map_20260827_184123` 完成多次完整冷启动，最终运行健康检查通过；
  Qt 清扫页已目视确认新增参数、黑色文字、地图和状态显示，QSettings 的“修改即落盘、
  重启后加载”已验证并恢复全部默认值。覆盖后端最终为 `IDLE`，地图和避障就绪，定位为
  `WAITING_INITIAL_POSE`，最终 `/cmd_vel` 连续三个样本均为零。本轮没有发送导航目标或
  非零速度，仍需操作员按真实位置设置初值后另行进行低速覆盖运动验收。
