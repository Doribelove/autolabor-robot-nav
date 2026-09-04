# 双机项目终端交接

更新时间：2026-08-31（Asia/Shanghai）

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
  每次修改立即写入 NVIDIA 当前用户的 Qt `QSettings`，经 400 ms 防抖后由
  `/coverage/set_planning_defaults` 事务性同步 J6M TEB、覆盖运行态和当前 release 的
  `coverage.yaml`，但不写进地图区域库；服务未确认时覆盖规划和启动保持锁定；
  全覆盖排序以覆盖完整度为硬优先级，再用不生成路径的 Dubins 秒级代理和上述运动约束，
  对候选角度运行受限 beam search 联合选择首线、后续顺序和方向；任务开始时不再计算或
  缓存全局可执行连接轨迹。首线入场、清扫线间转场和跨区入场都从实时位姿开始，先由
  Navfn 提供长距离拓扑引导，再按约 `10 m` 滚动窗口调用 Hybrid A*；换向 cusp 拆成固定
  档位子段并在零速确认后交接，最终由在线 TEB 跟踪。普通点到点仍使用 Navfn；
- 清扫侧栏分别显示覆盖导航状态、去重后的覆盖面积估算、路线宽度/间距/最小半径、
  VCU/Hybrid A*/TEB 在线运动学核对、MID360 + 前后 LD19 障碍感知状态和“清扫机构未接入”，
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

## 2026-08-31 覆盖转场 Hybrid A* 隔离优化（已集成，未部署）

- 基线固定为 `c872762`，标签为 `hybrid-transit-baseline-c872762`；隔离候选分支
  `experiment/hybrid-transit-opt` 的最终提交为 `d64b9a9`。主工作树只替换同一区域清扫线
  之间的 Hybrid A* 内核，保留现有 Navfn 首线/跨区入场、普通点到点、区域队列、预计算
  服务、在线缓存、Qt 参数和消息接口。
- 新内核枚举 `48` 类 Reeds–Shepp 曲线，按 c872 兼容代价和换向数筛选，逐 `0.10 m`
  检查完整 footprint；解析上界、非完整约束下界、障碍 Dijkstra、周期解析连接和安全上界
  剪枝共同限制扩展。普通/非对称/障碍连接改进窗分别为 `0.20/0.65/0.50 s`，周期连接间隔
  `200` 次扩展；总上限仍为 `1.5 s / 80000`。
- NVIDIA 隔离完整矩阵 `81` 场、每场预热 `3` 次并重复 `20` 次：0 超时，开放/参数/障碍
  最差 P95 分别为 `663.958/682.584/569.649 ms`。c872 在同一标准预算下有 `15` 个应成功
  场景失败；对其成功的 `64` 场，候选无代价增幅超过 `5%`，无换向次数增加。当前真实融合
  地图 A/C 区四个 `0.85 m` 相邻线端点离线场景均成功，候选约 `0.21 s`。
- 本地全量 Release 构建和覆盖包测试通过；路径 CSV/PNG 位于隔离结果目录
  `/home/slam/robot_j6m_ws_worktrees/results/`。J6M 时钟同步后又在独立 ARM64 catkin 工作区
  完成相同的 `81` 场、预热 `3` 次、重复 `20` 次基准：`79` 个应成功场景全部 `20/20`
  成功且 0 超时，`2` 个未知/致命墙场景均正确拒绝，最差 P95 为 `699.026 ms`；隔离库的
  `6` 个 C++ GTest 全部通过。结果保存在 J6M
  `/map/autolabor_runtime/benchmarks/hybrid_d64b9a9/full_20.csv`，本地副本为
  `/home/slam/robot_j6m_ws_worktrees/results/j6m_d64b9a9_full_20.csv`。
- 本轮没有启动 ROS 主链、发布导航目标或非零速度，也没有发布或切换生产 J6M release；
  `current` 仍为 `20260830_211812`。曾误以为 `deploy_j6m.sh` 支持 `--help` 而触发一次部署入口，
  已在远端构建阶段按精确 PID 中止，未生成/切换 release；部署辅助脚本已在中止前同步到 J6M
  持久目录，并保留可追溯的未完成构建目录
  `/map/autolabor_runtime/rootfs/opt/autolabor/dual_host/build_ws.20260831_025055`，未擅自删除。
  后续只剩另行授权的低速实车验收。

## 2026-08-30 覆盖转场 Hybrid A* 架构（已部署，未启动实车）

- 修改前的完整工作树已保存到分支 `0830`，快照提交为 `7384ec7`。该提交包含当时已有的
  Qt、FOD、未知区保护、覆盖入口容差等改动；可复现下载的 354 MB CLIP 权重按仓库规则忽略。
- `CoverageGlobalPlanner` 现有四种显式互斥模式：普通点到点、覆盖首线入场和跨区入场使用
  Navfn；只有同一区域清扫线之间使用 Hybrid A*；扫掠段继续使用管理器强制的精确清扫线。
  模式与所有权不匹配、交接过期或路径缺失时 fail-closed，不会偷偷切换规划器。
- 管理器在发出第一个运动目标前，对代理时间最优的四个完整路线候选批量预计算真实 Hybrid
  线间连接，再以实际路径秒级耗时重排。资源上限结果每 `5 s` 重试，整批最长 `60 s`；任何
  被选中路线的连接缺失都会阻止启动，不能通过跳过下一条线掩盖超时。选中连接由 move_base
  的 `1 Hz` 规划周期逐姿态复核，并按 Qt 的 `1–10 s` 配置周期（默认 `1 s`）主动重搜；
  周期搜索失败时可继续使用仍安全的缓存，缓存阻断则立即重搜并按同一周期重试。
- Hybrid A* 状态仍为 `(x,y,yaw,前进/倒车,转向档)`，按 `1.35 m` 最小半径积分恒曲率运动
  原语，每 `0.10 m` 检查完整 footprint，拒绝未知、致命障碍及地图外。它先尝试最长
  `14 m` 的碰撞检查 Dubins 解析连接，常见无遮挡转场只扩展根节点；失败后使用反向二维
  障碍 Dijkstra 启发式，不连通地图在进入格点搜索前返回；目标 `5 m` 内每 `8` 次扩展再次
  尝试解析收尾。`1.5 s / 80000` 上限没有放宽。
- `SetEnforcedPath` 新增 `TransitProfile`，管理器在提交 action goal 前原子交接本次任务的
  倒车许可、前进/倒车速度、角速度、线/角加速度、换向代价和入口容差。TEB 转场配置保留
  Hybrid 路径朝向、启用有序 via-point，并在允许倒车时降低纯前进偏好；精确扫掠的高权重
  直线跟踪配置保持不变，任务终态仍完整恢复普通导航基线。首线入场在允许倒车时从第一次
  Navfn + TEB 尝试就启用低速倒车；首线秒级排序同步采用相同规则。
- Qt 新增“转场重规划间隔”，并通过 `/coverage/set_navigation_profile` 把速度、倒车许可、
  最大角速度和线/角加速度实时应用为普通点到点及覆盖共用的 TEB 基线；所有值继续写入
  `QSettings` 作为下次默认。宽度、重叠率和换向/交接时间仍只影响覆盖规划。
- 离线条带顺序仍以覆盖完整度为硬优先级；秒级连接代价现在取静态已知自由格 A* 距离和
  无障碍 Dubins 最短曲率路径长度的较大值，再按前进/倒车速度、角速度、加速度、换向及
  分段交接时间估算。这样能在 beam search 中考虑最小转弯半径，同时避免对每个排列运行
  完整 Hybrid A*。它是离散、受限搜索下的最短预计时间，不是动态环境数学全局最优证明。
- NVIDIA 本地 23 包 Release 构建通过；覆盖包 `15` 项契约、`21` 项几何、`80` 项状态机和
  `8` 项 Hybrid A* GTest 全部通过，`health_check.sh --static` 汇总 `816` 项、0 错误、
  0 失败、0 跳过。新增消息、服务和 ARM64 产物已成套部署到 J6M release
  `20260830_211812`，`current` 原子切换和远端安装态健康检查均通过；双端变更消息/服务
  MD5 逐项一致，时钟中点偏差
  `-2 ms`。部署后双机主链保持停止，未发送目标或非零速度，仍需受限低速实车验收。

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

## 2026-08-29 未知区、TEB 前视与规划参数默认值修正（已部署）

- 静态全局和局部代价图新增最终 `UnknownSpaceGuardLayer`。该层位于 Static、Obstacle 和
  Inflation 之后，以原始 `/map` 为掩码，把 `-1` 与地图外格重新置为
  `NO_INFORMATION=255`；即使 `/scan` ray clearing 穿过也不能把未知区改成自由区。
  Navfn `allow_unknown=false`、TEB `treat_unknown_as_obstacle=true` 和覆盖几何仅接受
  `data==0` 的既有三重限制仍保留。
- 静态 TEB `max_global_plan_lookahead_dist` 最终从 `8.0 m` 降到 `2.5 m`，但全局规划 `1 Hz`
  和局部控制优化 `10 Hz` 不变；持续在线重规划是 move_base/TEB 的正常机制，前视长度只
  影响每轮优化规模。
- 按现场调参要求把静态 local costmap 从 `20.0 × 20.0 m` 最终缩为 `6.0 × 6.0 m`，分辨率
  保持 `0.10 m`，局部障碍标记/射线清除距离改为 `6.0/6.5 m`。滚动窗口相对车辆每侧
  只有约 `3 m`，因此更远的观测会被代价图边界裁剪；`2.5 m` TEB前视低于覆盖安全门的
  `2.55 m`上限。实车运动验收时仍需重点观察绕障提前量、TEB可行性重置和控制周期。
- Qt 原本已对全部十项规划参数做到“数值变化即 QSettings sync”；Qt 自身的下一次规划和
  队列请求也会携带当前整组值。修正了 AI 覆盖入口原先另有固定默认值的问题：未显式指定
  的字段现在逐项读取同一份 NVIDIA QSettings，并做范围校验；显式字段仍可逐项覆盖。
  当前用户保存的最大前进速度是 `1.2 m/s`，所以新的省略参数 AI 请求会使用 `1.2`，而
  不是旧的 `0.8` 回退值。
- 未知区、TEB 前视和规划参数改动此前已完成本地全量 Release 构建；未知掩码旋转地图/
  地图外/畸形输入 GTest 3 项、MCP 参数与安全合约 30 项通过。`6.0 × 6.0 m` local costmap
  调参后又完成 YAML 实值核对、静态地图合约 11 项及项目静态健康检查 769 项，均无失败。
  J6M 已构建并切换到 release `20260829_160747`，远端静态检查和 fused 静态地图完整冷启动
  运行检查通过。运行参数实测 `width=6`、`height=6`、`resolution=0.1`，局部代价图消息为
  `60 × 60` 格，Qt 进程是其订阅者；覆盖状态 `IDLE` 且连续三帧 `/cmd_vel` 为零。本轮没有
  发送导航目标或非零速度，因此尚未进行6 m窗口下的运动验收。
- 首次使用6 m窗口启动覆盖时发现已部署的`5.0 m` TEB前视会被上述余量安全门正确拒绝，
  Qt弹窗对应后端日志为`TEB lookahead must retain margin inside the rolling local costmap`。
  运行参数已在任务未活动时热调为`2.5 m`，原计划仍为`READY`且未触发运动；持久源码和
  Qt局部跟车取景已同步修正，等待当前现场验证结束后再发布新J6M release并重启Qt。

## 2026-08-29 局部窗口回退与静态障碍/转向摆动排查

- 现场确认 `6 × 6 m` 窗口和 `2.5 m` 前视没有改善直线跟踪，因此权威源码恢复到此前实际
  基线：local costmap `20 × 20 m`、`0.10 m` 分辨率、实时障碍标记/射线清除 `10/11 m`，
  静态 TEB 前视 `8 m`；Qt 跟车视图也恢复按 20 m 窗口取景。覆盖余量安全门上限相应为
  `0.85 × 10 = 8.5 m`。
- 本轮日志在 `(14.4, -6.0)` 附近连续记录 TEB 预测足迹命中 lethal cell、以 10 Hz 重置，
  随后触发 `possible oscillation`。对应 fused 静态地图确有固定占用带。这说明现场看到的
  左右修正至少主要来自规划器在占用边缘反复求解，现有证据不足以认定前轮执行器本身超调。
- local costmap 的 StaticLayer 和 ObstacleLayer 是叠加关系。实时 `/scan` 的 ray clearing
  只能清掉 ObstacleLayer 自己标记的格子，不能降低 `/map` 贡献的静态占用；否则真实墙体也
  会被无条件清除。因此建图时临时障碍残留不是更新漏洞，应修订对应 map-set。未获明确区域
  和人工确认前，不实现自动“清静态障碍”。
- 为降低控制器追逐单周期微小误差的敏感度，静态模式新增
  `TebLocalPlannerROS/control_look_ahead_poses=2`：控制命令跨两个优化轨迹间隔取平均。
  没有加入航向死区、没有降低障碍权重，也没有改 M2 的 Ackermann 几何转换。仍需在定位、
  双 LD19 和底盘急停状态恢复正常后，以不高于 `0.3 m/s` 对比 `/cmd_vel` 曲率与
  `/m2_driver/wheel_angle`，才能判断是否还存在机械回中偏差或执行器超调。
- 静止零速只读抽样中，连续 10 帧 `/cmd_vel` 均为零，连续 10 帧前轮反馈稳定在
  `-0.00291 rad`（约 `-0.17°`），没有看到静止回中漂移；这不能替代运动中的阶跃/跟踪测试，
  但不支持“零位本身严重偏斜”这一假设。
- 本地 Release 构建、静态地图合约 11 项、覆盖状态机 76 项、Qt 合约 46 项及项目静态检查
  769 项均通过。J6M 已原生构建并切换到 release `20260829_170203`，远端静态检查通过，
  时钟中点偏差 `-2 ms`。随后以 `map_20260827_184123` fused 地图完整冷启动，运行态检查
  通过；实测 ROS 参数依次为 local costmap `20/20/0.1 m`、scan `10/11 m`、TEB 前视
  `8.0 m`、`control_look_ahead_poses=2`。定位保持 `WAITING_INITIAL_POSE`，连续三帧
  `/cmd_vel` 为零；未发送目标或进行运动验收。定位前 rolling costmap 不发布网格属于当前
  TF 门控结果，待人工初值后再核对实际消息应为 `200 × 200` 格。避障链当前就绪，但 VCU
  仍报告 `gamepad_emergency=true`、`robot_emergency=true`，底盘执行门保持关闭；本轮没有
  清除或绕过该硬件急停状态。

## 2026-08-31 Hybrid 转场跟踪修复（已部署，未启动实车）

- 0830 实验 bag 中，Hybrid 转场的蓝色全局路径中位/P95/最大约为
  `6.56/8.77/10.51 m`，TEB 红色局部轨迹约为 `7.85/13.94/16.21 m`；同时记录到
  31 次轨迹不可行重置和 12 组振荡恢复。已部署 release 的静态 TEB 前视为 `8 m`，会让
  局部优化一次吞入相邻短线距转场的大部分长圆弧，是问题放大因素，但不能单独解释所有
  控制/代价图循环超期。
- 全局规划插件不再按 Qt 周期替换仍安全的 Hybrid 缓存。每个 `1 Hz` makePlan 调用仍会
  对剩余路径做完整 footprint/未知区复核；只有路径受阻或车辆偏离超过阈值才在线重搜。
  Qt 的 `1–10 s` 参数改为失败搜索的重试冷却时间，防止行驶中在前进/倒车或不同绕行拓扑
  之间翻转。
- 仅在线间 Hybrid action 中把 TEB `max_global_plan_lookahead_dist` 临时设为 `3.0 m`；
  普通点到点、Navfn 首线/跨区入场和沿线扫掠继续使用 `8.0 m` 基线，并通过已有 TEB 快照
  在换段/任务终态恢复。
- `nav_msgs/Path` 没有档位字段。管理器现在按路径位姿切向恢复每条边的正/倒车符号，在每个
  Reeds–Shepp 换向 cusp 拆成两个独立 move_base action。正向子段保留前进偏好，同时按操作员
  许可保留 `0.3 m/s` 以内的低速倒车脱困；反向子段取消 TEB 的纯前进偏好。中间 cusp 使用
  `0.12 m / 0.15 rad` 专用容差；最终条带
  入口仍为 `0.30 m / 0.40 rad`。因 `free_goal_vel=false`，每个换向点必须先以零终速完成，
  再提交下一子段；缓存续接跳过已被实时起点替代的最近锚点，避免先回头补 cusp。
- 删除了“搜索进入目标容差后，直接把精确 action goal 追加成最后一点”的非运动学收尾。
  容差现在只触发一次精确 Reeds–Shepp 终端连接；末端继续受 `1.35 m` 最小转弯半径、完整
  footprint 碰撞和时间代价约束。实车完成判断仍可使用入口位置/航向容差，不要求零误差压点。
- 这些修改已完成 NVIDIA 完整 Release 构建；覆盖包 `130` 项、覆盖状态机/契约/Qt 定向
  `144` 项和项目静态健康检查 `815` 项均为零失败。`180°`、`2.0 m` 线距场景重复 `20`
  次全部在 `800 ms` 预算内成功，中位 `201.092 ms`、零超时；路径被解析为
  `前进 1.941 m → 倒车 0.350 m → 前进 1.941 m` 三个停车子段，可视化和 CSV 位于
  `runtime/hybrid_fix_180deg_2m_20260831/`。随后完成 NVIDIA Release 重建和 815 项静态
  验收，并在 J6M ARM64 chroot 原生编译、安装、原子切换到 release `20260831_142443`；
  远端安装态健康检查通过，关键 coverage 消息/服务 MD5 与 NVIDIA 一致，时钟中点偏差
  `-3 ms`。部署前后主链均保持停止，没有发送目标或非零速度；仍需按现场安全条件做不高于
  `0.3 m/s` 的受限实车验收。

## 2026-08-31 Qt 规划参数与 J6M YAML 事务同步（已部署，主栈停止）

- 新增 `CoveragePlanningParameters` 消息和
  `/coverage/set_planning_defaults` 服务。Qt 清扫页的 11 项规划参数统一以完整快照下发；
  任一输入修改先写当前用户 QSettings，400 ms 无继续输入后才调用 J6M。
- J6M 严格校验范围并拒绝覆盖规划/预览/执行中的修改；随后在当前 release 的
  `coverage.yaml` 同目录生成、校验和 fsync 临时文件，动态写入并回读 TEB，最后以
  `os.replace` 原子提交 YAML 与覆盖管理器默认值。YAML 提交失败会把 TEB 恢复到调用前
  快照，管理器值和原文件不变；注释、字段顺序和不相关配置均保留。
- 清扫页新增“恢复默认参数”和明确的同步状态。出厂基线来自独立
  `coverage_factory_defaults.yaml`：`1.00 m`、`15%`、允许倒车、前进/倒车
  `0.80/0.30 m/s`、角速度 `0.60 rad/s`、线/角加速度 `1.00/0.50`、换向/交接
  `1.00/0.50 s`、Hybrid 重试 `1.00 s`。恢复服务成功前不修改 Qt 控件或 QSettings；
  参数未获 J6M 确认时禁止生成轨迹、单区启动和队列启动。
- NVIDIA 完整构建通过；覆盖、Qt、双机定向测试共 `199` 项零失败，项目静态健康检查
  `815` 项零失败（唯一提示是未枚举到物理麦克风）。J6M 已原生构建并原子切换到 release
  `20260831_170330`，远端静态健康检查通过；新消息/服务两端 MD5 分别为
  `0aba6740971723619d4f85f4840b60cd` 与 `fa21bd469f0a8b5c6298cf977cd09c1b`，时钟
  中点偏差 `-2 ms`。部署结束后双机主栈保持停止，未发送初始位姿、导航目标或非零速度；
  Qt 仅做了 5 秒 offscreen 无 master 存活冒烟测试，尚未做带地图的人工点击目视验收。

## 2026-09-01 Hybrid 换档/入口交接与完整路径显示（已部署，主栈停止）

- 最近一次实车 bag 的首个前进子段没有进入 `0.12 m` cusp 点容差，最近距离约 `0.308 m`，
  因而倒车子段从未启动。旧恢复路径又继承了当前前进子段的单档限制，继续追逐已经越过的
  旧 cusp，最终形成长前进绕行、TEB 无进展和任务暂停。
- 管理器现在对 Navfn 入场、Hybrid cusp 和最终清扫线入口维护连续的有向路径进度。补充
  完成门要求从正确起点侧接近、至少 `85%` 进度、连续越过终点平面 `0.02 m`、横向误差
  不超过 `0.30 m`，并满足 cusp `0.15 rad` 或入口 `0.40 rad` 航向限制；定位单帧跳变会
  fail-closed。精确取消当前 action 后，还必须由新鲜 M2 `/odom` 连续确认速度不超过
  `0.08 m/s`，才允许换档或开始扫掠。接近 cusp 的前进上限为 `0.60 m/s`。
- 单档子段阻断、明显越界或无进展后，不再锁在旧档位/旧 cusp。管理器调用同一 Hybrid
  预计算服务，从实时位姿到下一清扫线真实入口重新规划允许任务级前进/倒车的完整剩余路径，
  校验终点后按新 cusp 重新分段；下一清扫线不会被跳过。
- 新增锁存标准话题 `/coverage/hybrid_transition_path`。Qt 两个 RViz 配置用橙色显示当前
  完整逻辑 Hybrid 转场，蓝色仍是当前交给 move_base/TEB 的单档全局参考路径，红色是 TEB
  局部轨迹；取消、终态或切到非 Hybrid 分段时清空橙线。
- 覆盖状态机 `94` 项、覆盖契约 `15` 项、Qt 契约 `47` 项全部通过，Python 语法、
  `git diff --check`、NVIDIA 23 包 Release 构建和项目静态健康检查 `817` 项均通过。
  J6M 已在 ARM64 chroot 原生构建并原子切换到 release `20260901_211830`；远端 install-space
  健康检查通过，安装后的 `coverage_manager.py` 和 `coverage.yaml` SHA-256 与本地完全一致，
  时钟中点偏差 `-3 ms`。发布结束后 supervisor 和 J6M 主链均停止，没有发送初始位姿、
  导航目标或非零速度，仍需按现场安全条件进行不高于 `0.3 m/s` 的受限实车验收。

## 2026-09-02 覆盖按需滚动架构与入口异常恢复（已部署，主栈停止）

- 覆盖任务不再预计算任务级全局可执行轨迹。候选角度、清扫线顺序和方向只用几何完整度
  与不落地生成路径的 Dubins 时间代理做受限 beam search；实际首线入场、换行和跨区转场
  均从最新定位姿态开始，以 Navfn 长距离拓扑引导和约 `10 m` Hybrid A* 滚动前缀执行。
- Hybrid 路径按前进/倒车 cusp 拆成固定档位 action，换档前要求新鲜里程计连续确认零速；
  活动路径连续偏离位置 `0.35 m` 或航向 `0.40 rad` 三次即取消，并从实时位姿重新规划，
  不再追逐已经越过的历史入口或 cusp。
- 到达清扫线入口前保持硬门控。位置或航向超差时不会进入 `SWEEPING`，而是进行允许倒车且
  倒车代价较低的局部恢复；恢复路径总长限制 `4.0 m`、前进累计限制 `1.2 m`，避免用大幅
  前进绕圈修正入场姿态。越过清扫线终点后的补充完成仍要求车辆已确认降到零速。
- M2 最小转弯半径已在覆盖管理器、Hybrid A*、J6M launch 和 TEB 配置中统一为
  `1.35 m`；TEB 输出 Twist 还会按该半径做最终曲率投影。仿真树保留了异常入场与人为冲出
  清扫线测试，但使用的是 `1.20 m` 真值车辆和真值固定档位跟踪器；该跟踪器没有部署到
  生产系统，生产仍使用 move_base + TEB 以及现有 FOD、定位和看门狗安全链。
- NVIDIA 完整构建通过；覆盖包 `151` 项、TEB `12` 项定向测试均零失败，项目静态健康检查
  共 `876` 项零失败（唯一提示是未枚举到物理麦克风）。J6M 已在 ARM64 chroot 原生构建并
  原子切换到 release `20260902_204607`，远端 install-space 健康检查通过；新增 Hybrid
  请求、结果和预计算服务的消息 MD5 与 NVIDIA 一致，关键脚本及配置 SHA-256 也一致，时钟
  中点偏差 `-2 ms`。
- 部署前后 supervisor、ROS master 和 J6M 主链均保持停止，没有发送初始位姿、导航目标或
  非零速度；`MOTION_ENABLED=true` 和现有独立运动授权标记均未被部署流程改写。仍需满足
  实体急停、人员远离车轮、定位与传感器健康等现场条件后，以不高于 `0.3 m/s` 做 1.35 m
  半径底盘的受限实车验收，才能确认生产 TEB 的倒车换档与异常入场跟踪效果。

## 2026-09-03 直接 Hybrid 转场、cusp-aware 跟踪与 Qt 适配（已部署，主栈停止）

- 当前正式架构不预计算任务级可执行轨迹。每个区域首条清扫线使用一个 Navfn + TEB 普通
  导航 action；同一区域后续线间转场从实时停车位一次直接 Hybrid A* 到下一入口，不再使用
  Navfn 拓扑或 10 m 滚动前缀。A 区完成后进入 C 区属于 C 区的新首线，不计作线间转场。
- Hybrid 完整带符号连接在管理器层保持一个 action。新增必需节点
  `hybrid_path_follower.py`：普通导航/扫掠转发新鲜 `/cmd_vel_teb`，Hybrid 阶段根据
  `map -> base_link` 取得唯一上游速度控制权，并只在 M2 `/odom` 实测速度不高于
  `0.03 m/s` 后内部换挡。每条新路径还必须取得 `CoverageGlobalPlanner` 对最新 costmap
  未来 3.0 m 的 1 Hz 安全许可；许可失效或超过 1.5 s 即零速，失效路径只由管理器重算，
  不允许插件形成第二路径权威。输出仍依次经过静态定位门、FOD 仲裁和 NVIDIA 最终看门狗。
- 默认事件触发重算；1 Hz 无条件对照没有缩短 A 区转场，却产生约 6 倍 Hybrid 搜索并出现
  42 次控制循环、30 次地图更新循环超时，因此未作为默认架构。生产参数使用 1.35 m 最小
  转弯半径、1.30/0.80 m/s 转场前进/倒车上限、0.30 m Hybrid 前视、TEB 普通阶段 4 m
  前视和 Hybrid 后台 2 m 前视。
- Qt 已随扩展后的 `CoverageStatus` 重新构建，继续兼容原有覆盖服务；状态与提示区同时显示
  清扫任务和专用转场速度/加速度/前视，按两类参数较大值核对 NVIDIA 看门狗，并明确区分
  “首线 Navfn + TEB”和“区内直接 Hybrid”。
- latest 地图的 Gazebo 真值仿真中，A、C、自定义 D 区各 3 遍，共 39 次正式区内转场全部
  完成，平均 6.909 s、最大 8.492 s，计划长度 4.240–4.690 m；没有大幅前进绕圈、停住持续
  重规划或超过 10 s。D 区中央到首线的 Navfn + TEB 独立入场曾用时 19.996 s并发生速度
  符号抖动，按任务范围只记录，未计入转场统计。
- 本机 Release 构建通过；四个相关包定向测试共 452 项零失败，项目完整静态健康检查
  902 项零错误、零失败、零跳过。J6M 在 ARM64 chroot 原生构建并原子切换到 release
  `20260903_173104`，远端 install-space 健康检查通过，时钟中点偏差 `-3 ms`。
- 下列本地源码与远端 install-space SHA-256 一致：覆盖管理器
  `ecdd78bdd4f0f54e41ed323adf79d5acb53699a1512e31669a4c230dd04b624f`、Hybrid 跟踪器
  `78c5226ac68002cb3ab2f2226f5e19ad1e408a925f0132706812621a7fae44b7`、`coverage.yaml`
  `e36b8b5276c8c81b80ad39b345ffe20b3465f0bd6e82a33f4c8c9577bfd4e726`、覆盖 launch
  `71aa6a1fc5ff2e1821dbfeb952438327c49db09e3c893b3253ff57265e62cab9`、J6M 导航 launch
  `b5875ec8351c6db47a652f140dd8b696d197246698fe6922ec3202d3c2939896`、`CoverageStatus.msg`
  `e8c16dbb3bca8371aaefb4941358aabf0edfef1c4b8d1697082883749c49a5e4`；该消息两端 MD5 为
  `f42bc13ebd3bf75b9328b0d362577ebe`。
- 部署后 supervisor、ROS master 和 J6M 主链保持停止，没有发送初始位姿、导航目标或非零
  速度。`MOTION_ENABLED=true` 与已有运动授权标记未被改写；未做运行态 topic 所有权、Qt
  人工目视或实车运动验收。首次运动验收仍须满足现场全部安全条件，并限制在 0.3 m/s 以内。

当前架构和逐次实验数据分别见
[`COVERAGE_NAVIGATION_ARCHITECTURE_20260903.md`](COVERAGE_NAVIGATION_ARCHITECTURE_20260903.md)
与 [`COVERAGE_TRANSITION_EXPERIMENT_20260903.md`](COVERAGE_TRANSITION_EXPERIMENT_20260903.md)。

## 2026-09-04 TEB 固定档位、cusp 条件续接与异常仿真（已部署，待实车验收）

- Hybrid 最终清扫入口的搜索区域已从 `0.15 m / 0.20 rad` 放宽为
  `0.30 m / 0.349066 rad（20°）`；外层入场验收同步改为位置/横向 `0.40 m`、航向
  `0.436332 rad（25°）`。中间 cusp 仍为 `0.25 m / 0.20 rad`，没有随最终目标一起放宽。
  30°外层航向门在仿真中曾以约 28.5°姿态切入 forward-only 清扫并持续停车，因此被否决。
- 旧 release `20260903_180943` 中的 `hybrid_path_follower.py` 已从权威源码和新部署链移除。
  当前架构把每条直接 Hybrid A* 连接按 cusp 拆成固定档位 move_base action，
  全部由 TEB 闭环跟踪；`hybrid_teb_command_mux.py` 只做新鲜度、安全许可、档位符号和
  1.35 m 曲率 fail-closed 校验，不包含第二套局部跟踪控制律。
- 修复了把所有 cusp 当作异常、每次都重算的问题。固定档位段在位置/航向、短段有向进度和
  实测零速满足后，先尝试连接下一缓存段：最多裁掉 0.30 m，弦切向误差不超过 0.10 rad，
  连接半径不小于 1.35 m。能接就继续；不能接才从实时位姿到原最终清扫入口重算完整剩余
  连接，而不是只规划到邻近 cusp。
- 当前 TEB 架构 A 区正常回归三次换行为 19.00、16.00、16.61 s，均完成；无碰撞、曲率
  违规、非零速换挡、无进展或振荡。超过旧 10 s 指标主要来自 cusp 减速停车与事件搜索，
  10 s 已在仿真审计中降为性能告警，不再把完整成功任务判失败。
- 人为入口横移 0.35 m/偏航 0.25 rad 后，入口硬门拒绝清扫，1.50 m 的倒车0.60 m再前进
  0.90 m路径在 5.36 s 内恢复；人为 0.55 m/s 冲线后最大超出 0.623 m，系统等待实速降至
  0.01 m/s 后才提交清扫完成，并从实际停车位开始下一转场。两轮均 A 区 `COMPLETED`，
  没有大幅前进绕圈或持续停车。
- 隔离仿真证据与逐段路径形状位于
  `coverage_gz_sim_tree/TEB_FIXED_GEAR_VALIDATION_20260904.md`。仿真只含 latest 静态地图、
  真值位姿和 bicycle plant，不含激光、动态障碍、真实执行器或安全链。
- 当前 TEB 的 1 Hz 单变量对照在首个固定前进段连续生成 6 条有效路径，但反复触发 mux
  归零窗口，3 s 仅移动 0.092 m后无进展取消，最终因 4.50 m恢复候选超过4.00 m硬边界而
  `PAUSED`。这证明问题在无条件路径替换破坏闭环连续性，不是 Hybrid 搜索不出路径。
- 修正 TEB 倒车 via-point 的方向语义：倒车时车辆航向应为路径行驶切向加 π；倒车固定档位
  只用第一个优化边提取命令，前进/清扫仍保持两个 pose 前视。固定档位符号检查移动到速度
  饱和之前，反向候选不能再被夹成 `-0.0` 而形成不可见的持续停车。
- 最终配置在 A 区连续 3 轮完成 9 次线间转场，12.996--14.506 s、平均 13.781 s；每轮
  6 次实际换向，零碰撞、曲率违规、非零速换挡、无进展或振荡。转场停车累计
  2.704--4.134 s，TEB 相对 Hybrid 参考的 P95 最大偏差为 0.034--0.043 m。另一次
  0.220 m / 10.31° 人为入场扰动在实测横偏 0.235 m、航向 12.44°时直接被新门接受，
  0.238 s 后开始原清扫线，没有 Hybrid 恢复或大幅前进绕圈，任务 `COMPLETED`。
- 本地 Release 工作区 23 个包构建通过；覆盖包与 TEB 定向回归均零失败，全工作区测试汇总
  为 933 项、0 错误、0 失败、0 跳过。随后通过 `deploy_j6m.sh` 在 ARM64 chroot 原生构建并
  部署 release `20260904_022107`，J6M `current` 已原子切换到该版本；远端静态健康检查通过，
  安装态 `coverage_manager.py` 和 `hybrid_teb_command_mux.py` 的 SHA-256 与本地源码一致，
  时钟中点偏差为 `-2 ms`。
- 部署前已用统一入口完整停止双机栈；2026-09-04 随后用 `latest` 地图完成冷启动、运行态
  topic/节点归属检查和 Qt 目视，操作员设置初始位姿后定位为 `LOCALIZED`。首轮当前架构
  实车覆盖在第 3 条清扫线前自动暂停；随后换到 41.485 m² 更空旷区域复测，实际完成计划
  第 1 条南向北清扫线后，在去第 2 条线的第一次转场复现相同故障。Hybrid 路径均成功生成，
  TEB 固定倒车段反复产生反号候选并被安全门归零，偏离重规划后的恢复候选又因前进距离
  超过 1.20 m防绕圈门而被后置拒绝。当前第二轮任务保持暂停，release 未修改；清扫顺序与
  完整证据见
  `docs/COVERAGE_REAL_ROBOT_EXPERIMENT_20260904.md`。`MOTION_ENABLED=true` 与已有运动授权
  标记未被改写，受限低速实车验收仍未通过。

## 2026-09-04 线序恢复、入口门修正与 Navfn+TEB 对照（已部署，待实车验收）

- 当前权威源码与 `coverage_gz_sim_tree` 再次逐文件同步：线序恢复为允许跨行的确定性 time
  beam，首入口先限制在静态已知自由最短距离 `+0.30 m` 的端点候选带；入口恢复删除旧版
  “累计前进不得超过 1.20 m”后置门，仍保留 4.0 m 总长、footprint/未知区、1.35 m 曲率
  和最终入口区域合同。
- 新建隔离对照树 `coverage_gz_sim_navfn_teb_tree`，除把所有区内换线改为 Navfn + TEB、
  将该 TEB 前视改为 8 m 外，与当前仿真保持一致。同一 latest 地图、同一区域、同一初值和
  同一 `[0,3,1,2]` 线序下，当前 Hybrid+TEB 三次主连接为
  `8.500/11.498/14.500 s`；计入一次 21.000 s 的入口恢复后总计 55.498 s。Navfn+TEB
  对照总计 308.510 s、停车 166.184 s、局部路径相对全局路径 P95 最大偏差 1.040 m，且
  有 7 次速度反号未先归零，故没有把对照逻辑或 8 m 前视带入生产。
- 本机 23 包 Release 构建通过；覆盖包 Python 160 项和 C++ 7 项定向测试通过，工作区
  测试汇总 936 项、0 错误、0 失败、0 跳过；静态健康检查通过，仅提示本机当前没有枚举
  到物理麦克风。
- 已通过 J6M ARM64 chroot 原生构建并原子部署 release `20260904_140758`；`current`
  只读核对指向该 release，远端安装态静态健康检查通过。安装态 `coverage_manager.py`、
  `coverage.yaml`、`coverage_geometry.py` 的 SHA-256 分别为
  `b61d9b5d3e232c92a7f457b44b9f75ab095849aa9ccd2aa539a9fe1acd3c77ed`、
  `e043b412f81ad08be6bfc05224beb6c0e8b1a19d710ab6b8d648ca43d0c11565`、
  `4cd832aaca0a8691a666fb915949042aa5097f65c325905e83e62205dd716307`，与本地一致；时钟
  中点偏差 `-3 ms`。
- 部署后 NVIDIA supervisor、ROS master 和 J6M 主链保持停止，未发送初始位姿、导航目标
  或非零速度。`MOTION_ENABLED=true`、`FOD_MOTION_ENABLED=true` 和已有运动授权标记均
  保留；新 release 尚未做 Qt 目视、运行态 topic 归属或受限低速实车验收。

## 2026-09-04 0904 快照与异常恢复响应缩短（已部署，主栈停止）

- 修改前的完整工作树已固化到分支 `0904`、提交 `a25fdb4`；后续修改位于
  `0904-fast-replan`，代码提交为 `678fdd9`。
- Hybrid 连续无进展判定从 `3.0 s / 0.10 m` 缩短为 `2.0 s / 0.10 m`；分段 action
  异常取消并确认终态后的固定等待从 `10.0 s` 缩短为 `2.0 s`。最多 3 次重试、精确取消、
  1 Hz 当前路径安全校验、碰撞/未知区、1.35 m 曲率、固定档位和入口验收门均保持不变。
  生产模式仍是事件触发重算，不会每秒替换一条仍安全的 Hybrid 路径。
- 同次发布还包含三次实车实验后修正的首线入场 TEB 前视：Navfn + TEB 首线继承普通导航
  `4.0 m` 基线，区内 Hybrid 固定档位继续使用 `2.0 m`。
- 生产与隔离仿真镜像各通过 117 项覆盖状态机和 15 项覆盖合同；静态导航合同 13 项通过。
  NVIDIA 23 包 Release 构建通过，项目静态健康检查汇总 936 项、0 错误、0 失败、0 跳过，
  唯一警告为未枚举物理麦克风。
- J6M 已在 ARM64 chroot 原生构建并原子切换到 release `20260904_163527`；远端静态健康
  检查通过，时钟中点偏差 `-3 ms`。安装态 `coverage_manager.py` 与 `coverage.yaml`
  SHA-256 分别为 `9441bc772d34669f12315c33263466fef2bf9d5e13ba1c6a9c518eea415d8bf2` 和
  `737a492d848a3eef36ce387ac6c4f450694b755d546b7ee4a0900e6de8b94b9e`，与本地一致；YAML
  回读两项参数均为 `2.0`。
- 部署前统一入口已完整停栈，发布后 J6M PID 记录为空；没有发送初始位姿、导航目标或
  非零速度。`MOTION_ENABLED=true`、`FOD_MOTION_ENABLED=true` 和已有运动授权标记均保留。
  当前 MID360 物理链路无载波，导致部署前运行检查中的 Livox、FAST-LIO 和融合 `/scan`
  数据超时；新 release 尚未做静态地图冷启动、Qt 目视或实车运动验收。
