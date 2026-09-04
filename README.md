# robot_j6m_ws 双机工作空间

这是从 `/home/slam/robot_ws` 当前工作树提取出的独立 ROS Noetic 工作空间。原工作空间及其 68 项未提交改动没有被修改；新目录不包含旧 Git 历史，也没有复制旧 `build`、`devel`、日志、rosbag 或虚拟环境。

## 当前状态

- NVIDIA 端 23 个包已用 Release 配置编译通过；当前静态检查结果汇总为 933 项通过、
  0 错误、0 失败、0 跳过。
- `fast_lio_localization` 已通过编译、相关测试和现有 bag 重放验证；覆盖清扫的几何规划、
  已知区域持久化、多区域队列、Qt、静态地图启动、暂停桥和部署契约测试均已纳入同一
  测试汇总。
- J6M Ubuntu 20.04/ROS Noetic chroot 已存在于 `/map/autolabor_runtime/rootfs`。
- J6M `current` 当前实测指向版本 `20260904_022107`；本轮固定档位 TEB、直接 Hybrid A*
  线间转场和入口目标区域机制已经部署。local costmap 为 `20 × 20 m`、
  `0.10 m` 分辨率、`10/11 m` 障碍标记/射线清除距离；静态 TEB 普通导航/首线/扫掠前视为
  `4 m`，Hybrid 后台诊断前视为 `2 m`，并启用 `control_look_ahead_poses=2`。NVIDIA/J6M
  构建与远端安装态健康检查均通过；关键源码与 install-space SHA-256 一致，时钟中点偏差
  `-2 ms`。2026-09-04 已用 `latest` 地图完整冷启动并通过运行态健康检查；操作员设置初始
  位姿后定位为 `LOCALIZED`。当前架构两轮实车覆盖均暂停：首轮运行到第 3 条线前，第二轮
  换到 41.485 m² 的更空旷区域后在线 1 -> 线 2 的首次转场复现。两轮 Hybrid 搜索本身均
  成功，TEB 固定倒车段连续产生反号候选并被安全门归零，偏离事件重规划后的候选又因超过
  1.20 m防前进绕圈门而被拒绝。第二轮实际清扫顺序及逐线坐标已经记录；任务保持暂停，
  受限低速实车验收仍未通过。
- 当前本地覆盖源码不再预计算任务级可执行 Hybrid 全局轨迹：先用无路径 Dubins 秒级代理
  决定清扫线顺序/方向；每个区域首线用一个 Navfn + TEB 普通导航 action 入场，同一区域
  后续换行从实时位姿一次直接 Hybrid A* 到下一入口。完整连接按 cusp 拆成固定档位的
  move_base action，每段都由 TEB 闭环跟踪，cusp 换挡前确认实测零速；输出 mux 只校验
  TEB 指令新鲜度、档位、曲率及全局插件的安全许可，不再实现任何自定义路径跟踪控制律。
  cusp 零速后先检查实测位姿能否以同档切向且不小于 1.35 m 半径接回缓存后缀；可接就
  继续，不可接才从实测位姿到原最终入口重算完整剩余路径，cusp 本身不是异常触发器。
  全局插件以 1 Hz 校验未来 3 m costmap，失效时 mux 立即零速并由管理器从实时位姿到最终
  入口整段重算。Hybrid 最终入口搜索区域放宽为 `0.30 m / 20°`，清扫前使用
  `0.40 m / 0.40 m / 25°` 的位置/横向/航向入场硬门；中间 cusp 仍为
  `0.25 m / 0.20 rad`。超差时使用倒车优先且限制总长/前进长度的
  Hybrid 恢复；当前路径连续偏离会精确取消并从实时位姿重算。覆盖、Hybrid、TEB 和 launch
  已统一为 `1.35 m` 最小转弯半径，TEB 输出端另做硬曲率投影。本地 Qt、ROS 消息和 J6M
  控制链源码已完成适配；J6M 控制链已随 release `20260904_022107` 部署。详见
  [当前覆盖导航架构](docs/COVERAGE_NAVIGATION_ARCHITECTURE_20260903.md)和
  [当前 TEB 仿真验证](coverage_gz_sim_tree/TEB_FIXED_GEAR_VALIDATION_20260904.md)。首轮当前
  架构实车故障的路径、时间线和日志判读见
  [2026-09-04 实车实验记录](docs/COVERAGE_REAL_ROBOT_EXPERIMENT_20260904.md)。39 次旧转场
  数据使用已删除的自定义跟踪器，只保留为历史对照。
- 两机 ROS 普通消息和 Livox 自定义消息已在当前临时网段双向验证。
- J6M relay、FAST-LIO、避障融合、增强点云、move_base 与 FOD 仲裁已在实机数据流中完整拉起，并通过连续启停清理验证。
- 三地图建图、跨帧静态过滤和地图同步已完成 bag 验证；当前 `latest` 为
  `map_20260827_184123`。已知三维图定位采用 FAST-LIO
  高频里程计加低频多尺度 ICP。旧版覆盖清扫 V1 已完成静态地图冷启动、Qt 目视和低速
  实车分段验收；当前每区首线与普通点到点使用 Navfn + TEB，区内换行使用直接 Hybrid A*
  并由 TEB 跟踪固定档位 cusp action，精确扫掠仍由 TEB 强制跟踪清扫线。新架构已通过本地编译和离线
  测试；首轮操作员参数实车实验已经暴露倒车固定档位 TEB 反号重置和恢复候选后置拒绝，尚未
  完成整轮或通过低速验收。旧版现场整轮任务曾因人群封路、雷达被遮挡和操作员遥控接管而安全
  暂停。

Hybrid A* 的搜索状态、g/h 代价、Reeds–Shepp 候选选择、历史完整路线重评分方案和参数归属，
见 [docs/HYBRID_ASTAR_COST_CALCULATION.txt](docs/HYBRID_ASTAR_COST_CALCULATION.txt)。该文档
使用纯文本分式和 Unicode 数学符号，不依赖 Markdown 数学渲染。

## 机器分工

NVIDIA：MID360 网口驱动、USB-CAN/M2、前后 LD19、ZED、可切换视觉模型、Qt/RViz，以及最终 `/cmd_vel` 看门狗。

J6M：ROS master、Livox topic relay、FAST-LIO、高频里程计上的已知地图 ICP 定位、
MID360/LD19 避障融合、map_server、move_base + TEB、FOD 安全仲裁。

原 `/home/slam/robot_ws` 继续承担机场 GPS 模式；不要用其中的旧一体化脚本启动本双机模式。

当前视觉配置临时选择外部 `/home/slam/LocateAnything` 中的
`nvidia/LocateAnything-3B`；原 YOLO11-GAM 权重和运行环境未删除。视觉推理实际在
NVIDIA 执行，J6M 只接收 `/fod/detections` 和活动模型契约，不保存模型副本。
LocateAnything 当前只用于识别显示：其逐框置信度未校准、实测延迟为数秒，适配器
固定关闭运动资格并不向 J6M 提供可用的逐框深度。Qt 显示侧另以检测源帧时间戳匹配
ZED 注册深度和 CameraInfo，在框内做保守点云聚类、异常值剔除并显示簇深度中位数；
该显示结果只发布到 `/fod/vision/results`。恢复运动型视觉伺服前应把后端切回已验收的
YOLO，或先完成 LocateAnything 的标注校准和实时性验收。

Qt“视觉”页右侧的“视觉识别模型切换”可在 LocateAnything-3B（唯一 `trash` 类）、
YOLO11-GAM（`best6.pt`，五类）与 `detect and classify`（单类 YOLO11-GAM 检测后
YOLO11-cls 五材质分类）之间选择。只有视觉控制已停车、覆盖任务和 move_base
目标均为空、状态新鲜且里程计确认零速时才能应用；后台会再次独立校验模型 SHA、类别
契约和停车状态，然后执行双机完整冷重启。当前静态地图模式会保留，但一次性的
`--authorize-fod-motion` 不会跨重启继承。

## 第一次接线与配置

在 NVIDIA 主机：

1. WCH USB 转网口适配器（MAC `50:54:7B:E3:C9:10`）直连 MID360，NVIDIA 为 `192.168.1.50`，MID360 为 `192.168.1.112`。
2. ASIX USB 千兆网口（当前永久 MAC `6C:1F:F7:C4:96:B8`，USB `0B95:1790` / `000000000011CA`）接入 USB 扩展坞，扩展坞 RJ45 接交换机；J6M 也连接该交换机。NVIDIA 为 `192.168.10.50`，J6M 为 `192.168.10.100`。
3. USB-CAN、前 LD19、后 LD19 均连接 NVIDIA。
4. ZED 2 必须接在真正的 USB 3.x 数据口；`lsusb -t` 中视频端必须显示 `5000M`
   或更高，`480M` 只代表 USB 2.0 降级，SDK 不会正常出图。
5. 用 VS Code 编辑 [dual_host.env](/home/slam/robot_j6m_ws/config/dual_host.env)，不要猜串口。

当前 MID360 安装外参以底盘 `base_link` 为参考：X 向车头 `+0.20 m`、Y 为 `0.0 m`、Z 为 `+1.00 m`，姿态与车体同向。前后 LD19 分别位于
`(+0.46, 0, 0.20) m` 和 `(-0.46, 0, 0.20) m`，各自只保留朝车外的
`120°` 有效视场。更换安装位置后应同步修改 `dual_host.env` 中的外参，再重新部署并重启 J6M 栈。

导航避障点云当前会删除 `base_link` 中 `X=[-0.75,+0.75] m`、`Y=[-0.50,+0.50] m` 的 `1.5×1.0 m` 车体矩形内部点（含边界），保留矩形外且符合原有 `0.5–12.0 m` 距离和高度条件的点。矩形过滤在 MID360 外参变换之后执行，因此已包含雷达相对底盘前移 `0.20 m` 的偏置。范围由 `MID360_CROP_*` 配置控制。

识别串口：

```bash
cd /home/slam/robot_j6m_ws
./scripts/discover_devices.sh
```

本车已通过“只拔车头雷达”的方式确认固定 USB 物理口：`1-4.4` 为车头
LD19、`1-4.3` 为车尾 LD19。两只 CH341 的 USB 序列信息相同，因此使用物理口
生成可读的稳定别名；首次配置或重装系统后执行：

```bash
./scripts/install_dual_lidar_udev.sh
./scripts/install_zed_udev.sh
```

第二条命令一次性安装 ZED 的 `video` 组权限和开机后定向 coldplug 服务。该服务等待
Jetson 首轮 udev 枚举稳定后、NetworkManager 启动前，加载 ASIX/WCH、FTDI、CH341 和
UVC 驱动，并只对本车配置中 USB ID 与序列号匹配的网卡以及已知传感器重新触发 udev。
MID360 专用 WCH 网卡若仍无载波，还会只重置这一精确设备一次。它同时解决 ZED usbfs
（以及内核生成时的 hidraw）节点遗留为 `root:root 0600`、CAN/LD19 驱动未绑定和网卡
冷启动卡死的问题。SDK 4 可直接使用 f781 usbfs，因此 hidraw 未生成本身不作为启动失败条件。
它需要管理员密码；项目的一键启动命令本身不会静默提权。

为降低断电重启复发率，应先给交换机、J6M、MID360 和有源 USB 扩展坞供电并等待链路灯
稳定，再启动 NVIDIA；ZED 保持在原生 USB 3.x 数据口，两张网卡和串口设备保持当前物理口。
安装后可用 `systemctl is-enabled autolabor-zed-coldplug.service` 确认服务为 `enabled`。

运行配置使用 `/dev/autolabor/lidar_front` 和 `/dev/autolabor/lidar_rear`。
只要两个 USB 插口保持不变，`/dev/ttyUSBn` 如何变化都不会交换前后角色；改变接线后
必须重新做单设备插拔确认。确认后才把 `CAN_PORT_CONFIRMED`、
`DUAL_LIDAR_PORTS_CONFIRMED` 改成 `true`。

若输出包含 `IN_USE_BY_PID`，先明确停止占用该设备的旧 `robot_ws`/串口进程；新网关会拒绝冲突，不会自动杀进程。

交换机、J6M 和 MID360 均已连接、供电后，在 NVIDIA 首次配置时执行一次：

```bash
sudo /home/slam/robot_j6m_ws/scripts/configure_network.sh --apply
/home/slam/robot_j6m_ws/scripts/network_check.sh
```

该脚本先验证 MID360，再切换 J6M 地址；Wi-Fi 默认路由不会交给机器人网口。

## 一键启动和运行模式

每次启动先给交换机、J6M 和 MID360 供电，并确认 ASIX USB 网卡经扩展坞到交换机、
J6M 到交换机、MID360 专用 USB 网卡到 MID360 的链路灯均已亮。不要按可能变化的
`eth0/eth1/eth2` 名称判断 USB 网卡。启动器先按永久 MAC 识别；MAC 变化时只允许用配置中
完全匹配且唯一的 USB VID:PID + 序列号兜底，不会把另一个现存 `ethN` 误认成目标网卡。

以下命令均在 NVIDIA 主机执行。

启动器会先运行 `scripts/zed_camera_check.sh`，校验序列号、USB 3.x 速率和当前用户
访问权限；ZED 启用时还必须收到 `/fod_camera/image_raw` 与注册深度首帧才会写入
ready。可选 LD19 或检测结果短时缺流仍按降级告警处理，但“ZED 节点存在、实际无图”
不再算启动成功。

### 一键命令总览

以下命令使用绝对路径，可以从任意目录执行。末尾的 `</dev/null` 只表示启动命令不读取
终端标准输入；完整进程树仍由 transient `autolabor-dual-host.service` 托管，它不会
跳过模型、定位、运动授权、急停或其他安全检查。

无静态地图、无本次视觉运动授权：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start </dev/null
```

从完全停止状态加载最近静态地图，但不额外授予视觉运动权限：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start \
  --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest </dev/null
```

从完全停止状态加载最近静态地图，并且只为本次托管运行授予 FOD 视觉运动权限：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest --authorize-fod-motion </dev/null
```

这是当前推荐的视觉控制实验一键启动命令。Qt 窗口出现只表示图形进程已经拉起，不能据此
判断整条双机链启动成功；必须等待命令以状态 `0` 返回，并看到：

```text
Dual-host project is ready and managed by autolabor-dual-host.service.
```

在这行成功提示出现前不要发送 `/initialpose` 或进入视觉模式。若终端出现
`FAIL Summary: ... failures`，启动器会按 fail-closed 规则同步关闭 Qt、NVIDIA 网关和 J6M，
此时不是 Qt 闪退。先检查已有测试结果：

```bash
cd /home/slam/robot_j6m_ws
catkin_test_results --all build/test_results
```

修复并重新运行对应测试，确认汇总为 `0 errors, 0 failures` 后，再原样执行上面的一键启动
命令。不要删除失败 XML、修改 ready 标记或绕过 `health_check.sh` 强行启动。

如果双机栈已经运行，必须完整冷重启才能切换为同样的静态地图＋本次视觉授权模式：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --restart \
  --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest \
  --authorize-fod-motion </dev/null
```

查询状态和同步停止两端：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --status </dev/null
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --stop </dev/null
```

`--authorize-fod-motion` 是一次性运行覆盖：它让本次 NVIDIA/J6M 进程看到
`FOD_MOTION_ENABLED=true`，但不修改 `config/dual_host.env`。停止本次服务后该覆盖即失效。
该参数要求持久配置中的 `MOTION_ENABLED=true`，并要求
`runtime/motion_authorized.ok` 已由现场安全确认流程创建；缺少任一条件时启动会拒绝继续。
它只授予控制器通过全部预检后输出非零速度的资格，不会自动进入视觉行驶。静态地图模式
下的 move_base 导航和覆盖清扫仍必须根据车辆真实位置发送 `/initialpose` 并等待
`LOCALIZED`；视觉伺服本身使用本次启动的局部 `/odom`，不依赖 GPS 或全局定位。
由操作员在 Qt“视觉行驶模式”中点击“立即单独启动”后，最近有效深度 FOD 不小于
`5 m`、数据过期、急停或任一控制权/反馈检查失败时都保持零速。“启用智能控制”属于
可选相机图像质量控制，不是视觉行驶入口。Qt“视觉”页可在控制器停车时将目标锁定阈值调为
`0.25–0.95`（默认 `0.30`，冷重启恢复默认）。该页只对白色/浅色侧栏使用黑色文字；
相机画面、彩色按钮和安全告警等有色背景保留原字色。周期性 raw CAN 安全查询只由
NVIDIA 的 M2 驱动发出：平均尝试率限制为 `3.0 Hz`，周期加入 ±20% 去同步抖动，每项
安全状态最多重试 4 次后公平轮转；J6M 视觉控制器只监听回包，`8.00 s` 未更新即中止，
明确的不安全回复仍会立即中止。

### 视觉控制（定点清扫）实车使用

视觉行驶不会随开机或一键启动自动生效。完整条件是：主运动门为
`MOTION_ENABLED=true`、`runtime/motion_authorized.ok` 存在、本次启动带
`--authorize-fod-motion`、ZED/YOLO/注册深度/局部 odom/轮角/CAN 安全状态均新鲜，最后
再由操作员显式进入视觉模式。少一个条件都保持零速。

测试前必须确认车辆架空或位于封闭净空测试区、人员远离车轮、实体急停可用、CAN
物理端口已确认；当前默认接近和盲区速度均为 `0.20 m/s`。当前视觉命令不经过
move_base、TEB 或 costmap 障碍规划，MID360/LD19 的导航避障不会替视觉模式绕障或停车，
因此必须清空车辆前方至少 `7 m`，不能把本功能当作具备避障能力的自主导航。

推荐从完全停止状态启动：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start \
  --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest \
  --authorize-fod-motion </dev/null
```

若已启动但漏了 `--authorize-fod-motion`，不能在运行中补授权，必须用上文的
`--restart --map-set ... --authorize-fod-motion` 完整冷重启。启动返回
`Health check passed (--runtime)` 后，静态定位可能仍为 `WAITING_INITIAL_POSE`：这会阻止
move_base/覆盖导航，但不会单独阻止视觉伺服，因为视觉阶段只使用局部 `/odom`。不要把
这种视觉可用性误写成车辆已经完成 `map` 全局定位。

Qt 操作入口：

1. 打开“视觉”页，确认相机、YOLO、深度和视觉控制器状态均在线；
2. 需要时在停车状态调整“目标锁定置信度”，范围 `0.25–0.95`，默认 `0.30`；
3. 点击“立即单独启动”进入视觉模式；“启用智能控制”只控制相机图像质量，不会授权行驶；
4. 需要停车时点击“退出视觉模式并恢复相对导航”。

也可以在 NVIDIA 终端通过模式仲裁器操作。联动模式下不要直接调用
`/fod_visual_servo/set_enabled`：

```bash
cd /home/slam/robot_j6m_ws
source scripts/load_config.sh
source scripts/setup_env.sh

./scripts/fod_mode.sh status
./scripts/fod_mode.sh start
./scripts/fod_mode.sh watch
./scripts/fod_mode.sh stop
```

正常状态序列为：

```text
DISABLED -> PRECHECK -> ACQUIRE -> APPROACH
         -> EDGE_ARMED -> LOSS_CONFIRM -> STEER_SETTLE
         -> BLIND_ADVANCE -> FINAL_STOP -> COMPLETE
```

常用只读监控：

```bash
rostopic echo /fod_visual_servo/status
rostopic echo /fod_navigation_mode/status
rostopic echo /cmd_vel_safe
rostopic echo /cmd_vel
```

`/fod_visual_servo/status` 中应重点看 `state/reason`、`raw_can_age_sec`、
`detection_age_sec`、`odom_age_sec`、`wheel_angle_age_sec`、`target_depth_m`、
`horizontal_error`、`vertical_fraction` 和实际命令。原始 CAN 四项查询只由
`/m2_driver` 负责，平均尝试率 `3.0 Hz`、周期抖动 ±20%、每项最多重试 4 次；视觉端
`8.0 s` 没有新回复会停车，明确急停回复立即停车。同一物体偶尔被模型同时标成两个类别
时，只有框 IoU 不低于 `0.80`、锚点距离不超过画面对角线 `0.02` 且注册深度相差不超过
`0.15 m` 才作为同一几何目标继续跟踪；两个真实相邻目标仍按歧义停车。

当前底部门限仍保持 `bottom_fraction=0.88`。如果目标在到达画面底部中央前被车体前缘
遮挡，状态会以 `target was lost before reaching the bottom gate` 中止；按现场决定通过
调整相机安装角度保证目标能进入底部门，不要为绕过机械遮挡直接放宽该安全门。任何
`ABORT` 都会锁存零速；排除原因后先执行 `./scripts/fod_mode.sh stop` 明确退出，再开始下一轮。

整套双机栈停止仍只用统一入口：

```bash
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --stop </dev/null
```

### AI 语音控制与本地 ASR

Qt“AI语音控制”页使用 NVIDIA 本机的 OpenAI Whisper，不把原始音频上传
云端。`sweeper_ai` 通过隔离的 JSON-lines 子进程托管 ASR worker；worker 没有 ROS、
DeepSeek 密钥、Qt 会话令牌或 MCP 控制能力。Qt 可在 `small`、`medium`、`large` 三档
切换，默认 `medium`；界面中的 `large` 对应官方 `large-v3`。checkpoint 位于：

```text
/home/slam/robot_j6m_ws/runtime/asr/models/small.pt
/home/slam/robot_j6m_ws/runtime/asr/models/medium.pt
/home/slam/robot_j6m_ws/runtime/asr/models/large-v3.pt
```

首次安装只在 NVIDIA 执行，安装器会创建 `runtime/asr/venv`，复制并校验适配当前
JetPack 的 CUDA PyTorch wheel，固定 Whisper Git commit，再把模型先下载到 `.partial`；
三套 checkpoint 都只有 SHA-256 校验通过才原子改名。运行时缺少或损坏目标模型时切换
失败并保留当前 worker，绝不
联网补下载：

```bash
cd /home/slam/robot_j6m_ws
bash ./scripts/install_whisper_asr.sh
```

真实 AI/ASR 配置仍写在 Git 忽略的 `src/sweeper_mcp/config/sweeper_mcp.yaml`，文件必须由
当前用户持有且权限为 `0600`。`input_device: auto`（空值也等价）会在 worker 启动时
只读枚举 ALSA capture 端点，优先选择 USB 麦克风，并使用稳定的 ALSA card ID 生成
`plughw:CARD=<id>,DEV=<n>`；枚举不会打开麦克风。仍可显式配置设备，且不会把
`auto_null.monitor`、Jetson APE fabric 端点当成实体麦克风。实际采集仍必须经过 Qt
语音授权。

每次启动后三项授权和智能语音模式都默认关闭。操作员先确认“授权语音输入”，再在两种
互斥模式中选择。模型下拉框不需要语音授权，但只能在未录音、未监听和未识别时切换：

1. 手动录音：点击“开始录音”，说完后点击“停止并识别”；只采集两次点击之间的音频。
2. 智能语音：另行点击“启用智能语音”并在默认选择“否”的确认框中确认。此后持续监听，
   本地能量 VAD 自动判断起句和停顿，停顿约 `0.8 s` 后形成一句并交给当前所选模型；无需为
   每句话手动开始或结束。关闭智能语音后立即停止采集并废弃该会话的排队及迟到结果。
3. 未授权“AI 语义解析”时，两种模式的识别文字都只在本机显示；授权后，智能语音识别出的
   每句非空文本进入有界 FIFO，由云端逐句拆解并严格串行处理。
4. 未授权“AI 控制”时，只显示完整计划和解析结果；三项授权均满足且 Qt 心跳新鲜时，
   MCP 变更类工具才可能按顺序执行，并继续受定位、急停、CAN、避障和运动门限制。授权状态
   变化会清除尚未提交的旧语音，避免旧口令在之后获得更高权限。

智能语音是“持续监听 + 自动断句 + 逐句批量识别”，不是边说边显示 partial token 的流式
Whisper。正常情况下，从说完到开始云端请求还需等待断句静音以及当前模型的本地推理时间。

手工输入不需要语音授权，但发送云端仍需要 AI 语义解析授权，实际控制仍需要 AI 控制
授权。撤销语音授权、关闭 Qt 或心跳超时会停止手动录音或智能监听，并丢弃迟到结果。ASR、Qt、AI
规划和 MCP 客户端都只运行在 NVIDIA；仅修改这些组件时重新构建 NVIDIA 工作区即可，
无需执行 `deploy_j6m.sh` 或切换 J6M release。

AI 地图绝对导航支持用户明确提供的任意 `map` x/y/yaw，以及固定语义的“地图坐标原点”
`(0,0,0°)`。`/map` 是锁存静态数据，只要求成功接收并保持有效缓存，不使用秒级消息新鲜度；
执行前改为核对本地完整 OccupancyGrid 摘要与新鲜 `/coverage/status.map_digest`，再检查
`LOCALIZED`、地图范围、原点旋转和占用栅格，并在发布导航请求前复核地图未切换。AI 使用
预先生成唯一 GoalID 的 `/navigation_goal/action_request`，由 J6M 安全桥
改写本机时间后转发到 `/move_base/goal`；只有同 ID 的 action 回显和唯一状态均匹配才算
接受，精确撤销和心跳租约也只作用于该 AI ID。Qt/RViz 手工设点仍保留原 simple-goal 入口。
动态定位、控制模式、里程计和覆盖状态仍保留各自的新鲜度安全门。未提供精确坐标
的“基地、充电点、起点”等名称不能由云端自行编造。

### 无静态地图启动

日常建图、录包或只需要 FAST-LIO 增量里程计时，使用默认启动：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_dual_host.sh
```

不传 `--map-set` 时不会加载历史 PCD，也不会启动 map_server 和已知地图定位器。
FAST-LIO 在本次启动建立的 `camera_init` 坐标系内输出增量里程计。这也是 Qt
“录入静态地图”功能要求的运行模式。

### 实验性可选项：加载静态地图启动

三维已知地图定位加二维导航目前按实验功能交付，只有显式传入 `--map-set` 才会启用；
默认启动不会读取历史地图，也不会启动定位器或改变现有无图导航基线。

注意：下面的一键命令只负责加载地图并拉起定位链，不包含车辆当前初始位姿。
地图文件本身不知道车辆本次上电后停在旧地图的什么位置，因此每次静态地图模式
冷启动后都必须重新发送 `/initialpose`。

从完全停止状态首次加载最近完成的地图集：

```bash
./scripts/start_dual_host.sh --start \
  --map-set global_maps/map_sets/latest
```

如果双机栈已经在运行，切换到静态地图模式必须冷重启：

```bash
./scripts/start_dual_host.sh --restart \
  --map-set global_maps/map_sets/latest
```

启动器会验证 `manifest.yaml` 和三类地图是否完整，将选中的地图集原子同步到 J6M，
然后启动以下链路：

```text
map_server ------------------------------> /map（move_base 二维静态地图）
FAST-LIO --------------------------------> camera_init -> body
fast_lio_localization + map_3d/map.pcd --> map -> camera_init
静态外参 --------------------------------> body -> base_link
```

默认给 move_base 加载 `map_fused_2d`。需要只使用前后 LD19 建成的二维图时：

```bash
./scripts/start_dual_host.sh --restart \
  --map-set global_maps/map_sets/latest \
  --static-map-source lidar2d
```

`--static-map-source` 只影响 move_base 使用的二维占据图；三维重定位始终加载同一
地图集的 `map_3d/map.pcd`。此模式不启动 AMCL，map_server 也不负责定位。

### 静态地图启动后的必做步骤：设置初始位姿

一键启动命令返回成功，只表示双机节点和传感器链已经就绪，不表示车辆已经完成
全局定位。此时定位器正常状态应为 `WAITING_INITIAL_POSE`，导航速度门保持关闭。

推荐在 Qt 内嵌 RViz 中操作：

1. 打开“综合”页；静态地图加载后，RViz 会自动缩放到完整二维地图。若视角被拖走，
   点击地图上方的“① 显示整张地图”。
2. 点击“② 设置初始位姿”（等价于 RViz 工具栏 `2D Pose Estimate`）。按钮只选择
   工具，不会自动发布位姿。
3. 在地图中车辆实际所在位置按下鼠标，并沿车头方向拖动后松开。这里指定的是
   `base_link` 在 `map` 中的二维位置和朝向，不是 MID360 的安装位置。
4. 等待 `/fast_lio/localization_status` 变为 `LOCALIZED`；Qt 随后自动进入二维跟车
   视角，也可点击“③ 跟随车辆”。该模式仍以 `map` 为 Fixed Frame，只把视角目标改为
   `base_link`，因此局部代价地图和实时点云会随车体居中更新。
5. 定位完成后再发送导航目标。

查看三维先验地图是可选操作：点击“④ 显示静态三维先验”后，界面按需订阅
`/fast_lio_localization/prior_map` 并切换为可旋转视角；再次点击恢复二维全图。
点击“① 显示整张地图”或“② 设置初始位姿”也会自动返回二维视角。三维显示默认关闭，
不会改变上述二维初始位姿和导航流程。该 PCD 是锁存的已知先验，本来不会更新；
实时匹配点云由 `/fast_lio_localization/aligned_scan` 显示。

综合页和清扫页使用亮绿色二维矩形显示 move_base 当前生效的车辆安全轮廓。矩形来自
`/move_base/local_costmap/footprint`，会按 `base_link` 位姿移动并自动跟随
导航 footprint 参数；当前基础车体为 `1.04 × 0.70 m`，加 `0.10 m` padding 后外框约为
`1.24 × 0.90 m`。静态地图冷启动时必须先完成初始位姿，并有新鲜里程计/TF 建立
`map -> base_link`，车框才会出现在地图中的真实位置；暂时没有数据流时不显示是正常的。
综合页右侧“FAST-LIO / map 全局定位”栏同时显示里程计局部坐标和只读的
`map -> base_link` 全局 `X/Y/Yaw`；全局定位未完成、状态过期或 TF 不可用时明确显示等待
原因，不会把局部里程计坐标冒充为 map 坐标。

初始位姿前 RViz 视角固定在 `map` 并显示全图，不跟随尚未接入全局坐标系的
`base_link`；因此不应再出现必须盲点位姿的黑色空视图。
`operator_gui.launch` 会在静态地图模式发布无害的
`map -> autolabor_map_display_anchor` 静态叶子帧，使 RViz 在定位前能识别并渲染
`/map`。该叶子不连接 `camera_init`、`base_link` 或任何车辆坐标系，不代表车辆初值；
真实的 `map -> camera_init` 仍只能由已知地图定位器在接受人工 `/initialpose` 后发布。
Qt 还会在收到 `/map` 后检查 RViz `MapDisplay` 自身加载的宽、高和分辨率；若内嵌
RViz 在初始化窗口时漏接了 `map_server` 的锁存消息，会自动重新订阅，最多重试 5 次。
只有 `/autolabor_operator_gui/map_display_status` 报告 `READY`，一键启动的运行态自检
才会把二维地图判为已渲染，避免仅凭 GUI 自己的地图订阅误报“完整显示”。
静态模式下 Qt 默认以 `0.55` 透明度显示
`/move_base/global_costmap/costmap`，并把 `0.90` 透明度的滚动局部代价图放在其上层；
综合页顶部“⑤ 显示/隐藏全局代价图”可直接切换，不再要求打开 RViz 调试面板。右侧栏
还显示全局代价图的尺寸、分辨率、消息年龄或缺流原因，从而区分“显示层关闭”和
“move_base 尚未发布 topic”。

地图上路线颜色固定为：青色是覆盖条带预览，橙色是当前直接 Hybrid A* 完整连接
（包含全部前进、倒车和换档段），蓝色是当前交给 move_base 的全局参考路线，红色是 TEB
当前优化的局部轨迹，绿色是覆盖任务实际执行记录。Hybrid 阶段的速度权威同样是 TEB；
安全 mux 只执行 fail-closed 检查，不计算路径跟踪速度。橙/蓝/红路线只在相应目标活动期间显示，终止或取消后 Qt 会清空
RViz 缓存，避免旧路线残留。静态
模式的滚动局部代价地图同时加载二维静态层和实时 `/scan` 障碍层；TEB 最终可行性检查
扩展到局部时域最多前 51 个姿态，并将静态障碍和未知区 footprint 判为不可行。预测
足迹仅超出滚动窗口时不再误报碰撞，但当前姿态位于窗口外仍保持 fail-closed；静态模式
还在全部静态、实时障碍和膨胀层之后运行 `UnknownSpaceGuardLayer`，把原始地图 `-1` 格及
地图边界外重新写为 `NO_INFORMATION=255`；因此雷达清空射线不能把静态未知区变成自由区。
local costmap 为 `20 × 20 m`，障碍标记/射线清除距离为 `10/11 m`；TEB 普通导航、首线
入场和扫掠的全局计划前视为 `4 m`，Hybrid 转场后台诊断前视为 `2 m`。静态导航额外以
`control_look_ahead_poses=2` 跨两个优化轨迹间隔提取控制指令，减少单个 10 Hz 周期的
小幅左右反向修正，同时保留完整转向和避障能力。move_base仍以`1 Hz`重做全局路线、TEB
仍以`10 Hz`做局部优化；这两个频率是正常在线规划机制，不由前视距离触发。

静态层中的占用格始终比实时障碍层的清空射线更权威：`/scan` 可以清掉此前由实时传感器
标记的动态障碍，却不能把 `/map` 里已经建成障碍的格子改为空闲。否则同一逻辑也会把真实
墙体清掉。因此，建图时的临时物体需要在对应 map-set 中修订，不能靠局部代价图自动猜测。

也可以从命令行发布初值。以下示例仅适用于车辆回到该地图的建图起点、车头方向与
建图开始时一致的情况。`/initialpose` 表示 `base_link` 的位姿，定位器会根据
`MID360_SENSOR_X/Y` 自动换算到 FAST-LIO 的 `body`，因此不要手工减去雷达安装偏置。
当前地图集 `map_20260822_slice_zm040_hw020_final` 记录的首帧底盘位姿约为
`(-0.180, -0.047, 0.0015 rad)`；车辆确实位于建图起点附近且车头方向一致时，可以
发送近似初值 `(0, 0, 0)`：

```bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
"header:
  frame_id: map
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

其中四元数 `z=0,w=1` 表示 `yaw=0`；其他朝向应使用
`z=sin(yaw/2), w=cos(yaw/2)`。如果车辆不在建图起点，必须填写其在二维地图上的
实际近似位置，不能照抄示例。初值不需要厘米级精确，但位置和朝向必须落在 ICP
能够收敛的邻域内。

定位器收到初值后执行粗、精两级 scan-to-map ICP。观察定位状态和 TF：

```bash
rostopic echo /fast_lio/localization_status
rosrun tf tf_echo map camera_init
```

只有连续两次 ICP 质量检查通过、状态消息以 `state=LOCALIZED;` 开头时，导航速度门
才会放行 move_base 的速度。状态退化、丢失或超时会立即输出零速并取消当前
move_base 目标，重新定位后也不会自动恢复旧目标。
当前 `start_dual_host.sh` 没有 `--initial-pose` 参数，也不会复用上一次初值，这是为了
避免车辆被移动后仍套用旧位姿造成错误定位和危险导航。

### 覆盖清扫 V1

#### 可用条件与安全边界

覆盖清扫只在带 `--map-set` 的静态地图模式下启用。无图启动时，Qt“清扫”页的
框定、规划参数和开始按钮全部置灰；即使 ROS 图中偶然出现其他 `/map`，界面仍同时
检查一键启动传入的 `static_map_mode`，不会把无图建图模式误判为可清扫模式。

界面只有在以下地图身份链全部就绪后，才允许框定、保存、载入或管理区域：

- 本次一键启动显式加载了静态地图；
- Qt 已收到 `/map`，内嵌 RViz 的 MapDisplay 已核对宽、高和分辨率并报告 `READY`；
- J6M `/coverage/status` 新鲜，且其中的地图 SHA-256 与 Qt 当前区域库一致。

“开始覆盖清扫”和“开始队列清扫”还要求全局定位为 `LOCALIZED`、底盘反馈与急停状态
正常、`/scan` 新鲜、MID360 和前后 LD19 均参与融合，并且没有另一个覆盖任务占用
move_base。按钮置灰时先看右侧“全局地图、底盘执行门、障碍感知、详情”，不要绕过 Qt
直接调用服务。若任务已经活动后地图显示掉线，暂停、跳过和取消等安全出口会在后端状态
仍可确认时继续保留，不会因为普通编辑功能置灰而一起消失。V1 只执行覆盖导航，不控制
主刷、边刷、风机或喷淋。

#### 单区域：框定、规划和执行

推荐操作流程：

1. 按上一节发送车辆真实 `/initialpose`，等待定位状态为 `LOCALIZED`。
2. 打开 Qt“清扫”页，点击“框定覆盖清扫范围”。左侧专用 RViz 会自动切换到
   `Publish Point`，可在全局地图上连续点选 `3–4096` 个顶点。
3. 可使用“撤销一点”逐点修改，或随时“取消框定”。点击“确认区域并生成轨迹”后，
   首尾自动闭合；自交、重复边、面积过小和非有限坐标都会被拒绝。
4. 检查青色覆盖条带、可覆盖面积、不可覆盖估算和预计总时间。界面不再把条带之间的
   直线连接画成可执行路径；开始执行后，橙线显示当前直接 Hybrid A* 完整连接，蓝线显示
   move_base 的当前全局参考，红线显示 TEB 实时局部轨迹。每个区域第一条线和跨区后的新
   区域首线使用一个 Navfn + TEB 普通导航 action；只有同一区域换行使用直接 Hybrid A*，
   按 cusp 拆为固定档位 action 后仍由 TEB 闭环跟踪。默认有效
   宽度为 `1.00 m`、重叠率 `15%`，所以默认车道中心距为 `0.85 m`。规划参数还包括前进
   速度 `0.80 m/s`、倒车速度 `0.30 m/s`、最大转弯角速度 `0.60 rad/s`、最大线/角
   加速度 `1.00 m/s² / 0.50 rad/s²`、换向附加时间 `1.0 s` 和每段交接附加时间
   `0.5 s`，同一区域线间 Hybrid A* 异常重规划重试间隔默认为 `1.0 s`，可在 `1–10 s`
   内调节；有效缓存不会再按这个间隔主动替换。
   最高前进速度可在 `0.10–1.60 m/s` 内选择；NVIDIA 最终看门狗的
   `1.70 m/s` 是绝对软件边界；任务开始时还会读取 VCU 实际速度上限，不能把任一上限
   当作室内推荐巡航速度。此时可点击“取消已生成轨迹”丢弃区域和预览，再重新框定。
5. 点击“开始覆盖清扫”。后端会按车辆当前全局位置重新做已知自由空间连通域裁剪，
   再导航到兼顾距离、当前车头方向和最小转弯半径的合适轨迹端点并执行覆盖。侧栏会先
   显示“正在复核起点与安全门”，通过后才进入“前往起点”。

“规划参数”中的所有数值都会参与秒级时间代价和/或 TEB 执行约束。计划进入 `READY` 后
整组参数会锁定；若要修改，必须先“取消已生成轨迹”并重新生成，不能把新数值套在旧路线
上。每次在 Qt 修改参数都会立即通过 `QSettings` 写入 NVIDIA 当前用户配置；400 ms 内没有
继续修改后，Qt 调用 `/coverage/set_planning_defaults`，由 J6M 在同一事务内校验整组参数、
更新 TEB/覆盖管理器运行态，并原子改写当前 release 的 `coverage.yaml`。只有 J6M 回传的
生效值与文件写入都成功，清扫页才显示“已同步”，否则确认区域、单区启动和队列启动均保持
锁定并自动重试。Qt 重连或新 release 启动时以当前用户的 `QSettings` 为优先值重新下发，
避免部署后回到旧参数。其中前进/倒车速度、允许倒车、最大角速度、线/角加速度写入 TEB 的
普通导航基线，供普通点到点、首线/跨区入场和覆盖任务共同使用；宽度、重叠率、两个时间
附加项和 Hybrid 异常重规划重试间隔同步成为后续覆盖规划默认值。下一次 Qt 单区
规划或队列启动直接冻结当前界面的完整参数组；AI
清扫若没有显式指定某一参数，也会在提交任务时读取同一份最新 `QSettings`，不再使用一套
独立的 `0.8 m/s` 等固定默认值。显式 AI 参数只覆盖对应字段。该偏好不是地图区域记录的
一部分。点击“恢复默认参数”并二次确认后，J6M 从只读
`coverage_factory_defaults.yaml` 恢复 `1.00 m / 15% / 允许倒车 / 0.80 m/s` 等出厂基线；
服务成功后 Qt 才更新界面和 `QSettings`。队列模式没有预先生成整批轨迹，开始时会统一
冻结当时显示的全部参数。

取消按钮会随阶段改变文字和作用；除尚未点任何顶点的“取消框定”外，都要在弹框中再次
确认：

| 当前阶段 | 按钮/操作 | 结果 |
| --- | --- | --- |
| 正在点选顶点 | “取消框定” | 只清空尚未提交的顶点，不调用运动服务 |
| `PLANNING` | “取消轨迹生成” | 作废本次异步请求；迟到结果不能恢复旧草稿 |
| `READY` | “取消已生成轨迹” | 丢弃区域、计划 ID 和预览，不发送导航目标 |
| `PREPARING` | “取消覆盖启动” | 中止在线安全复核，保证不会再提交该任务目标 |
| 活动单区域任务 | “取消覆盖清扫” | 取消当前 move_base 目标并终止本次任务 |
| 活动队列 | “取消全部队列清扫” | 取消当前目标和整批，后续区域不再执行 |

单区域完成、部分完成、失败或取消后，后端会清空锁存的区域、青色预览和绿色实走记录，
Qt 同步清空草稿和计划。此后可直接再次点击“框定覆盖清扫范围”，不需要重启 Qt 或双机栈。

#### 保存和复用已知清扫区

当前多边形成功生成轨迹后，可点击“保存为已知清扫区”：

1. 在“区域命名为：”弹框中输入名称；支持汉字和字母，去除首尾空格后长度为
   `1–80` 个字符，不能包含控制符或换行。
2. 同一地图内名称按大小写不敏感方式保持唯一，例如 `Area-A` 与 `area-a` 视为同名。
3. 核对名称和顶点数，在第二个确认框中点击“是”后才真正写入。

保存内容只有 UUID、名称、地图身份、`map` 坐标顶点、版本和时间信息，不包含 `plan_id`、
生成轨迹、宽度、重叠率、速度、队列、清扫进度或自动恢复指令。再次载入记录时，它只
恢复成可编辑多边形草稿，仍需目视核对并点击“确认区域并生成轨迹”；载入或编辑不会自动
覆盖原记录。

J6M 对实际 `/map` 的 frame、尺寸、分辨率、完整 origin pose 和全部栅格计算稳定
SHA-256。NVIDIA 以当前规范化后的 map-set 目录作为第一层隔离边界，并在文件内复核该
摘要和 `source_mode`；`map_sets/latest` 会先解析到它实际指向的 map-set。即使地图 A 与
地图 B 的二维栅格摘要完全相同，只要位于不同 map-set 目录，也不会共用保存区域。地图
上下文变化时，Qt 会清空本地草稿、预览和当前会话队列。

区域库的唯一可写副本位于 NVIDIA：

```text
global_maps/map_sets/<map-set>/coverage_regions/<source_mode>/regions.json
```

它和该地图的 YAML/PGM/PCD 一起位于同一个 map-set 大目录，使用文件锁、写前文件指纹
复核和原子替换；损坏或地图身份不符的文件会拒绝加载且不会被覆盖，持有旧快照的进程
发现区域库已更新时会拒绝写入并重新加载。旧版集中目录
`global_maps/coverage_regions/v1/...` 仅作只读兼容来源：只有其 JSON 内记录的规范化
`map_source` 与当前 map-set 完全一致时才会自动复制到新位置；旧文件不会自动删除，来源
不一致时也绝不会按摘要猜测迁移。区域库属于地图运行数据，受 `global_maps/.gitignore`
保护，不随源码 Git 提交；备份源码仓库并不等于备份已知清扫区。

#### 已保存区域和多区域队列

点击“选择已保存区域 / 管理队列”后，左侧是当前地图的保存记录，右侧是本次会话队列：

- “载入为可编辑区域”：恢复到地图上，确认后可按单区域执行；不会立刻发车；
- “加入清扫队列”：加入队尾，同一 UUID 在一批中不能重复；
- “上移 / 下移”：改变区域执行顺序；
- “从队列移除”：只影响本次队列，不删除保存记录；
- “删除区域记录”：从区域库永久删除该记录，但不删除地图。正常同一 Qt 会话中，界面会
  阻止删除已排队或正在执行的区域，必须先从队列移除或取消任务；
- 上述载入、加入、调序、移除和删除操作均有确认弹框，选择“否”不会更改状态。

队列非空后，设置当前有效宽度、重叠率、前进/倒车速度、线/角加速度、最大转弯角速度、
换向/分段附加时间和“仅在转场允许 TEB 尝试低速倒车”，
再点击“开始队列清扫”。启动只确认一次；单批后端最多接受 `100` 个区域。J6M 收到后会
冻结区域多边形、顺序和整批参数，批次结束或取消前不能在 Qt 修改该队列。区域记录本身
不固化规划参数，所以同一保存区域在不同批次可以使用不同时间模型与运动约束。

队列由 J6M 覆盖管理器一次接收并逐区即时规划，不依赖 Qt 在区域间持续发服务。一个区域
`COMPLETED` 或 `COMPLETED_PARTIAL` 后自动进入下一区域，`SKIPPED` 计数后也继续；明确
`FAILED` 或整批 `CANCELED` 会停止余下区域。“跳过当前区域”只终止当前区，“取消全部队列
清扫”终止整批。`/coverage/active` 从批次接受到最终清理始终为 `true`，因此换区间隙普通
导航也不能抢占 move_base。跳过后继续的前提是规划器所有权和 TEB 参数已安全清理；若
清理失败，fail-closed 的 `FAILED` 优先并停止整批。

保存区域会跨 Qt 和完整双机栈重启加载。尚未下发的队列只在 Qt 内存中，重启 Qt 后丢失；
已接受的批次驻留 J6M 内存，仅关闭或重启 Qt 时仍可能继续执行，而重启 J6M 或完整双机栈
后不会自动恢复、更不会自动发车。已被后端接受的整批任务在完成或取消并收到终态后，Qt
会清空本次队列；若尚在提交阶段且后端未接受，队列可以保留供重新确认。

活动任务期间直接关闭 Qt 不会取消 J6M 后端任务；若希望车辆停止，必须先在清扫页完成
取消并确认终态，再关闭界面。当前“暂停”和“跳过”的按钮请求没有完全互锁，因此操作员
必须在点击开始、暂停、跳过或取消后等待上一条命令返回，并确认 `/coverage/status` 已更新，
再发下一条命令。若 Qt 在活动批次中异常重启，本地队列列表无法恢复，界面也无法识别后端
尚未执行的全部区域；此时先取消整批或等待终态，再删除区域记录或重组队列。

#### 启动事务与任务所有权

单区域 `/coverage/start` 采用两阶段事务：管理器先锁定计划 ID、地图摘要和区域快照，然后
在不占用状态锁的情况下执行 VCU/Hybrid A*/TEB 在线核对及连通域重规划，使 `/scan`、双 LD19、定位
和里程计回调能继续刷新；最后重新核对同一计划、同一地图和全部新鲜度安全门，才原子地
声明该区域活动。
因此耗时重规划不会再把仍在正常流动的 `/scan` 误判为过期并让任务刚接受就永久暂停。
准备期间若地图或计划变化，启动会明确失败且不会提交任何 move_base 目标。
轨迹生成、启动准备和活动任务都可从清扫页取消。规划与启动各自使用不可复用的请求代际
令牌，取消后的迟到回调不能恢复旧计划；活动任务完成、部分完成、失败或取消后，后端会
失效计划 ID，并把区域、青色条带和绿色实走记录的锁存消息清空。Qt 同步清除本地草稿，
随后可直接开始下一次框定，无需重启。

队列的 `/coverage/start_batch` 返回 `accepted=true` 只表示 J6M 已冻结队列快照并取得任务
所有权，不表示第一个区域已经规划成功或车辆必然发车。管理器随后按顺序对每个区域执行
即时规划和同类在线复核；第一个区域也可能在接受队列后以 `FAILED` 结束，因此必须继续观察
`/coverage/status`，不能只看启动服务返回值。

Qt 与 AI 在调用前都会生成 `coverage-batch-<32hex>` 请求 ID，J6M 原样用作 batch ID。
同 ID、同参数重试只回放原结果，不会生成第二个任务；同 ID、不同参数会拒绝。服务响应
丢失、取消先于启动线程完成或界面丢弃迟到响应时，客户端调用 `/coverage/cancel_batch`
精确建立 tombstone 或取消该 ID。旧 ID/foreign ID 不会取消当前其他批次；只有该批次已
证明从未启动，或已完成精确 goal 终结、规划器/TEB 恢复和 owner 释放后，界面才清空 ID。

覆盖状态的主要含义如下。队列换区时单区 `active` 可以短暂为 `false`，但
`batch_active` 和 `/coverage/active` 仍保持为 `true`，这不是任务已经结束：

| 原始状态 | Qt 显示 | 含义 |
| --- | --- | --- |
| `IDLE` | 等待框定 | 没有可执行计划 |
| `PLANNING` | 正在生成覆盖轨迹 | 对多边形做地图裁剪和条带生成，可取消 |
| `READY` | 轨迹已就绪 | 只有预览，尚未发送 move_base 目标 |
| `PREPARING` | 正在复核起点与安全门 | 按最新位姿重规划并在线核对底盘、Hybrid A*、TEB 和感知 |
| `GOING_TO_START` | 前往起点 | 每个区域首线使用一个 Navfn + TEB 普通导航 action，不计覆盖轨迹 |
| `TRANSITING` | 转场中 | 从实时位姿直接 Hybrid A* 到同一区域下一条清扫线入口 |
| `SWEEPING` | 覆盖路线执行中 | 使用固定清扫条带作为全局参考线 |
| `WAITING_OBSTACLE` | 等待动态障碍 | 当前分段无可行解，按有限次数等待和重试 |
| `PAUSED` | 已暂停 | 任务保留，但当前 move_base 目标已取消或不再推进 |
| `COMPLETED` / `COMPLETED_PARTIAL` | 已完成 / 部分完成 | 全部可执行分段完成，或仍有有限次重试后阻塞的分段 |
| `CANCELED` / `FAILED` | 已取消 / 失败 | 任务终止；队列失败时不再执行余下区域 |

#### 路径生成、排序和执行架构

区域多边形表示“需要覆盖的面积”，不是车辆地理围栏。静态障碍、未知格和与车辆不连通
的地图岛会被裁掉并计入不可覆盖面积；转场可以离开区域，但只能经过地图中的已知自由空间。

当前正式架构不预先计算任务级可执行全局轨迹。规划器先按几何完整度选择清扫角度，再对
已选清扫线用无障碍 Dubins 曲率长度、速度/加速度、角速度以及换档和停车时间构造秒级代理，
以有界 beam search 联合决定清扫线顺序和方向。这个任务层不调用栅格 A* 或 Hybrid A*，
也不生成条带间连接路径；它避免把相邻清扫线都定成同向而产生接近 360° 的大前进圈。

每个区域首线直接发送一个 Navfn + TEB 普通导航 action。只有同一区域换行时，管理器才从
实时 `map -> base_link` 位姿直接 Hybrid A* 到最终下一入口。完整带符号连接按 cusp 拆为
固定档位 move_base action，每段由 TEB 闭环跟踪；管理器由新鲜 M2 `/odom` 确认线速度
不高于阈值后才提交下一档位。action 阻断、无进展、连续路径偏离或入口超差时，才精确取消
当前 generation 并从实时位姿到最终入口整段重算。

Hybrid TEB 安全 mux 不会仅因收到一条连接就转发非零速度。每条新路径先撤销旧许可，move_base
全局插件以 1 Hz 在最新 costmap 上校验当前位置偏离与未来 3.0 m footprint，发布的新鲜
许可为 true 后才执行；许可为 false 或超过 1.5 s 立即零速。mux 不计算任何路径跟踪速度，
实际控制仅来自 TEB。插件不另建只对 TEB 可见的
第二条路径，避免恢复双重重规划权威。

move_base 的全局周期仍为 `1 Hz`，但
`CoverageGlobalPlanner/hybrid_replan_every_cycle=false`；一个已经交接且仍有效的连接不会
只因时钟触发而无条件替换。相同参数下无条件 1 Hz 对照没有缩短 A 区转场，却产生约 6 倍
Hybrid 搜索并挤占控制/地图循环，因此没有作为默认值。

`CoverageGlobalPlanner` 的普通点到点和每区首线模式仍委托 Navfn。区内换行使用
`MODE_HYBRID_TRANSIT`，清扫线使用 `MODE_ENFORCED_SWEEP`；模式、任务 owner、plan ID、
segment generation 或端点不匹配时 fail-closed，不会静默退回 Navfn 捷径。每个分段先通过
同步服务完成这一交接，之后 topic 只能刷新同一代际。

Hybrid A* 以 `(x,y,yaw,档位,转向档)` 搜索，使用 Reeds–Shepp 解析候选、障碍启发式和
恒曲率格点运动，解析与格点部分都按 `0.10 m` 稠密输出并检查完整 footprint。覆盖管理器、
Hybrid、全局规划器和 TEB 统一使用 `0.65 m` 轴距与 `1.35 m` 最小转弯半径；对应前轮
转角约 `25.71°`，比 VCU 约 28° 的机械上限保留约 `2.29°` 裕量。TEB 的转弯半径是软
优化边，因此最终 Twist 另硬限制 `|omega| <= |v| / 1.35`，零线速度时也不会残留原地角速度。

Hybrid 路径连续 3 个样本横向偏离超过 `0.35 m`，或相对局部路径航向偏离超过
`0.55 rad` 时，只取消当前任务 generation 的 action；确认 terminal/零速后从实时位姿
重算。跟踪器保留路径档位和有序姿态，并在换挡前强制零速；TEB 在 Hybrid 阶段保留后台
诊断，但不能重新取得速度权威。失败范围由偏离、无进展、有限重试和安全暂停约束。

每条清扫线启动前有独立硬门：距入口、横向误差、航向误差必须分别不大于
`0.40 m / 0.40 m / 0.436332 rad（25°）`。任一超差就保持 `TRANSITING`，不会短暂进入
`SWEEPING`。入口恢复仍瞄准原清扫线起点，但规划代价采用前进/倒车等效速度
`0.20/0.80 m/s` 和 `0.15 s` 换档代价以优先短倒车修正；最终入口搜索区域为
`0.30 m / 0.349066 rad（20°）`，实际交接仍使用外层 0.40 m / 25° 合同，中间 cusp
保持 `0.25 m / 0.20 rad`。候选总长超过
`4.0 m` 或累计前进超过 `1.20 m` 会被视为“大幅前进绕行”并拒绝，失败不会跳过原线。
对恰好发生在 sweep 启动后的扰动，清扫线最初 `0.75 m` 还有连续 3 样本的兜底恢复门。

清扫完成不是命中终点圆。补充门要求从本线入口建立连续历史、完成至少 `90%` 有向进度、
越过出口平面 `0.02 m`、满足 `0.30 m / 0.35 rad` 横向/航向限制，并连续两次确认实速
不高于 `0.08 m/s`；大于 `0.50 m` 的单次定位跳变会使完成历史失效。因此制动导致略微
冲线时先停车再完成，下一转场从实际停车位开始，不会倒回精确终点或绕圈压点。

隔离仿真保留在 [coverage_gz_sim_tree](coverage_gz_sim_tree/README.md)，包含预计算基线、
1 Hz 对照、历史滚动方案、最终分层方案，以及正/负入口偏差和冲线实验。当前 TEB 配置使用
1.35 m 半径、真值定位且无动态障碍；放宽最终入口后 A 区连续 3 轮的 9 次线间转场全部
完成，时长 12.996--14.506 s，且 0.220 m / 10.31° 入场扰动没有触发绕圈恢复。
较早 A、C、自定义 D 区共 39 次、最大 8.492 s 的数据使用已删除的自定义跟踪器，仅作
历史对照。仿真数值不能替代实车跟踪验收。生产实现和验证边界详见
[全覆盖导航当前架构](docs/COVERAGE_NAVIGATION_ARCHITECTURE_20260903.md)与
[转场重复实验](docs/COVERAGE_TRANSITION_EXPERIMENT_20260903.md)。
#### 障碍、暂停和失败恢复

分段 action 返回 `ABORTED/REJECTED/LOST`、执行超时或暂时没有可行路线时，都会按阻塞
处理；动态障碍是最常见原因。任务等待 `10 s`，最多重试 `3` 次。被阻断的是扫掠线时可
先记录为未完成，执行其余可达扫掠后最终再试 `1` 次，并以“部分完成”和阻塞分段数报告；
被阻断的是第一条线入场或线间转场时绝不跳过其依赖的下一条清扫线，而是进入人工暂停，
待路线恢复后由操作员恢复或取消。FOD 进入视觉/安全暂停时，
覆盖管理器保留当前精确分段并在恢复后自行重发，暂停桥不会把覆盖端点当普通目标重发。
定位丢失会暂停且要求人工恢复。覆盖活动期间 `/move_base_simple/goal` 先经过 J6M 目标
入口仲裁器；Qt 相对目标和 RViz `SetGoal` 不得抢占覆盖分段，清扫页选择普通目标工具也会
自动退回地图浏览工具。

操作员点击“暂停覆盖清扫”时，管理器取消当前 move_base 目标但保留当前分段；排除问题并
确认定位、底盘和障碍融合重新就绪后，点击“恢复覆盖清扫”才会继续。感知缺流、底盘故障
或定位丢失同样要求人工恢复；外部 FOD 导航安全暂停则由仲裁器控制，安全暂停解除后管理器
会重发自己保留的分段。任何情况下都不会由 Qt 把旧端点当成普通导航目标重发。

覆盖启动和恢复还要求 `/scan` 的消息时间、`base_link` 帧和几何有效，并要求
`/avoidance/dual_lidar_active=true`，即 MID360 与前、后 LD19 都在参与实时融合。
单个异常 `/scan` 在最近有效样本仍处于 `0.5 s` 新鲜窗口内时不会永久锁停；异常持续到
窗口耗尽、扫描缺流或 LD19 融合失效时才会取消当前分段并进入人工暂停。暂停原因会保留
在 `/coverage/status.detail` 和日志中；持续缺流不会重复发送取消，感知恢复后也不会自行
继续。普通点到点导航仍可按现有策略在 LD19 缺失时退化为 MID360-only，
这一更严格条件只施加在覆盖任务层。

覆盖启动和恢复还要求 M2 的 `/odom` 反馈里程计新鲜，并要求
`/m2_driver/chassis_info` 在实际轮询周期内更新，且硬急停、软件急停、手柄/遥控器急停和
机器人急停全部解除。VCU 的 `ControllerMonitor` 在当前固件上是事件/故障帧，活动
故障会高频重复，健康时可能不发；管理器因此把最新 TCU、左 ECU、右 ECU 的 emergency、
通信超时、电流超限和制动故障锁存 `3 s`，而不把健康时的静默误判为离线。任务执行中
出现任一急停或控制器故障会取消当前 move_base 目标并进入人工暂停，状态恢复后也不会
自动继续旧目标。`/odom` 的 10 Hz 反馈门负责快速识别 VCU/CAN 数据中断。周期性状态查询
只允许 M2 驱动持有；它把平均尝试率限制为 `3.0 Hz`，以 ±20% 周期抖动避免与 VCU 广播
形成固定相位；硬急停、软急停、手柄急停和整车运行状态各自最多重试 4 次，再发送一项
轮换电池遥测。视觉控制器不再调用 `/canbus_server`，只监听 `/canbus_msg`，从而进入视觉
模式不会叠加查询流。组合状态继续使用 `3 s` 新鲜门，视觉原始四项使用 `8 s` 新鲜门；
该窗口覆盖一次全字段均漏回的有界公平轮询，任何明确不安全回包仍会立即中止。Qt“底盘
执行门”显示具体原因，并在未就绪时禁用开始按钮。

#### Qt 显示与只读诊断

每次任务开始时，覆盖管理器会把 Qt 选择的前进速度写入 TEB，并核对该值不超过 NVIDIA
看门狗和 VCU 实时上限；精确扫掠时还应用上面的直线跟踪配置，任务结束后恢复原 TEB
配置。清扫地图同时显示当前车辆模型和
最近 `120` 个有效 `/Odometry` 位姿采样，便于观察最近一段行驶轨迹；白色清扫侧栏还显示
当前 `map -> base_link` 位姿（未完成全局定位时明确标注并回退到里程计坐标系），以及最近
`10 s` 的里程计样本数、累计距离和消息年龄。Qt 明确区分“覆盖
导航状态”和“清扫机构”；面积进度按实际覆盖航段的栅格扫掠并集估算，重复、重试不再
重复累计，但它仍不是刷盘或清洁效果传感器的实测结果。白色/浅色侧栏、输入框和弹窗均
使用黑色或深色文字；深色地图和有色安全告警仍使用各自的高对比前景色。

清扫地图的固定颜色含义为：青色是覆盖条带预览，橙色是当前直接 Hybrid A* 完整连接，
蓝色是 move_base 当前接收的全局参考路线，红色是 TEB 当前局部优化轨迹，绿色是覆盖扫掠
段的实际执行记录。橙线可直接看出连接是否包含倒车、换向以及本次是否已到最终入口；
入场和换道转场的橙/蓝/红路线不累计到绿色覆盖记录，任务终止后都会被清空。

需要排查而不发送运动命令时，可在 NVIDIA 终端查看：

```bash
source /home/slam/robot_j6m_ws/scripts/load_config.sh
source /home/slam/robot_j6m_ws/scripts/setup_env.sh
rostopic echo -n 1 /coverage/status
rostopic echo -n 1 /coverage/active
rostopic info /coverage/planned_path
rostopic echo -n 1 /coverage/hybrid_transition_path
rostopic info /coverage/executed_path
```

`/coverage/status` 中单区域看 `state/plan_id/current_segment/total_segments/detail`；队列另看
`batch_id/batch_active/batch_current_index/batch_total_regions`、完成/部分完成/跳过计数和
`current_region_name`。正常操作仍以 Qt 为唯一入口，不建议手工调用 `/coverage/plan`、
`/coverage/start`、`/coverage/start_batch`、`/coverage/set_paused`、
`/coverage/skip_current` 或 `/coverage/cancel`。

`/coverage/cancel` 和 `/coverage/skip_current` 是请求型 Trigger；服务返回成功只代表后端已
记录请求，不代表目标已经停止或状态已经提交。整批取消或终结应等待
`/coverage/status` 进入批次终态且 `/coverage/active=false`；跳过当前区后批次仍保持
active，应等待 `last_region_id/last_region_state=SKIPPED` 或当前区域索引前进，不能等待
`/coverage/active=false`。正常操作使用 Qt 跟踪这些变化。

#### 当前 V1 约束和判读注意

- 当前 GridMap 的世界坐标换算按轴对齐二维占据栅格实现。使用的 `/map` 必须是 `map`
  frame、平面地图且 origin yaw 为 `0`；当前 `latest` 地图满足这一条件。不要直接换成带
  旋转 origin 或非平面 origin 的外部 OccupancyGrid 后执行覆盖。
- 人工“恢复覆盖清扫”会重新检查定位、障碍融合和底盘状态，但当前不会完整重跑最初的
  watchdog、FOD 放行、任务速度上限和全部启动代际事务。短暂停顿且配置未变化时才使用
  恢复；长时间中断、节点重启或配置/授权变化后应取消旧任务并重新开始。
- 批次两个区域之间的即时规划阶段若恰逢定位、感知或底盘状态失鲜，当前策略是 fail-closed
  结束整批为 `FAILED`，不是无限等待后自动恢复。
- `COMPLETED_PARTIAL` 只表示执行器完成了所有仍可执行的段并保留阻塞段记录，当前没有设置
  “最低覆盖率”门槛，不能仅凭“部分完成”就认定清扫合格。若需要按覆盖率验收，必须在
  任务终态清理前由外部程序持续记录 `coverage_ratio`、阻塞段数和不可覆盖面积。
- 终态清理会清空计划和实时覆盖栅格；当前状态消息不保存整批累计面积，也不作为历史报表。
  未提前留档时，终态只能依据保留下来的阻塞计数和区域结果等有限信息，无法还原最终
  覆盖率；需要完整报告时应另接任务日志系统。

V1 只完成导航覆盖，不控制主刷、边刷、风机或喷淋。点击开始前仍必须满足现场运动
安全条件；后端还会复核全局定位、NVIDIA 主运动门、FOD 目标放行、里程计新鲜度、
车辆静止、VCU 急停/控制器状态、完整障碍融合、静态/未知区碰撞策略和 move_base action
server，任何一项失败都不会发车。

如果当前正在静态地图模式运行，要返回无图增量模式：

```bash
./scripts/start_dual_host.sh --restart
```

### 启停和状态管理

一键启动器会先按硬件身份找回可能改名的 USB 网卡，恢复 NetworkManager 托管、修正并
激活两个静态地址 profile，等待载波和双向网络检查通过；这些操作发生在首次远程停机
之前，避免断网导致冷启动提前退出。随后才回收能由运行令牌或工作区来源严格确认的旧
进程，完成 J6M 时间同步、双端冷启动和运行态健康检查。
启动时暂时缺少传感器消息会显示降级 `WARN`，不会拆掉已经正确建立的 ROS 图；节点
缺失、主机归属错误、参数或 topic 所有权错误仍是启动硬失败。启动成功后栈由用户级
`autolabor-dual-host.service` 托管，终端可以直接关闭。

常用管理入口：

```bash
./scripts/start_dual_host.sh --status   # 查看服务与完整运行态
./scripts/start_dual_host.sh --restart  # 清理后以无图 FAST-LIO 模式冷启动
./scripts/start_dual_host.sh --stop     # 同步停止并验证无残留
./scripts/start_dual_host.sh --foreground  # 前台诊断，不交给用户服务托管
```

`--status` 会读取当前运行实例保存的地图模式和地图集，并在静态地图模式下同时显示
`/fast_lio/localization_status`。脚本不会为了抢占 CAN 串口而杀死无法证明属于本项目
的程序。完整现场交接见 [终端交接文档](/home/slam/robot_j6m_ws/docs/HANDOFF.md)。

只有在分端诊断时才使用下面的手工顺序；它不能替代带 `--map-set` 的一键地图同步
和模式管理。

先在 J6M 的 VS Code 终端或 SSH 终端执行：

```bash
/map/autolabor_runtime/dual_host/bin/start.sh
```

它先启动 J6M `roscore`，然后等待 NVIDIA 的 Livox、看门狗和 CAN，不满足条件时不会启动导航。

再在 NVIDIA 执行：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_nvidia.sh
```

该命令依次建立 NVIDIA 硬件网关，等待 J6M FAST-LIO/move_base 就绪，再启动 ZED、YOLO 和 Qt/RViz。

当前室内版本已移除 GPS/WGS84 目标发送、GNSS 状态页和 RabbitMQ 模块。Qt 使用
`/Odometry`、注册点云和 IMU 判断 FAST-LIO 健康度，人工目标统一使用
`camera_init` 局部坐标或相对车体坐标。

停止 NVIDIA 端但保留 J6M：

```bash
./scripts/start_nvidia.sh --stop
```

停止两端：

```bash
./scripts/stop_dual_host.sh
```

该命令是同步关停：先停 NVIDIA 上的 Qt/视觉，再停 Livox/CAN 网关，自动回收具有本项目严格来源证据的遗留进程，确认 ROS 注册和 CAN 串口都已释放后，最后停 J6M。脚本正常返回即可立即重新启动；若仍有无法证明归属的占用者，会返回非零状态并列出具体 PID，避免误杀主机上的其他程序。

## 静态地图建图与录包

静态地图建图必须先使用无图模式启动完整双机栈，并确保 MID360、IMU、前后两只
固定物理口 LD19 都在线：

```bash
./scripts/start_dual_host.sh --restart
```

随后在 Qt 中执行：

1. 点击“录入静态地图”。后台同时启动 MID360 三维体素累积和双 LD19 二维占据建图。
2. 在符合运动安全条件的情况下覆盖需要建图的区域；建图过程中不要切换静态地图模式。
3. 点击“结束静态地图录入”，等待界面提示“三类静态地图均已保存，latest 已更新”。
4. 使用上面的 `--restart --map-set global_maps/map_sets/latest` 加载新地图。

每次成功建图会生成自包含目录：

```text
global_maps/map_sets/map_<时间>/
├── manifest.yaml
├── map_3d/       # MID360 /cloud_registered 跨帧体素 PCD，供三维重定位
├── map_2d/       # 前后 LD19 跨帧确认后的 PGM/YAML
└── map_fused_2d/ # LD19 占据图与 MID360 持久高度切片的占据并集
```

只有三类地图全部保存并且 `manifest.yaml` 标记为 `complete` 后，
`global_maps/map_sets/latest` 才会原子切换到新地图集。

Qt 的“开始录包”与“录入静态地图”是两个独立功能：“开始录包”只运行 rosbag 记录，
不会自动启动或保存地图。已有 bag 必须包含 `/cloud_registered`、
`/dual_lidar/scan` 和 `/Odometry`，才能离线生成同样的三地图地图集：

```bash
./scripts/build_static_map_from_bag.sh \
  rosbags/example.bag example_map
```

二维原始图严格只使用 `/dual_lidar/scan`；FAST-LIO `/Odometry` 只负责放置扫描。
历史虚拟 360° scan 会再次按前向/后向各 `120°` 截取，并删除车体矩形内的自反射。
某个 LD19 栅格至少需要在 5 个不同的已积分扫描帧中被命中，才会保存为静态障碍。

MID360 数据进入三维 PCD 前会按 FAST-LIO `/Odometry` 限制为车体周围 `20 m`，三维
体素至少需要在 3 个不同点云帧中出现。融合二维图不是直接投影整张 PCD，而是在
FAST-LIO 的 `camera_init`/地图坐标中截取 `Z=-0.4±0.2 m`，即闭区间
`[-0.6,-0.2] m`、总厚度 `0.4 m`。按当前
`base_link -> body ≈ (0.211, 0.02329, 0.95588) m` 和水平建图起点换算，该带约对应
离地 `0.356–0.756 m`，中心约为 `0.556 m`，用于补充 LD19 平面之上的中高度障碍。
该二维栅格还必须在至少 20 个不同 MID360 点云帧中出现才参与并集，因此跟车行人、
飞点和单帧远点不会直接固化为全局地图。若雷达高度、FAST-LIO 内部激光到 IMU 外参
或建图初始姿态发生变化，必须同步重算 `MAPPING_SLICE_CENTER_Z`；建图起点应位于水平
地面并保持车体姿态正常。

为避免上述中高度切片把车体反射或沿建图轨迹拖出的残影固化为静态障碍，生成器只对
二维切片启用两级随姿态裁剪，三维定位使用的 PCD 保持完整不变：每个点云必须与同时间戳
的 FAST-LIO `/Odometry` 精确配对，先逆变换到当时的 `base_link`，删除
`X=[-0.75,+0.75] m、Y=[-0.50,+0.50] m` 内的点；保存前再用导航实际含 padding 的
`1.24×0.90 m` footprint 沿完整轨迹做保守栅格扫掠，删除与扫掠区域相交的 MID360
切片格。轨迹按不超过 `0.05 m / 2°` 插值，`slice_observations.yaml` 会记录裁剪范围、
扫掠格和过滤计数；缺少这些 schema-2 证据的旧切片会被融合器拒绝，不能切换为新地图。

## 关键退化行为

- 前后 LD19 未连接：MID360 仍独立生成 `/scan`，FAST-LIO 与避障不受影响。
- 任一 LD19 数据超过 0.35 秒未更新：`/scan` 自动退回纯 MID360。
- MID360 或 IMU 缺失：J6M 主链不应进入可导航状态。
- 静态地图模式未发送 `/initialpose`、ICP 质量不合格或定位数据超时：状态不会保持
  `LOCALIZED`，导航速度门持续输出零速度。
- 建图时任一固定物理口 LD19 不在线：三地图建图拒绝启动，不会用 MID360 `/scan`
  冒充双 LD19 二维建图数据。
- CAN 未确认且 `REQUIRE_CAN=true`：J6M 一直等待，不会启动运动链。
- J6M 指令超过 0.25 秒未更新、节点所有权异常或 NVIDIA 视觉端退出：NVIDIA 看门狗持续输出零速度。

## 如何判断 FAST-LIO 与全局定位质量

Qt 的“FAST-LIO”页主要判断局部 FAST-LIO 里程计是否连续可靠，而不是只看 RViz
轨迹是否在动：

- `/Odometry` 与注册点云应稳定在约 `10 Hz`，IMU 应约 `200 Hz`；
- 三路数据年龄应分别小于约 `0.30 s / 0.30 s / 0.10 s`；
- `camera_init -> base_link` TF 必须连通；
- 近 2 秒不应出现大于 `0.15 m` 或 `5°` 的单帧跳变；
- 车辆静止满 5 秒后，窗口漂移小于 `0.05 m` 为正常，超过 `0.15 m` 判为异常；
- FAST-LIO 内部协方差只能反映估计器自信程度，不能替代外部真值测量。

界面综合为 `0–100` 分：`≥85` 且无关键故障为“健康”，`65–84` 为“注意”，
其余或任一关键流中断为“异常”。只有“健康”时 Qt 相对目标按钮才会放行。

静态地图模式还必须单独检查全局定位状态：

```bash
rostopic echo -n 1 /fast_lio/localization_status
```

- `WAITING_INITIAL_POSE`：尚未在 RViz 中设置初始位姿；
- `ALIGNING`：已经收到初值，正在进行粗、精两级 ICP；
- `LOCALIZED`：重叠率、RMSE 和内点数满足阈值，速度门可以放行；
- `DEGRADED` 或 `LOST`：匹配失败、成功结果过期或传感器数据过期，速度立即受阻。

状态消息同时包含 `overlap`、`rmse` 和 `inliers`。调试时可查看
`/fast_lio_localization/aligned_scan` 与 `/fast_lio_localization/prior_map` 是否重合，
以及 `/localization` 是否持续输出 `map -> body` 位姿。Qt 的 FAST-LIO 健康分正常
并不等价于全局地图定位已经达到 `LOCALIZED`。

## 安全启用运动

当前正常交付配置为：

```text
MOTION_ENABLED=true
FOD_MOTION_ENABLED=false
```

`MOTION_ENABLED=true` 只表示主控制链功能已启用，不等于允许任意实车测试。
`runtime/motion_authorized.ok` 是独立现场授权门；首次授权或标记缺失时，只有车辆架空或位于
封闭净空区、人员远离车轮、实体急停可用且 CAN 端口已逐个确认后，才执行：

```bash
./scripts/authorize_motion.sh --confirm-elevated-estop
```

未经当前任务明确授权，不发送导航目标或非零速度。获准做首轮运动验证时先限制在
`0.3 m/s` 以内，并保留全部定位、感知、急停、FOD 仲裁和跨机看门狗；不得为验证清扫逻辑
绕过这些 fail-closed 安全门。只有操作员明确要求撤销运动授权时才执行：

```bash
./scripts/authorize_motion.sh --revoke
```

## 构建、部署和检查

均在 NVIDIA 执行。包含 `fast_lio_localization` 的代码更新需要先停止双机栈，再重新
构建并部署到 J6M：

```bash
./scripts/start_dual_host.sh --stop
./scripts/build_workspace.sh
./scripts/health_check.sh --static
./scripts/deploy_j6m.sh
./scripts/sync_j6m_time.sh
```

部署完成后，可按需要使用无图或静态地图的一键启动命令。网络切换后可运行
`./scripts/health_check.sh --network`，实际主链启动后运行
`./scripts/health_check.sh --runtime` 做严格数据流验收；它发现缺流时返回非零但不会
停止服务。J6M 自身检查命令是：

```bash
/map/autolabor_runtime/dual_host/bin/health_check.sh
```

详细 topic 链和回滚说明见 [架构说明](/home/slam/robot_j6m_ws/docs/ARCHITECTURE.md)；当前未完成事项见 [工作交接](/home/slam/robot_j6m_ws/docs/HANDOFF.md)。
