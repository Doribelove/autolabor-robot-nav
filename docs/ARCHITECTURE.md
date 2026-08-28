# 双机架构说明

## 物理网络

```text
MID360 192.168.1.112
        │ RJ45
NVIDIA WCH USB Ethernet 192.168.1.50

J6M eth0 192.168.10.100
        │ 交换机 / USB 扩展坞 RJ45
NVIDIA ASIX USB Ethernet 192.168.10.50

NVIDIA wlan0 ── Internet/default route
```

两个有线接口必须位于不同 `/24`。J6M 不直接接 MID360；NVIDIA 作为物理协议和 ROS 网络网关。
接口名不是身份依据：NVIDIA 启动器分别用永久 MAC `6C:1F:F7:C4:96:B8`
（ASIX USB 网卡，经扩展坞和交换机连接 J6M）和 `50:54:7B:E3:C9:10`（MID360）
解析当前接口名并绑定 NetworkManager 配置。若永久 MAC 未命中，只接受配置中 USB
VID:PID + serial 的唯一精确匹配；网络自愈必须在首次 J6M 远程停机之前完成。

## 生命周期与故障恢复

默认入口 `scripts/start_dual_host.sh` 通过用户级 transient systemd 服务
`autolabor-dual-host.service` 托管整棵 NVIDIA/J6M 启动进程树，使用
`KillMode=control-group` 做同步关停。服务与启动终端、VTE scope 和 GDM 会话解耦，
因此桌面重启不会留下失去监督器的 ROS 子进程。

每次冷启动先按两类可验证来源回收旧进程：本次运行令牌，或工作区、ROS Master、
节点名/入口命令全部匹配的兼容来源。UID 不同、命令不在白名单或仅仅占用串口的
进程不会被信号终止；这种不确定情况会列出 PID 并安全失败。

## 数据链

```text
MID360
  └─ NVIDIA livox_ros_driver2
       ├─ /gateway/livox/lidar ── J6M relay ── /livox/lidar ── FAST-LIO
       └─ /gateway/livox/imu   ── J6M relay ── /livox/imu   ── FAST-LIO

前/后 LD19 ── NVIDIA 双雷达融合 ── /dual_lidar/scan ─┐
MID360 ── J6M 水平切片 ── /mid360/scan ──────────────┴─ /scan ── move_base

FAST-LIO /cloud_registered_body + 可选 LD19
  └─ /cloud_registered_body_enhanced（显示/调试，不反馈给 FAST-LIO）

ZED + YOLO（NVIDIA）── /fod/detections ── J6M FOD 仲裁

麦克风（NVIDIA）── arecord ── 隔离 Whisper large-v3 worker
  └─ 本地识别文本 ── sweeper_ai 授权门 ── 云端计划 / MCP 工具
```

FAST-LIO 始终只使用 MID360 原始点和 IMU，前后二维雷达不会污染定位。move_base 的避障输入是独立的 `/scan`：MID360 是强制主源，LD19 是可超时移除的增强源。

静态建图是独立数据链：`/cloud_registered` 体素累积为三维 PCD；
`/dual_lidar/scan + /Odometry` 生成纯双 LD19 二维占据图；停止后再把三维图在配置的
中高度障碍带投影，并以占据并集合成第三张图。普通“录包”不会触发该流程。
MID360 高度切片与同时间戳 `/Odometry` 精确配对，逐帧在 `base_link` 删除车体自点；
保存切片观测前，再减去含 padding 的导航 footprint 沿建图轨迹扫过的全部相交栅格。
该裁剪只影响融合二维图，完整三维 PCD 仍用于已知地图 ICP 定位。

不传 `--map-set` 时启动器保持无图 FAST-LIO 模式。已知地图定位属于实验性显式
可选项；传入地图集后，FAST-LIO 仍按
原始算法输出高频 `camera_init -> body` 里程计；独立的 `fast_lio_localization` 节点
加载 PCD，在 `/initialpose` 附近执行低频粗到精 scan-to-map ICP，估计
`map -> camera_init`。map_server 只为 move_base 加载二维图，不启动 AMCL；ICP
质量不合格、数据过期或定位丢失时，速度门控输出零速度并取消旧导航目标。

原始 Livox 数据跨机只建立一组 relay 订阅，避免 FAST-LIO、点云转换各自重复传输大消息。默认不跨机发送 ZED RGB、深度图或完整视觉点云。

Qt/RViz 默认显示 `/cloud_registered_body_enhanced`，并在静态定位模式显示低频动态
`/fast_lio_localization/aligned_scan`，用于区分实时匹配点云与锁存的三维先验。完整点云
显示会占用跨机带宽；需要节省带宽时可在 Displays 中取消勾选。融合后的 `/scan` 也默认
显示并持续用于避障。

静态地图模式下，初始位姿前 TopDownOrtho 以 `map` 为视角目标显示全图；定位达到
`LOCALIZED` 后可自动或手动把视角目标改为 `base_link`，Fixed Frame 始终保持 `map`。
综合页可由“④ 显示静态三维先验”按需订阅 J6M 上锁存发布的
`/fast_lio_localization/prior_map`。该显示默认关闭，开启时使用 `map` 固定坐标系和
Orbit 视角；关闭或进入初始位姿工具时恢复二维 TopDownOrtho 全图。清扫页仍保持纯二维。

综合页和清扫页不再依赖全局 `robot_description` 生成车体；两者直接用
`rviz/Polygon` 显示 `/move_base/local_costmap/footprint`。这个 Polygon 是
costmap 根据 `base_link` 和当前 footprint/padding 持续发布的实际导航安全边界，当前约为
`1.24 × 0.90 m`。静态地图在初始位姿或新鲜里程计/TF 缺失时没有可用的
`map -> base_link`，因此车框暂不显示；定位链建立后，它在二维和三维视角中都会随
车辆移动。综合页侧栏只在静态地图模式且全局定位状态新鲜时读取
`map -> base_link`，显示 map 系 `X/Y/Yaw`；条件不满足时显示具体等待原因而不回退成
未标注的局部坐标。共享 RViz 默认显示透明度 `0.55` 的全局 costmap，并在其上叠加
透明度 `0.90` 的滚动局部 costmap；正常界面提供全局层显隐按钮和 topic 新鲜度状态。

## 控制链

```text
J6M move_base
  /cmd_vel_navigation
        ↓
J6M /fod_navigation_mode
  /cmd_vel_safe
        ↓  ROS TCP/IP
NVIDIA /nvidia_cmd_vel_watchdog（250 ms lease、1.7 m/s 绝对硬上限）
  /cmd_vel
        ↓
NVIDIA /m2_driver → USB-CAN → M2 底盘
```

静态地图覆盖清扫在现有控制链上增加任务层，不旁路安全仲裁：

```text
Qt 清扫页（NVIDIA）
  /coverage/clicked_point → 多边形 /coverage/plan
  区域库（按 map-set 目录隔离，/map SHA-256 复核）→ /coverage/start_batch
        ↓
J6M /coverage_manager
  冻结批次 → 逐区域即时规划 → 静态图裁剪 + 连通域 + 弓字扫描线 + 秒级时间优化
  start: PREPARING（无锁重规划）→ 最终安全复核 → 原子激活
        ├─ 同步 set_enforced_path(转场) ── CoverageGlobalPlanner → Navfn
        └─ 同步 set_enforced_path(扫掠) ── CoverageGlobalPlanner → /coverage/enforced_path
                                ↓
                         move_base + TEB
                                ↓
             原有 FOD 仲裁、跨机看门狗与 M2 控制链
```

普通点到点入口使用独立的目标门：Qt/RViz 发布 `/move_base_simple/goal`，J6M
`/navigation_pause` 仅在未暂停且没有活动覆盖任务时转发到
`/navigation_goal/accepted`；move_base 在完整双机启动中只订阅后者。覆盖管理器仍通过
actionlib 直接拥有自己的分段目标，因而普通目标不能再抢占覆盖任务。

AI 普通导航不走匿名的 simple-goal 路径。NVIDIA 后端先生成唯一显式 GoalID，发布
`MoveBaseActionGoal` 到 `/navigation_goal/action_request`；J6M `/navigation_pause` 在与覆盖
所有权相同的互斥锁内校验授权租约、时间、目标和占用状态，统一改写为 J6M 时间后才转发到
`/move_base/goal`。后端只接受同 ID 的 action 回显和唯一状态。精确取消通过独立的
`/navigation_goal/cancel_request` 在 J6M 本机转发；若撤销先赢得桥内同一把锁，桥只对从未
转发的 ID 发布 `/navigation_goal/cancel_ack`。AI 心跳失联、状态为 `LOST` 或闭环确认
失败时均保留 ID 并重复撤销，不能以“界面已失败”释放目标。

覆盖管理器在发送任一 action 目标前，还使用带代际 token 的同步
`/navigation_pause/set_coverage_owner` 占有 move_base。该服务与 AI 请求共享同一把锁：覆盖
先取得所有权时 AI 立即拒绝；AI 先通过时，覆盖 claim 会在返回前精确取消现有 AI ID。
`/coverage/active` 继续作为锁存状态广播和重启兜底，但不再承担跨 topic 的唯一互斥证明。

`CoverageGlobalPlanner` 是 move_base 的全局规划插件：没有活动覆盖分段时委托 Navfn，
活动清扫分段时只接受管理器发布且端点匹配的新鲜路径。路径过期且任务仍活动时保持
fail-closed，不允许突然改走最短路。区域本身不是 geofence；转场由 Navfn 在完整已知
自由地图中规划。覆盖管理器只在静态地图模式启动。覆盖默认有效宽度 `1.00 m`、重叠率
`15%`、车道中心距 `0.85 m`。普通导航和覆盖任务默认最高前进速度为 `0.80 m/s`；清扫页
按任务限制在 `0.10–1.60 m/s`，管理器动态写入 TEB，并在开始前同时核对 NVIDIA 看门狗
和 VCU 实时上限。Qt 还下发倒车速度、最大角速度、线/角加速度、换向附加时间和分段交接
附加时间；这些值既参与秒级路线估算，相关运动上限也会临时写入 TEB。VCU/TEB/覆盖模型
统一按 `0.65 m` 轴距与 `1.35 m` 最小转弯半径核对，强制覆盖路径还检查终点 yaw 和完整
车辆 footprint。倒车只允许用于转场；Navfn 转场尚不是带档位的 Reeds-Shepp 规划。精确
扫掠段把 TEB 参考点间距动态设为
`0.30 m`，位置/横向/航向代价权重设为 `50/200/100`，前进运动学权重设为 `1000`，并让
全部同伦候选计入参考线代价；转场与普通导航维持基线参数，任务终态恢复。TEB 对零反向
速度上限提供原生约束：零速可行、负速受罚且输出硬截断，不再依赖小于惩罚缓冲区的无效
参数组合。

已知清扫区的唯一可写副本位于 NVIDIA 的
`global_maps/map_sets/<map-set>/coverage_regions/<source_mode>/regions.json`。物理 map-set
目录是持久化隔离边界；`latest` 等符号链接先规范化到实际目录，因此两个不同 map-set 即使
具有相同栅格也不会串用。地图摘要由 J6M 针对实际 `OccupancyGrid` 的 frame、宽高、
分辨率、完整 origin pose 和栅格数据计算并随 `CoverageStatus` 发布，Qt 在目录隔离之外
继续复核摘要和来源模式。目录内只保存区域定义，使用 UUID、锁和原子替换；规划 ID、轨迹、
任务进度与运动指令都不持久化。旧集中目录只在其 `map_source` 精确匹配当前规范化 map-set
时作为只读迁移来源，不能仅凭摘要跨目录迁移。

`/coverage/start_batch` 一次接收用户排序后的不可变区域快照和整批规划参数。管理器始终
根据每一区域开始时的最新全局位姿即时生成条带顺序，所以跨区移动仍是普通 Navfn + TEB
点到点转场，不属于覆盖实走轨迹。批次期间 `/coverage/active` 持续锁存为 true，单区
`active` 只表示当前已有 move_base 分段；这一区分既让跟踪/安全回调保持准确，又消除换区
间隙被普通目标抢占的窗口。完成和部分完成会推进队列，失败终止；`/coverage/skip_current`
只跳过当前区；Qt/AI 批次取消优先使用带精确 batch ID 的 `/coverage/cancel_batch`。
`StartCoverageBatch.client_request_id` 必须为 `coverage-batch-<32hex>` 并原样成为 batch ID：
同 ID/同 payload 幂等回放，同 ID/不同 payload 拒绝；cancel-before-start tombstone 能阻止
已取消请求的迟到提交，foreign ID 不改变当前任务。批次仅驻留 J6M 进程内，重启后不自动
恢复。若启动失败后仍保留了导航 owner，所有清理按一次完整事务串行执行：先确认清理开始
时捕获的 move_base generation 和精确 `ClientGoalHandle` 已进入可信终态，再关闭覆盖规划器、
恢复 TEB，最后释放同一 owner token。重复 exact/broad cancel 只返回“清理中”，不会再次执行
外部副作用；清理期间新 batch ID 不会被写成永久拒绝记录，可在事务结束后用同 ID 重试。
状态定时器和普通取消也携带捕获的 generation/handle，因此旧 A 回调晚到时既不能取消 B，
也不能把 B 改写为 `FINALIZING`。

首个条带入口和条带间连接均是普通点到点转场：`EnforcedPath.active=false`，规划插件明确
调用 Navfn，而不是把预览条带间的直线当作可执行路径。静态模式的全局代价地图也订阅
实时 `/scan`，move_base 以 `1 Hz` 重做全局路线，TEB 以 `10 Hz` 做局部运动学避障；有可行
通路时允许离开所选多边形绕障。`EnforcedPath.active=true` 只用于扫掠条带，避免障碍或
消息竞态让扫掠线被最短路替代。无可行通路时仍保持停车、等待和有限次重试。

规划与启动是两个不同事务。`/coverage/plan` 使用不可变 `GridMap` 快照生成预览，并在提交
前复核地图摘要；`/coverage/start` 再以当前 `map -> base_link` 为连通域种子重规划。VCU
service、move_base 等待和几何重规划全部在覆盖状态锁之外执行，使 `/scan`、双 LD19、定位
与里程计回调不被耗时计算阻塞；激活前在锁内再次核对计划 ID、地图摘要、车辆静止和全部
新鲜度门。规划和启动分别携带独立代际令牌；`/coverage/cancel` 可撤销 `PLANNING`、
`READY`、`PREPARING` 或活动任务，旧线程的迟到提交不能覆盖下一轮计划。终态清理在同一
生命周期锁内失效计划并发布空 Path、空 Polygon 和 Marker `DELETEALL`，确保锁存预览和
实走轨迹不会残留或误删随后生成的新预览。每个分段先同步调用
`/move_base/CoverageGlobalPlanner/set_enforced_path`，插件在同一个事务中确认覆盖任务所有权
和 Navfn/精确路径模式后才发送 action goal；`/coverage/enforced_path` topic 只能对服务
已确认的同一计划、分段和模式做后续新鲜度刷新。规划器先对全部扫掠角度快速估时，再对
最好的三个角度运行宽度 `128` 的确定性 beam search，联合选择第一条线、任意后续条带顺序
及每条线方向。覆盖完整度为硬优先级；其后的目标函数严格使用秒：静态已知自由格 A* 距离、
三角形/梯形线加减速、受最小半径和最大角速度约束的转向时间、倒车换向及 move_base 分段
交接附加时间。连接模型会强惩罚“已在端点但要求原地改航向”的 Ackermann 不可行入口；
执行器仍会拒绝残余的纯朝向转场，而不会让 TEB 尝试原地旋转。

TEB 原始优化 cost 是障碍、可行性和参考线等不同量纲残差的加权和，不作为秒数直接进入
排序。规划只复用与执行一致的 TEB 运动上限，真实转场仍由 Navfn + 在线 TEB 决定；动态
障碍和局部同伦会改变实际耗时。因此 beam search 结果是受限搜索下的最短预计时间，不是
任意排列、动态代价地图和 TEB 轨迹的数学全局最优证明。

NVIDIA Qt 的综合页和清扫页共用唯一 librviz 画布。实际加载的
`operator_navigation.rviz` 用 `rviz/Odometry` 保留最近 `120` 个 `/Odometry` 位姿；清扫
侧栏从同一数据维护最近 `10 s` 窗口，显示样本数、累计里程和年龄，并在全局定位有效时
额外显示 `map -> base_link` 数值位姿。清扫规划控件的每次修改都立即写入 NVIDIA 当前用户
的 Qt `QSettings`（组 `coverage/planning_parameters`），下次启动作为默认值；计划进入
`READY` 后整组控件锁定，取消并重规划后才可修改。该偏好是操作台全局用户配置，不属于
按 map-set 隔离的已知区域 JSON，也不会静默改写 J6M 的 `coverage.yaml`。

静态模式的 rolling local costmap 由 StaticLayer（`/map`）、ObstacleLayer（融合
`/scan`）和 InflationLayer 叠加，避免 TEB 只看实时雷达而穿过二维已知墙体。静态模式
还单独启用 `treat_unknown_as_obstacle=true` 和 Navfn `allow_unknown=false`；无图模式的
TEB 默认值仍为 false。该开关只拒绝 CostmapModel 的 `-2` 未知区；`-3` 未来足迹超出
滚动局部窗口不是碰撞，但当前姿态超窗仍拒绝。静态 TEB 视野为 `8.0 m`，在 `20 m`
局部窗口内保留边界裕量。Qt 中蓝色为 TEB 接收的全局参考路线，红色为当前局部优化轨迹；
两者只在 move_base 目标活动期间启用，终态会清掉 RViz 缓存。

覆盖任务比普通导航多一层障碍链 fail-closed：`/scan` 必须具有新鲜时间戳、正确
`base_link` 帧和有效几何，同时 `/avoidance/dual_lidar_active` 必须新鲜且为 true。
失去任一条件只在边沿取消一次当前目标，转入人工暂停；恢复数据不会自动恢复运动。
单个异常 `/scan` 只会替换最新诊断；最近有效样本仍在 `0.5 s` 窗口内时保持就绪，持续
异常或缺流超过窗口后才锁存暂停。锁存原因在暂停等待期间保持可见。

覆盖任务还从 NVIDIA M2 驱动订阅 `/m2_driver/chassis_info` 与
`/m2_driver/chassis_monitor`，并以 M2 `/odom` 作为 10 Hz 物理反馈心跳。反馈里程计超过
`1 s` 不更新会快速拒绝启动/恢复；M2 驱动按实测 VCU 吞吐以 `4 Hz` 发送单项请求，每秒
优先完成硬急停、软急停和手柄急停查询，再轮询一项电池遥测。它避免突发请求丢回复，
安全字段约 `1 Hz`、五项电池字段各约 `0.2 Hz`。组合状态使用 `3 s` 门限，四类急停任一
置位会立即拒绝。控制器监控是当前
VCU 固件的事件/故障帧，活动故障会高频重复而健康时可能静默，
因此只在 `3 s` 窗口内锁存 TCU、左右 ECU 的 emergency、通信超时、电流超限和制动位。
执行中触发时仅在故障边沿取消一次当前 action goal 并转入人工暂停；故障消失不会自动
恢复旧目标。`CoverageStatus.chassis_ready/detail` 同时驱动 Qt“底盘执行门”和开始按钮。

`/fod_navigation_mode` 是 `/cmd_vel_safe` 的唯一发布者；NVIDIA 看门狗是 `/cmd_vel` 的唯一发布者。看门狗拒绝 NaN/Inf、非平面指令、超限指令、重复发布者、错误订阅者和过期命令。

AI 语音链也不旁路上述控制链。Qt 和 `sweeper_ai` 每次启动都把语音输入、AI 语义解析、
AI 控制三门以及智能语音会话置为关闭。确认语音授权后可使用保留的“开始录音/停止并识别”
手动模式，或另行确认“启用智能语音”。智能模式保持一个连续 `arecord` 流，本地能量 VAD
使用前滚、连续有声帧、约 `0.8 s` 静音和单句最长时限自动切句；采集线程与 `large-v3`
推理线程分离，句子通过有界队列依次识别。因此它是自动断句后的逐句批量识别，不提供
partial-token 流式转写。

语音门只允许本地音频采集与 Whisper 推理；解析门决定识别文本能否发送云端；控制门决定
校验后的计划能否调用改变机器人状态的 MCP 工具。解析门关闭时识别结果仅通过 `AiEvent`
在 Qt 本地显示，解析门开启时智能语音按有界 FIFO 严格串行提交；控制授权变化会清除尚未
提交的旧句，防止旧口令事后升级为可执行命令。控制门关闭时云端计划只能预览。Qt 心跳超过
3 秒、语音撤权、智能模式关闭或 worker 异常都会关闭精确的采集进程、清空队列；session ID
和代际令牌共同丢弃旧会话的迟到结果。

ASR worker 是 `sweeper_ai` 的本地子进程，以 JSON-lines 协议通信，子进程环境移除 ROS、
云端密钥、UI 会话令牌和 MCP 控制令牌。它只接受固定 `large-v3`、CUDA、本地 checkpoint
和显式 ALSA 设备；checkpoint 位于 NVIDIA 的 `runtime/asr/models/large-v3.pt`，运行时
禁止下载。缺少 CUDA 环境、checkpoint、哈希匹配或真实麦克风任一条件时，ASR 状态为
不可用且不能打开录音。该链全部位于 NVIDIA，不产生 J6M 构建或发布产物。

AI 的 `navigate_map_pose` 与 Qt 地图设点使用同一个 move_base/TEB 执行器和安全链，但 AI
使用可端到端追踪的显式 action GoalID，而 Qt 继续使用经过目标门的 simple-goal 入口。AI
在发布前增加本地安全预检。锁存 `/map` 首次收到后按静态快照持有，不使用接收时间判断过期；其
frame、尺寸、分辨率、完整 origin pose 和全部栅格生成与覆盖管理器相同的 SHA-256，并与
新鲜 `CoverageStatus.map_digest` 比对。目标坐标先按 OccupancyGrid origin yaw 逆旋转到栅格
局部坐标，再检查范围及未知/占用单元；发布边界再次复核定位、覆盖状态和地图代际。只有
定位、模式、里程计等动态消息继续使用秒级新鲜度。

## 运行位置

| 功能 | NVIDIA | J6M |
|---|---:|---:|
| ROS master |  | 是 |
| Livox 物理驱动 | 是 |  |
| FAST-LIO |  | 是 |
| 前后 LD19 驱动/初次融合 | 是 |  |
| MID360 + LD19 避障融合 |  | 是 |
| move_base + TEB |  | 是 |
| 覆盖规划、任务状态机、全局规划插件 |  | 是 |
| USB-CAN/M2 | 是 |  |
| ZED、CUDA、YOLO | 是 |  |
| Qt AI、Whisper large-v3 ASR、MCP 客户端 | 是 |  |
| FOD 速度仲裁 |  | 是 |
| 最终速度看门狗 | 是 |  |
| Qt/RViz | 是 |  |

## J6M 文件布局

```text
/map/autolabor_runtime/
├── rootfs/                         Ubuntu 20.04 ARM64 chroot
│   └── opt/autolabor/dual_host/
│       ├── releases/<时间戳>/install
│       └── current -> 当前版本
├── dual_host/bin/                  启停、健康检查、回滚
├── bin/                            chroot 挂载/卸载
├── dual_host/config/dual_host.env
├── config/                         持久配置映射
├── maps/                           持久地图映射
├── fast_lio/                       FAST-LIO 持久运行数据
└── logs/dual_host/                 日志
```

chroot 使用 J6M 自身内核，并 bind mount `/dev`、`/proc`、`/sys`、`/run`、`/etc/hosts`。应用 overlay 使用 catkin install 空间；ROS home、日志和动态数据映射到 `/map`，不写 J6M overlay 根文件系统。

## 回滚

先停止 J6M 主链，再列出或切换版本：

```bash
/map/autolabor_runtime/dual_host/bin/stop.sh
/map/autolabor_runtime/dual_host/bin/rollback.sh --list
/map/autolabor_runtime/dual_host/bin/rollback.sh YYYYMMDD_HHMMSS
```

这只切换 `current` 符号链接，不刷写 ACORE、MCU、boot、system 或整盘镜像。
