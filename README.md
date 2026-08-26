# robot_j6m_ws 双机工作空间

这是从 `/home/slam/robot_ws` 当前工作树提取出的独立 ROS Noetic 工作空间。原工作空间及其 68 项未提交改动没有被修改；新目录不包含旧 Git 历史，也没有复制旧 `build`、`devel`、日志、rosbag 或虚拟环境。

## 当前状态

- NVIDIA 端 22 个包已用 Release 配置编译通过；当前静态健康检查汇总为 494 项通过、
  0 错误、0 失败、0 跳过。
- `fast_lio_localization` 已通过编译、相关测试和现有 bag 重放验证；覆盖清扫新增几何、
  Qt、静态地图启动、暂停桥和部署契约测试均已纳入同一测试汇总。
- J6M Ubuntu 20.04/ROS Noetic chroot 已存在于 `/map/autolabor_runtime/rootfs`。
- J6M 版本 `20260824_170341` 已部署并通过远端静态健康检查。
- 两机 ROS 普通消息和 Livox 自定义消息已在当前临时网段双向验证。
- J6M relay、FAST-LIO、避障融合、增强点云、move_base 与 FOD 仲裁已在实机数据流中完整拉起，并通过连续启停清理验证。
- 三地图建图、跨帧静态过滤和地图同步已完成 bag 验证；当前 `latest` 为
  `map_20260822_slice_zm040_hw020_final`，并已同步到 J6M。已知三维图定位采用 FAST-LIO
  高频里程计加低频多尺度 ICP。覆盖清扫 V1 已完成静态地图冷启动、Qt 目视和低速实车
  分段验收；现场已确认“Navfn 转场→精确扫掠→Navfn 转场”的模式切换。整轮任务后来因
  人群封路、雷达被遮挡和操作员遥控接管而安全暂停，仍需在正常现场条件下重新完整跑一轮。

## 机器分工

NVIDIA：MID360 网口驱动、USB-CAN/M2、前后 LD19、ZED、YOLO、Qt/RViz，以及最终 `/cmd_vel` 看门狗。

J6M：ROS master、Livox topic relay、FAST-LIO、高频里程计上的已知地图 ICP 定位、
MID360/LD19 避障融合、map_server、move_base + TEB、FOD 安全仲裁。

原 `/home/slam/robot_ws` 继续承担机场 GPS 模式；不要用其中的旧一体化脚本启动本双机模式。

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

第二条命令一次性安装 ZED 的 `video` 组权限和开机后定向 coldplug 服务，解决 Jetson
重启过程中 udev 过早退出后，ZED usbfs（以及内核生成时的 hidraw）节点遗留为
`root:root 0600` 的问题。SDK 4 可直接使用 f781 usbfs，因此 hidraw 未生成本身不作为
启动失败条件。
它需要管理员密码；项目的一键启动命令本身不会静默提权。

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
/home/slam/robot_j6m_ws/scripts/start_dual_host.sh --start \
  --map-set /home/slam/robot_j6m_ws/global_maps/map_sets/latest \
  --authorize-fod-motion </dev/null
```

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

地图上路线颜色固定为：青色是覆盖条带预览，蓝色是传给 TEB 的全局参考路线，红色是
TEB 当前优化的局部轨迹，绿色是覆盖任务实际执行记录。蓝/红路线仅在 move_base 目标
处于活动状态时显示，终止或取消后 Qt 会禁用再清空 RViz 缓存，避免旧路线残留。静态
模式的滚动局部代价地图同时加载二维静态层和实时 `/scan` 障碍层；TEB 最终可行性检查
扩展到局部时域最多前 51 个姿态，并将静态障碍和未知区 footprint 判为不可行。预测
足迹仅超出滚动窗口时不再误报碰撞，但当前姿态位于窗口外仍保持 fail-closed；静态模式
同时把 TEB 视野限制为 `8.0 m`，在 `20 × 20 m` 局部窗口内保留边界裕量。

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

覆盖清扫只在带 `--map-set` 的静态地图模式下启用。无图启动时，Qt“清扫”页的
框定、规划参数和开始按钮全部置灰；即使 ROS 图中偶然出现其他 `/map`，界面仍同时
检查一键启动传入的 `static_map_mode`，不会把无图建图模式误判为可清扫模式。

操作流程：

1. 按上一节发送车辆真实 `/initialpose`，等待定位状态为 `LOCALIZED`。
2. 打开 Qt“清扫”页，点击“框定覆盖清扫范围”。左侧专用 RViz 会自动切换到
   `Publish Point`，可在全局地图上连续点选任意数量的顶点。
3. 可使用“撤销一点”逐点修改，或随时“取消框定”。点击“确认区域并生成轨迹”后，
   首尾自动闭合；自交、重复边、面积过小和非有限坐标都会被拒绝。
4. 检查青色覆盖条带、可覆盖面积与不可覆盖估算。界面不再把条带之间的直线连接画成
   可执行路径；转场的实际蓝/红路线由 Navfn + TEB 在执行时生成。默认有效宽度为 `1.00 m`、重叠率
   `15%`，所以默认车道中心距为 `0.85 m`；这两个参数可在规划前修改。最高前进速度
   默认 `0.80 m/s`，覆盖任务可在 `0.10–1.60 m/s` 内选择。NVIDIA 最终看门狗的
   `1.70 m/s` 是绝对软件边界；任务开始时还会读取 VCU 实际速度上限，不能把任一上限
   当作室内推荐巡航速度。此时可点击“取消已生成轨迹”丢弃区域和预览，再重新框定。
5. 点击“开始覆盖清扫”。后端会按车辆当前全局位置重新做已知自由空间连通域裁剪，
   再导航到兼顾距离、当前车头方向和最小转弯半径的合适轨迹端点并执行覆盖。侧栏会先
   显示“正在复核起点与安全门”，通过后才进入“前往起点”。

任务启动采用两阶段事务：管理器先锁定计划 ID、地图摘要和区域快照，然后在不占用状态
锁的情况下执行 VCU/TEB 在线核对及连通域重规划，使 `/scan`、双 LD19、定位和里程计回调
能继续刷新；最后重新核对同一计划、同一地图和全部新鲜度安全门，才原子地声明任务活动。
因此耗时重规划不会再把仍在正常流动的 `/scan` 误判为过期并让任务刚接受就永久暂停。
准备期间若地图或计划变化，启动会明确失败且不会提交任何 move_base 目标。
轨迹生成、启动准备和活动任务都可从清扫页取消。规划与启动各自使用不可复用的请求代际
令牌，取消后的迟到回调不能恢复旧计划；活动任务完成、部分完成、失败或取消后，后端会
失效计划 ID，并把区域、青色条带和绿色实走记录的锁存消息清空。Qt 同步清除本地草稿，
随后可直接开始下一次框定，无需重启。

区域多边形表示“需要覆盖的面积”，不是车辆地理围栏。转场可以离开多边形，但只能经过
静态图中的已知自由空间；障碍物、未知格和与车辆不连通的地图岛会被裁掉并计入不可覆盖
估算。覆盖直线由
`autolabor_coverage/CoverageGlobalPlanner` 强制交给 move_base，普通点到点目标仍回退
Navfn；管理器先同步调用规划插件的 `set_enforced_path` 服务，在同一个事务中确认任务
所有权并切换到“Navfn 转场”或“精确扫掠”模式，之后才提交对应 action goal；后续 topic
只能刷新服务已确认的同一分段，避免跨 transport 到达顺序让旧扫掠路径污染新转场，或让
扫掠短暂走最短路。强制直线路径逐姿态核对完整导航 footprint 和
终点航向。转场可选让 TEB 尝试
`0.30 m/s` 低速倒车，覆盖直线禁止倒车。

精确扫掠段会临时启用独立的 TEB 直线跟踪代价：参考点间距收紧为 `0.30 m`，位置、
横向偏差和航向偏差权重分别为 `50/200/100`，前进运动学权重提高到 `1000`，并让所有
同伦候选都计入参考线代价。TEB 底层原生支持 `max_vel_x_backwards=0`：静止仍是可行解，
负向速度会被优化器惩罚并在输出端硬截为零。上述参数只用于扫掠直线；Navfn 转场和普通
点到点导航继续使用原配置，分段切换及任务结束时恢复基线。直线代价不会越过 footprint
碰撞和障碍安全约束；条带被障碍阻断时仍停车、等待和重试。

“从当前位置去第一条清扫线”和“从上一条清扫线去下一条清扫线”都属于转场，不计入
覆盖轨迹，也不会把两点间的直线强塞给底盘。转场时 Navfn 每秒依据包含实时 `/scan`
障碍的全局代价地图重新规划，TEB 以 `10 Hz` 在局部选择满足前轮转向约束的绕行轨迹；
只要存在足够车辆宽度和转弯半径的通路，车辆应绕开障碍继续前往条带入口。只有障碍把
所有可行通路堵死、定位/传感器失联或真实安全门触发时才允许停下。扫掠段则相反：它要
保证不漏扫，因此全局参考线固定为该条带；遇到无法局部避开的障碍会等待和重试，而不会
悄悄改成一条绕开大片待扫区域的最短路。

室内链按 `0.65 m` 轴距、VCU 最大前轮转角约 `0.488692 rad / 28°` 建模；理论机械最小
半径约为 `1.222 m`。覆盖规划和 TEB 统一使用 `1.35 m`，对应所需转角约 `25.71°`，保留
约 `2.29°` 裕量，并启用成比例速度饱和以保持 `v/ω` 曲率。每次任务开始前会在线读取
`/m2_driver/chassis_parameter`，并核对 VCU、TEB 的轴距、转角、半径和 Twist 输出模式；
不一致时拒绝发车。条带排序先保留由车道间距和最小转弯半径确定的跨行步长，再枚举该
顺序及其反序的所有循环起点；每个候选用动态规划联合选择每条线的行驶方向，并比较
“当前位置到首条入口 + 全部清扫线长度 + 全部换道连接”的完整估算代价。因此首条线不再
由最近端点单独决定，而是选择这一可转弯候选族中的总代价最低路线。连接代价计入距离、
当前/目标航向和转弯半径，但转场仍是 Navfn + TEB，尚未实现 Dubins/Reeds-Shepp 全局
连接曲线，因此软件不会把“路径已生成”表述为整条路线已经通过实体车运动学证明；仍需
低速实车验收。

动态障碍导致分段失败时，任务等待 `10 s`，最多重试 `3` 次后跳过；其余路线完成后
再最终重试 `1` 次，并以“部分完成”和阻塞分段数报告。FOD 进入视觉/安全暂停时，
覆盖管理器保留当前精确分段并在恢复后自行重发，暂停桥不会把覆盖端点当普通目标重发。
定位丢失会暂停且要求人工恢复。覆盖活动期间 `/move_base_simple/goal` 先经过 J6M 目标
入口仲裁器；Qt 相对目标和 RViz `SetGoal` 不得抢占覆盖分段，清扫页选择普通目标工具也会
自动退回地图浏览工具。

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

每次任务开始时，覆盖管理器会把 Qt 选择的前进速度写入 TEB，并核对该值不超过 NVIDIA
看门狗和 VCU 实时上限；精确扫掠时还应用上面的直线跟踪配置，任务结束后恢复原 TEB
配置。清扫地图同时显示当前车辆模型和
最近 `120` 个有效 `/Odometry` 位姿采样，便于观察最近一段行驶轨迹；白色清扫侧栏还显示
当前 `map -> base_link` 位姿（未完成全局定位时明确标注并回退到里程计坐标系），以及最近
`10 s` 的里程计样本数、累计距离和消息年龄。Qt 明确区分“覆盖
导航状态”和“清扫机构”；面积进度按实际覆盖航段的栅格扫掠并集估算，重复、重试不再
重复累计，但它仍不是刷盘或清洁效果传感器的实测结果。

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

默认保持：

```text
MOTION_ENABLED=false
FOD_MOTION_ENABLED=false
```

只有车辆架空、实体急停可用、CAN 端口已逐个确认后，才执行：

```bash
./scripts/authorize_motion.sh --confirm-elevated-estop
```

随后再修改配置、重新同步部署并以不高于 `0.3 m/s` 测试。测试结束立即执行：

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
