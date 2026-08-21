# robot_j6m_ws 双机工作空间

这是从 `/home/slam/robot_ws` 当前工作树提取出的独立 ROS Noetic 工作空间。原工作空间及其 68 项未提交改动没有被修改；新目录不包含旧 Git 历史，也没有复制旧 `build`、`devel`、日志、rosbag 或虚拟环境。

## 当前状态

- NVIDIA 端 22 个包已用 Release 配置编译通过；当前测试汇总为 399 项通过、0 失败。
- `fast_lio_localization` 已通过编译、相关测试和现有 bag 重放验证；覆盖清扫新增几何、
  Qt、静态地图启动、暂停桥和部署契约测试均已纳入同一测试汇总。
- J6M Ubuntu 20.04/ROS Noetic chroot 已存在于 `/map/autolabor_runtime/rootfs`。
- J6M 版本 `20260821_180229` 已部署并通过远端静态健康检查。
- 两机 ROS 普通消息和 Livox 自定义消息已在当前临时网段双向验证。
- J6M relay、FAST-LIO、避障融合、增强点云、move_base 与 FOD 仲裁已在实机数据流中完整拉起，并通过连续启停清理验证。
- 三地图建图、跨帧静态过滤和地图同步已完成 bag 验证；当前 `latest` 为
  `map_20260821_filtered_final`，并已同步到 J6M。已知三维图定位采用 FAST-LIO
  高频里程计加低频多尺度 ICP。覆盖清扫 V1 已完成静态地图无运动冷启动和 Qt 目视
  验收，尚未发送 `/initialpose` 或执行实车覆盖路线行驶验收。

## 机器分工

NVIDIA：MID360 网口驱动、USB-CAN/M2、前后 LD19、ZED、YOLO、Qt/RViz，以及最终 `/cmd_vel` 看门狗。

J6M：ROS master、Livox topic relay、FAST-LIO、高频里程计上的已知地图 ICP 定位、
MID360/LD19 避障融合、map_server、move_base + TEB、FOD 安全仲裁。

原 `/home/slam/robot_ws` 继续承担机场 GPS 模式；不要用其中的旧一体化脚本启动本双机模式。

## 第一次接线与配置

在 NVIDIA 主机：

1. WCH USB 转网口适配器（MAC `50:54:7B:E3:C9:10`）直连 MID360，NVIDIA 为 `192.168.1.50`，MID360 为 `192.168.1.112`。
2. ASIX USB 千兆网口（MAC `6C:1F:F7:C4:82:83`）接入 USB 扩展坞，扩展坞 RJ45 接交换机；J6M 也连接该交换机。NVIDIA 为 `192.168.10.50`，J6M 为 `192.168.10.100`。
3. USB-CAN、前 LD19、后 LD19 均连接 NVIDIA。
4. 用 VS Code 编辑 [dual_host.env](/home/slam/robot_j6m_ws/config/dual_host.env)，不要猜串口。

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
```

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
`eth0/eth1/eth2` 名称判断 USB 网卡，启动器会按 MAC 识别。

以下命令均在 NVIDIA 主机执行。

### 无静态地图启动

日常建图、录包或只需要 FAST-LIO 增量里程计时，使用默认启动：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_dual_host.sh
```

不传 `--map-set` 时不会加载历史 PCD，也不会启动 map_server 和已知地图定位器。
FAST-LIO 在本次启动建立的 `camera_init` 坐标系内输出增量里程计。这也是 Qt
“录入静态地图”功能要求的运行模式。

### 加载静态地图启动

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

1. 等待二维静态地图显示出来，并确认 `/Odometry` 已有数据。
2. 点击工具栏 `2D Pose Estimate`。
3. 在地图中车辆实际所在位置按下鼠标，并沿车头方向拖动后松开。这里指定的是
   `base_link` 在 `map` 中的二维位置和朝向，不是 MID360 的安装位置。
4. 等待 `/fast_lio/localization_status` 变为 `LOCALIZED`，再发送导航目标。

也可以从命令行发布初值。以下示例仅适用于车辆回到该地图的建图起点、车头方向与
建图开始时一致的情况。`/initialpose` 表示 `base_link` 的位姿，定位器会根据
`MID360_SENSOR_X/Y` 自动换算到 FAST-LIO 的 `body`，因此不要手工减去雷达安装偏置。
当前地图集 `map_20260821_filtered_final` 记录的首帧底盘位姿约为
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

只有状态消息以 `state=LOCALIZED;` 开头时，导航速度门才会放行 move_base 的速度。
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
4. 检查青色预定轨迹、可覆盖面积与不可覆盖估算。默认有效宽度为 `0.70 m`、重叠率
   `15%`；这两个参数可在规划前修改。
5. 点击“开始覆盖清扫”。后端会按车辆当前全局位置重新做已知自由空间连通域裁剪，
   再导航到最近的合适轨迹端点并执行覆盖。

区域多边形表示“需要清扫的面积”，不是车辆地理围栏。转场和满足阿克曼最小转弯半径
的转弯可以离开多边形，但只能经过静态图中的已知自由空间；障碍物、未知格和与车辆
不连通的地图岛会被裁掉并计入不可覆盖估算。覆盖直线由
`autolabor_coverage/CoverageGlobalPlanner` 强制交给 move_base，普通点到点目标仍回退
Navfn；转场可选低速倒车，清扫直线禁止倒车。

动态障碍导致分段失败时，任务等待 `10 s`，最多重试 `3` 次后跳过；其余路线完成后
再最终重试 `1` 次，并以“部分完成”和阻塞分段数报告。FOD 进入视觉/安全暂停时，
覆盖管理器保留当前精确分段并在恢复后自行重发，暂停桥不会把覆盖端点当普通目标重发。
定位丢失会暂停且要求人工恢复。

V1 只完成导航覆盖，不控制主刷、边刷、风机或喷淋。点击开始前仍必须满足现场运动
安全条件；后端还会复核全局定位、NVIDIA 主运动门、FOD 目标放行、里程计新鲜度、
车辆静止和 move_base action server，任何一项失败都不会发车。

如果当前正在静态地图模式运行，要返回无图增量模式：

```bash
./scripts/start_dual_host.sh --restart
```

### 启停和状态管理

一键启动器会自动回收能由运行令牌或工作区来源严格确认的旧进程，按 MAC 找回可能
改名的 USB 网卡，再完成网络检查、J6M 时间同步、双端冷启动和运行态健康检查。
启动成功后栈由用户级 `autolabor-dual-host.service` 托管，终端可以直接关闭。

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
FAST-LIO 的 `camera_init`/地图坐标中截取 `Z=-0.756±0.10 m`：这个高度来自
`base_link -> body ≈ (0.211, 0.02329, 0.95588) m` 和 LD19 离地 `0.20 m`，即
LD19 扫描平面相对 FAST-LIO 的 IMU `body` 原点约为 `-0.756 m`。该二维栅格还必须
在至少 20 个不同 MID360 点云帧中出现才参与并集，因此跟车行人、飞点和单帧远点
不会直接固化为全局地图。若雷达高度、FAST-LIO 内部激光到 IMU 外参或建图初始姿态
发生变化，必须同步重算 `MAPPING_SLICE_CENTER_Z`；建图起点应位于水平地面并保持车体
姿态正常。

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
`./scripts/health_check.sh --runtime`。J6M 自身检查命令是：

```bash
/map/autolabor_runtime/dual_host/bin/health_check.sh
```

详细 topic 链和回滚说明见 [架构说明](/home/slam/robot_j6m_ws/docs/ARCHITECTURE.md)；当前未完成事项见 [工作交接](/home/slam/robot_j6m_ws/docs/HANDOFF.md)。
