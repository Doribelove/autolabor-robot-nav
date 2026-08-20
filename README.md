# robot_j6m_ws 双机工作空间

这是从 `/home/slam/robot_ws` 当前工作树提取出的独立 ROS Noetic 工作空间。原工作空间及其 68 项未提交改动没有被修改；新目录不包含旧 Git 历史，也没有复制旧 `build`、`devel`、日志、rosbag 或虚拟环境。

## 当前状态

- NVIDIA 端 21 个包已用 Release 配置编译通过。
- 启动前单元、ROS 集成和安全链共 377 项测试通过，0 失败。
- J6M Ubuntu 20.04/ROS Noetic chroot 已存在于 `/map/autolabor_runtime/rootfs`。
- J6M 版本 `20260820_141651` 已部署并通过静态及实机运行健康检查。
- 两机 ROS 普通消息和 Livox 自定义消息已在当前临时网段双向验证。
- J6M relay、FAST-LIO、避障融合、增强点云、move_base 与 FOD 仲裁已在实机数据流中完整拉起，并通过连续启停清理验证。
- 三地图建图、地图同步、固定三维图 FAST-LIO 重定位、map_server 和定位速度门已完成实机静止验证；尚未用完整场地图执行路线行驶验收。

## 机器分工

NVIDIA：MID360 网口驱动、USB-CAN/M2、前后 LD19、ZED、YOLO、Qt/RViz，以及最终 `/cmd_vel` 看门狗。

J6M：ROS master、Livox topic relay、FAST-LIO、MID360/LD19 避障融合、move_base + TEB、FOD 安全仲裁。

原 `/home/slam/robot_ws` 继续承担机场 GPS 模式；不要用其中的旧一体化脚本启动本双机模式。

## 第一次接线与配置

在 NVIDIA 主机：

1. WCH USB 转网口适配器（MAC `50:54:7B:E3:C9:10`）直连 MID360，NVIDIA 为 `192.168.1.50`，MID360 为 `192.168.1.112`。
2. ASIX USB 千兆网口（MAC `6C:1F:F7:C4:82:83`）接入 USB 扩展坞，扩展坞 RJ45 接交换机；J6M 也连接该交换机。NVIDIA 为 `192.168.10.50`，J6M 为 `192.168.10.100`。
3. USB-CAN、前 LD19、后 LD19 均连接 NVIDIA。
4. 用 VS Code 编辑 [dual_host.env](/home/slam/robot_j6m_ws/config/dual_host.env)，不要猜串口。

当前 MID360 安装外参以底盘 `base_link` 为参考：X 向车头 `+0.20 m`、Y 为 `0.0 m`、Z 为 `+0.90 m`，姿态与车体同向。更换安装位置后应同步修改 `dual_host.env` 中的 `MID360_SENSOR_X/Y/Z`，再重新部署并重启 J6M 栈。

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

## 正常启动

每次启动先给交换机、J6M 和 MID360 供电，并确认 ASIX USB 网卡经扩展坞到交换机、
J6M 到交换机、MID360 专用 USB 网卡到 MID360 的链路灯均已亮。不要按可能变化的
`eth0/eth1/eth2` 名称判断 USB 网卡，启动器会按 MAC 识别。

推荐在 NVIDIA 主机使用一键双机启动器：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_dual_host.sh
```

它会自动回收能由运行令牌或工作区来源严格确认的旧进程，按 MAC 找回可能改名的 USB 网卡，再完成网络检查、J6M 时间同步、双端冷启动和运行态健康检查。启动成功后栈由用户级 `autolabor-dual-host.service` 托管，终端可以直接关闭；桌面会话重启也不会再把 ROS 子进程遗留在主机上。完整现场交接见 [终端交接文档](/home/slam/robot_j6m_ws/docs/HANDOFF.md)。

常用管理入口：

```bash
./scripts/start_dual_host.sh --status   # 查看服务与完整运行态
./scripts/start_dual_host.sh --restart  # 同步清理后冷启动
./scripts/start_dual_host.sh --stop     # 同步停止并验证无残留
./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest
# 可选：--static-map-source fused（默认）或 lidar2d
```

脚本只回收可以严格证明属于本项目的进程，不会为了抢占 CAN 串口而杀死无关程序。需要观察前台监督器详细输出时，可使用 `./scripts/start_dual_host.sh --foreground`。

只有在分端诊断时，才按下面的手工顺序启动。

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

## 关键退化行为

- 前后 LD19 未连接：MID360 仍独立生成 `/scan`，FAST-LIO 与避障不受影响。
- 任一 LD19 数据超过 0.35 秒未更新：`/scan` 自动退回纯 MID360。
- MID360 或 IMU 缺失：J6M 主链不应进入可导航状态。
- CAN 未确认且 `REQUIRE_CAN=true`：J6M 一直等待，不会启动运动链。
- J6M 指令超过 0.25 秒未更新、节点所有权异常或 NVIDIA 视觉端退出：NVIDIA 看门狗持续输出零速度。

## 如何判断 FAST-LIO 定位质量

Qt 的“FAST-LIO”页把判断拆成可核查的证据，而不是只看 RViz 轨迹是否在动：

- `/Odometry` 与注册点云应稳定在约 `10 Hz`，IMU 应约 `200 Hz`；
- 三路数据年龄应分别小于约 `0.30 s / 0.30 s / 0.10 s`；
- `camera_init -> base_link` TF 必须连通；
- 近 2 秒不应出现大于 `0.15 m` 或 `5°` 的单帧跳变；
- 车辆静止满 5 秒后，窗口漂移小于 `0.05 m` 为正常，超过 `0.15 m` 判为异常；
- FAST-LIO 内部协方差只能反映估计器自信程度，不能替代外部真值测量。

界面综合为 `0–100` 分：`≥85` 且无关键故障为“健康”，`65–84` 为“注意”，
其余或任一关键流中断为“异常”。只有“健康”时 Qt 相对目标按钮才会放行。

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

均在 NVIDIA 执行：

```bash
./scripts/build_workspace.sh
./scripts/sync_j6m_time.sh
./scripts/deploy_j6m.sh
./scripts/health_check.sh --static
```

网络切换后可运行 `--network`，实际主链启动后运行 `--runtime`。J6M 自身检查命令是：

```bash
/map/autolabor_runtime/dual_host/bin/health_check.sh
```

详细 topic 链和回滚说明见 [架构说明](/home/slam/robot_j6m_ws/docs/ARCHITECTURE.md)；当前未完成事项见 [工作交接](/home/slam/robot_j6m_ws/docs/HANDOFF.md)。
