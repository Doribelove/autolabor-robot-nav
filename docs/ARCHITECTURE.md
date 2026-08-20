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
接口名不是身份依据：NVIDIA 启动器分别用永久 MAC `6C:1F:F7:C4:82:83`
（ASIX USB 网卡，经扩展坞和交换机连接 J6M）和 `50:54:7B:E3:C9:10`（MID360）
解析当前接口名并绑定 NetworkManager 配置。

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
```

FAST-LIO 始终只使用 MID360 原始点和 IMU，前后二维雷达不会污染定位。move_base 的避障输入是独立的 `/scan`：MID360 是强制主源，LD19 是可超时移除的增强源。

静态建图是独立数据链：`/cloud_registered` 体素累积为三维 PCD；
`/dual_lidar/scan + /Odometry` 生成纯双 LD19 二维占据图；停止后再把三维图在
LD19 高度带投影并以占据并集合成第三张图。普通“录包”不会触发该流程。

不传 `--map-set` 时启动器保持无图 FAST-LIO 模式。传入地图集后，FAST-LIO 仍按
原始算法输出高频 `camera_init -> body` 里程计；独立的 `fast_lio_localization` 节点
加载 PCD，在 `/initialpose` 附近执行低频粗到精 scan-to-map ICP，估计
`map -> camera_init`。map_server 只为 move_base 加载二维图，不启动 AMCL；ICP
质量不合格、数据过期或定位丢失时，速度门控输出零速度。

原始 Livox 数据跨机只建立一组 relay 订阅，避免 FAST-LIO、点云转换各自重复传输大消息。默认不跨机发送 ZED RGB、深度图或完整视觉点云。

Qt/RViz 默认显示 `/cloud_registered_body_enhanced`，用于直接检查 MID360 与可选 LD19 的空间效果。该显示会让完整点云跨机传输；需要节省带宽时可在 Displays 中取消勾选。融合后的 `/scan` 也默认显示并持续用于避障。

## 控制链

```text
J6M move_base
  /cmd_vel_navigation
        ↓
J6M /fod_navigation_mode
  /cmd_vel_safe
        ↓  ROS TCP/IP
NVIDIA /nvidia_cmd_vel_watchdog（250 ms lease、0.3 m/s 上限）
  /cmd_vel
        ↓
NVIDIA /m2_driver → USB-CAN → M2 底盘
```

`/fod_navigation_mode` 是 `/cmd_vel_safe` 的唯一发布者；NVIDIA 看门狗是 `/cmd_vel` 的唯一发布者。看门狗拒绝 NaN/Inf、非平面指令、超限指令、重复发布者、错误订阅者和过期命令。

## 运行位置

| 功能 | NVIDIA | J6M |
|---|---:|---:|
| ROS master |  | 是 |
| Livox 物理驱动 | 是 |  |
| FAST-LIO |  | 是 |
| 前后 LD19 驱动/初次融合 | 是 |  |
| MID360 + LD19 避障融合 |  | 是 |
| move_base + TEB |  | 是 |
| USB-CAN/M2 | 是 |  |
| ZED、CUDA、YOLO | 是 |  |
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
