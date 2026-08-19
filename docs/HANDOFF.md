# 双机项目终端交接

更新时间：2026-08-19（Asia/Shanghai）

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
8. 运行最终 `--runtime` 健康检查。

看到以下文字才表示完整启动成功：

```text
Dual-host project is ready and managed by autolabor-dual-host.service.
```

此时终端可以直接关闭。完整进程组由用户级 `autolabor-dual-host.service` 托管，不再依赖启动它的终端或图形桌面会话；任一关键进程退出时，服务会同步停止 NVIDIA 与 J6M，避免 FAST-LIO 在 Livox 断流后继续运行。

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
| Qt/RViz | 是 |  |

主要数据链：

```text
MID360 -> NVIDIA Livox driver -> J6M relay -> FAST-LIO
FAST-LIO -> /Odometry + registered cloud -> /scan -> move_base
Qt 相对目标 -> /move_base_simple/goal -> move_base
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

接口名称可能在重启后变化；启动器会按下列永久 MAC 自动找回接口并修正 NetworkManager 配置：

```text
6C:1F:F7:C4:82:83  ASIX 千兆网卡         -> 扩展坞 RJ45/交换机/J6M
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
- “综合、FAST-LIO、测试、视觉、清扫、日志”全部页签；
- 中央嵌入式 RViz 网格、MID360 点云和避障扫描；
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

## 安全状态

当前配置必须保持：

```text
MOTION_ENABLED=false
FOD_MOTION_ENABLED=false
```

且 `runtime/motion_authorized.ok` 不应存在。这样 CAN/M2 仍正常显示，但 NVIDIA 看门狗持续输出零速度。

只有车辆架空、人员远离车轮、实体急停可用，并明确要做低速运动测试时，才允许按项目原安全流程临时授权：

```bash
./scripts/authorize_motion.sh --confirm-elevated-estop
```

此前实测出现过 `/cmd_vel` 非零但左右轮速仍为零。下次运动测试必须优先检查 M2 控制模式、VCU 急停/制动输入与 CAN 下行帧，不能把非零 `/cmd_vel` 当作底盘已执行。

## 尚未闭环的可选硬件

前后 LD19 目前仍保持：

```text
DUAL_LIDAR_PORTS_CONFIRMED=false
```

本次接线出现 `/dev/ttyUSB1`～`/dev/ttyUSB4` 四路 FTDI 多串口，但以 `230400` 波特率被动读取均没有持续数据，无法安全判断哪两路是前/后 LD19。因此当前：

```text
/avoidance/source_mode = mid360
/avoidance/dual_lidar_active = false
```

MID360 会独立生成 `/scan`，FAST-LIO 与避障仍可正常运行。没有完成逐个拔插识别和前后方向确认前，不得把 `DUAL_LIDAR_PORTS_CONFIRMED` 改为 `true`。

## Jetson/ZED 注意事项

本次开机后曾出现 `/dev/nvhost-vic` 错误变为 `root:root 0600`，导致 ZED 节点段错误。正确状态应为：

```bash
stat -c '%a %U %G %n' /dev/nvhost-vic
# 期望：660 root video /dev/nvhost-vic
```

一键启动器会在启动 ZED 前检查访问权限并拒绝带故障启动。系统已有 `/etc/udev/rules.d/99-tegra-devices.rules`，其预期权限就是 `root:video 0660`。若重启后复发，需要管理员重新应用 udev 权限并重启 `nvargus-daemon`，再执行完整冷启动。

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

## 2026-08-16 最后检查点

- 完整双机运行态健康检查通过：262 项测试，0 失败。
- Qt、嵌入式 RViz、MID360 点云、ZED 画面和 YOLO 检测均已现场目视确认。
- `/cmd_vel` 与左右轮速均为 `0.0`，四类急停状态均为 `false`。
- 静止采样 60 帧：`x=-0.0359~-0.0241 m`、`y=-0.0486~-0.0338 m`、`z=0.0038~0.0058 m`。
- Qt 已改为室内 FAST-LIO 健康页和局部相对目标；GPS 目标与 RabbitMQ 模块已移除。
- 运动授权已撤销，当前仅允许观察、诊断和无运动导航链验证。
