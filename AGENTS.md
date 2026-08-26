# robot_j6m_ws Codex 工作约束

本文件适用于 `/home/slam/robot_j6m_ws` 整个目录树。进入子目录工作时也必须遵守。

## 信息源与优先级

开始任何修改前，至少阅读：

1. 本文件；
2. `README.md`；
3. 与任务有关时再读 `docs/ARCHITECTURE.md`、`docs/HANDOFF.md` 和对应脚本。

`/home/slam/AGENTS.md` 要求读取的全局交接文档记录的是更早期的
`/home/slam/robot_ws` 迁移状态，其中旧网络拓扑、旧设备归属和“尚未完成”结论可能
已经过时。发生冲突时，本项目当前的 `AGENTS.md`、`README.md`、
`config/dual_host.env`、实际脚本和实时只读检查优先。

本项目是室内 J6M 双机版本；`/home/slam/robot_ws` 继续承担机场 GPS 模式。不要用
旧工作区的一体化脚本启动本项目，也不要顺手修改、同步或清理旧工作区。

## 源码和工作区保护

- 工作树可能包含用户已有的 tracked、untracked 和子模块修改。先运行
  `git status --short`，只改任务直接涉及的文件，保留所有无关变化。
- 未经明确要求，不得执行 `git reset`、`git checkout --`、`git clean`、提交、推送、
  删除地图/rosbag/日志/发布版本或重写用户配置。
- 不得刷写 J6M 的 ACORE、MCU、boot、system、整盘镜像或机器人固件。
- 不要把 `/home/slam/robot_ws` 或 J6M 上的无 Git 源码副本当作本项目权威源码；权威
  开发目录是 `/home/slam/robot_j6m_ws`。
- 不要为了抢占串口或 ROS 节点而使用宽泛的 `pkill`/`killall`。项目脚本只停止具有
  PID、运行令牌或严格工作区来源证据的进程；无法证明归属时应报告并停止操作。

## 双机职责和 ROS 拓扑

NVIDIA 本机负责：

- MID360 物理网口驱动；
- USB-CAN/M2、前后 LD19；
- ZED、CUDA、YOLO11、Qt/RViz；
- `/cmd_vel` 最终看门狗。

J6M 负责：

- ROS master；
- Livox topic relay、FAST-LIO、已知地图 ICP 定位；
- MID360/LD19 避障融合、map_server、move_base + TEB；
- FOD 安全仲裁。

两端是两个独立进程环境，但加入同一 ROS1 图：

```text
J6M:    ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.100
NVIDIA: ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.50
```

不得设置 `ROS_HOSTNAME`。ROS1 节点通信使用动态 TCP 端口，两端必须双向可达，不能
只验证 11311。不要默认跨机发送 ZED 未压缩图像、深度图或完整视觉点云；跨机 topic
应维持现有最小数据链。

节点运行位置由两端不同的启动入口决定，不是由 ROS 自动调度：

- NVIDIA：`scripts/nvidia_gateway.sh` / `nvidia_gateway.launch`；
- J6M：远程 `deploy/j6m/start.sh` 最终执行已安装的 `j6m_stack.sh` 和
  `j6m_fastlio_navigation.launch`；
- `scripts/start_dual_host.sh` 通过 SSH 启动 J6M，再启动 NVIDIA 端，并由用户级
  `autolabor-dual-host.service` 托管完整进程树。

## 本机编译与 J6M 部署是两套产物

严禁假定本机 `devel` 会自动出现在 J6M。

### NVIDIA 本机产物

`./scripts/build_workspace.sh` 在 NVIDIA 本机执行 `catkin_make`，产物位于本项目的
`build/` 和 `devel/`。`scripts/setup_env.sh` 加载本机 `devel/setup.bash`，因此本机
节点使用本机产物。

### J6M 产物

`./scripts/deploy_j6m.sh` 执行另一条独立部署链：

1. 通过 `rsync` 只发送选定源码，明确排除本机 `build`、`devel`、`install`；
2. 通过 SSH 在 J6M 的 Ubuntu 20.04 ARM64 chroot 内原生执行
   `catkin_make install`；
3. 生成 `/map/autolabor_runtime/rootfs/opt/autolabor/dual_host/releases/<时间戳>/install`；
4. 完成 `rospack`、共享库和 launch 检查后，原子切换
   `/opt/autolabor/dual_host/current` 符号链接；
5. J6M 启动时依次加载 `/opt/ros/noetic`、基础
   `/opt/autolabor/ros/install` 和项目版本 `/opt/autolabor/dual_host/current`。

不要复制 NVIDIA 的 `devel` 或本机 ELF 到 J6M，也不要让 J6M 直接运行源码目录中的
未安装脚本。J6M 使用版本化 catkin install 空间，回滚只允许在停止主链后通过
J6M 上的 `/map/autolabor_runtime/dual_host/bin/rollback.sh` 切换 `current`。

当前 `deploy_j6m.sh` 只同步并编译其 `paths` 和
`CATKIN_WHITELIST_PACKAGES` 中的包，主要包括：

```text
conventional
teb_local_planner
fast_lio
fast_lio_localization
robot_bringup
autolabor_dual_lidar
autolabor_fod_control
autolabor_dual_host
```

`livox_ros_driver2` 等当前来自 J6M 基础
`/opt/autolabor/ros/install`。若修改了不在部署白名单中的包，不得声称普通
`deploy_j6m.sh` 已让修改在 J6M 生效；应先核对依赖，再明确扩展同步路径和白名单，
或按用户授权更新基础 install。自定义消息有变化时必须保证两端消息定义和 MD5 一致。

### 修改后的选择

- 只影响 NVIDIA 节点：本机重新构建、相关测试、完整冷重启；通常无需部署 J6M。
- 影响 J6M 节点、J6M launch/config 或两端共享包：停止双机栈，本机构建和静态检查，
  执行 `deploy_j6m.sh`，同步时间，再完整冷启动。
- 同一个包在两端都有节点：本机 `devel` 和 J6M 版本化 `install` 都必须更新。
- 静态地图不是程序发布物；使用现有 `sync_static_map.sh` 或带 `--map-set` 的一键启动
  同步，不要塞进程序 release。

推荐发布流程均在 NVIDIA 执行：

```bash
./scripts/start_dual_host.sh --stop
./scripts/build_workspace.sh
./scripts/health_check.sh --static
./scripts/deploy_j6m.sh
./scripts/sync_j6m_time.sh
```

部署属于外部状态修改；若用户只要求解释、审查或诊断，不要擅自部署。

## 网络与设备身份

当前固定拓扑：

```text
MID360 192.168.1.112
  ↕
NVIDIA WCH USB Ethernet 192.168.1.50

J6M eth0 192.168.10.100
  ↕ 交换机
NVIDIA ASIX USB Ethernet 192.168.10.50

NVIDIA wlan0：互联网默认路由
```

NVIDIA USB 网卡的 `eth0/eth1/eth2` 名称会在重启后变化，绝不能把接口名当作硬件身份：

```text
6C:1F:F7:C4:96:B8  ASIX -> 交换机/J6M
50:54:7B:E3:C9:10  WCH  -> MID360
```

- 使用 `scripts/load_config.sh` 的硬件身份解析结果；永久 MAC 未命中时，只允许用
  `config/dual_host.env` 中同时配置且唯一匹配的 USB VID:PID + serial 兜底。不得仅凭
  VID:PID、驱动名或现存 `ethN` 自动认领网卡。
- 正常启动会在任何 J6M 远程停机前调用 `scripts/network_prepare.sh`，恢复
  NetworkManager 托管、修正 profile、等待载波/地址/两端可达；不要把这个顺序改回
  “先远程 stop，后修网络”。同步停机后还必须再次复检网络。
- NetworkManager 持久 profile 应绑定永久 MAC、`connection.autoconnect=yes`，并让
  `connection.interface-name` 保持为空。空接口名是预期状态，不要“修复”为某个
  易变的 `ethN`；激活时可以临时传入当前 `ifname`。
- 两个机器人有线接口必须位于不同 `/24`，且不得接管 Wi-Fi 默认路由。
- J6M 固定地址由厂商硬件持久配置 `hrut_ipfull` 保存；不要依赖易失 overlay 中的
  `/etc` 网络文件，也不要未经确认删除 J6M 默认路由。
- `scripts/network_check.sh` 同时检查 J6M 和 MID360。若它失败，要逐项区分是哪条链路，
  不得把 MID360 未供电误报成 J6M 网络仍故障。

首次网络配置只在两端设备均已接线供电且用户授权时执行：

```bash
sudo ./scripts/configure_network.sh --apply
./scripts/network_check.sh
```

常用只读诊断：

```bash
ip -br address
ip route
nmcli -t -f NAME,TYPE,DEVICE connection show
./scripts/network_check.sh
```

J6M 重启后时钟可能失效；网络恢复后运行 `scripts/sync_j6m_time.sh`。不要把错误时间
引起的启动拒绝误判为编译或 ROS 故障。

## 串口规则

不要猜测 `/dev/ttyUSBn`：

- J6M 串口控制台：
  `/dev/serial/by-path/platform-3610000.xhci-usb-0:4.1.1:1.2-port0`，`921600` baud；
- CAN：`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0`；
- 前后 LD19：`/dev/autolabor/lidar_front`、`/dev/autolabor/lidar_rear`，由稳定物理 USB
  路径产生；改变插口后必须重新做逐个拔插确认。

先运行 `./scripts/discover_devices.sh`，再核对 `config/dual_host.env`。出现
`IN_USE_BY_PID` 时只报告占用者或安全停止能证明属于本项目的进程，不得杀死未知进程。
使用 screen 诊断后必须退出并释放串口。

## ZED 相机就绪判定

- ZED 2 视频端 `2b03:f780` 必须以 `5000M` 或更高的 USB 3.x 速率枚举；`480M`
  是 USB 2.0 降级，不能仅凭 `2b03:f780/f781` 出现在 `lsusb` 就判定正常。
- 启动或诊断先运行 `./scripts/zed_camera_check.sh --wait 0`。`f780/f781` usbfs 必须对
  `slam` 可访问；hidraw 是可选内核接口，不得因它未生成而单独拒绝已能被 ZED SDK
  识别的相机。重启后 usbfs 若遗留为 `root:root 0600`，使用已安装的
  `autolabor-zed-coldplug.service`，首次安装入口为 `scripts/install_zed_udev.sh`。
- `/zed2/zed_node` 存活不是相机 ready。必须确认 `/fod_camera/image_raw` 和
  `/fod_camera/depth_registered` 有新鲜消息；否则检查 `log/nvidia_ui_*/vision.log`。
- 不要用可能争抢相机的 ZED Explorer/Diagnostic 作为运行栈内健康探针；确需运行时
  先通过统一入口完整停止双机栈，诊断完再完整冷启动。

## 启停、地图和运行态

正常操作只使用统一入口：

```bash
./scripts/start_dual_host.sh             # 无图 FAST-LIO 模式
./scripts/start_dual_host.sh --status
./scripts/start_dual_host.sh --restart   # 完整冷重启
./scripts/start_dual_host.sh --stop      # 同步停止并验证残留
```

- 不要只重启 `nvidia_gateway.sh`；Livox 断流而 J6M FAST-LIO 未同步重置曾导致位姿严重
  发散。修改和故障恢复优先完整冷重启。
- 手工分端启动仅用于诊断，不能替代一键脚本的地图同步、时间同步和生命周期管理。
- 静态地图模式使用 `--map-set global_maps/map_sets/latest`。切换地图模式必须冷重启。
- 静态地图冷启动不会复用历史位姿；必须由操作员根据车辆真实位置发送
  `/initialpose`，并等 `/fast_lio/localization_status` 为 `LOCALIZED`。不得猜测初值，
  也不得把 FAST-LIO 健康等同于全局定位成功。
- `map_server` 只加载二维地图；全局定位由 `fast_lio_localization` 提供
  `map -> camera_init`，本模式不使用 AMCL。
- 定位前 Qt/RViz 依靠 `operator_gui.launch` 的
  `map -> autolabor_map_display_anchor` 静态叶子识别二维地图根帧。该叶子绝不能改成
  `map -> camera_init/base_link`，也不得据此声称车辆已定位；它仅解决初始位姿前的
  地图渲染循环依赖。
- Qt 收到 `/map` 不等于内嵌 RViz 已完成渲染。静态地图启动验收还必须要求
  `/autolabor_operator_gui/map_display_status` 为 `READY`；保留 MapDisplay 自动重订阅和
  实际宽、高、分辨率校验，禁止把这项检查降级成仅检查 `/map` 有消息。
- 静态建图必须从无图模式开始，并确认 MID360、IMU 和要求的固定物理口 LD19 在线；
  普通 rosbag 录制不会自动生成地图。

日志位置：

```text
NVIDIA: /home/slam/robot_j6m_ws/log/
J6M:    /map/autolabor_runtime/logs/dual_host/
```

## 运动安全与运动门交付状态

本项目的正常交付状态为 `MOTION_ENABLED=true`。诊断、部署、停机或排障过程中可以按需
临时改为 `false`，但任务结束前必须恢复为 `true`，同步到 J6M，并核对实际运行的
`nvidia_gateway.launch` 参数也是 `motion_enabled:=true`。除非用户明确要求，禁止把
`MOTION_ENABLED=false` 作为最终状态遗留。`FOD_MOTION_ENABLED` 是独立功能门，不得因
恢复主底盘运动门而擅自联动修改。

`runtime/motion_authorized.ok` 是独立的启动安全门：用户已确认车辆架空、人员远离车轮、
实体急停可用且 CAN 端口已确认后，可以创建并保留；不得在部署、重启或任务收尾时静默
删除。若现场条件未确认或标记缺失，不得代替用户虚构确认，应先询问用户。

即使主运动门已经恢复，除非用户在当前任务中明确授权运动测试，并且现场同时满足车辆
架空、人员远离车轮、实体急停可用、CAN 端口逐个确认、速度限制不高于 `0.3 m/s`，否则：

- 不发布导航目标、非零 `/cmd_vel` 或会恢复旧目标执行的控制指令；
- 不把“启动成功”“`/cmd_vel` 非零”或“定位为 LOCALIZED”当作底盘已经安全执行；
- 诊断控制链时只允许观察、健康检查和零速验证。

只有用户明确要求撤销运动授权时，才执行：

```bash
./scripts/authorize_motion.sh --revoke
```

MID360/IMU 缺失、静态定位未完成或退化、CAN 未确认、指令过期、节点所有权异常时，
现有安全门应保持零速。修改相关代码时必须保留这些 fail-closed 行为，并为安全边界添加
或更新测试。

## J6M chroot 和持久数据

- J6M rootfs：`/map/autolabor_runtime/rootfs`，Ubuntu 20.04 ARM64 用户态，使用 J6M
  自身内核。
- `/dev`、`/proc`、`/sys`、`/run`、`/tmp` 及持久配置/地图/日志目录由脚本 bind mount。
- 不要把地图、日志、FAST-LIO PCD、ROS home 或其他运行数据写进 overlay 根目录；使用
  `/map/autolabor_runtime/{config,maps,fast_lio,ros-home,logs}` 对应持久目录。
- 不要直接修改某个历史 release。通过新部署创建新 release，失败时保持 `current`
  不变；回滚前必须先停止 J6M 主链。

## 验证和交付

根据改动风险选择验证，不能只说“应该可以”：

- Shell 修改至少运行 `bash -n` 和 `git diff --check`；
- Python/ROS 包运行对应单元测试或 rostest；
- 通用静态检查使用 `./scripts/health_check.sh --static`；
- 网络修改使用 `./scripts/health_check.sh --network` 或 `scripts/network_check.sh`，并区分
  J6M 与 MID360 的结果；
- 部署后运行 J6M 的
  `/map/autolabor_runtime/dual_host/bin/health_check.sh`；
- 主链启动后才运行 `./scripts/health_check.sh --runtime`；
- 改动两端消息、launch 或控制链时，验证节点实际运行主机、topic 发布者/订阅者、消息
  MD5、频率、时间新鲜度和双向网络；
- 未经明确要求不要为验证重启整机、部署、启动实车主链或连接运动授权。

最终交付必须说明：改了哪些文件、验证了什么、是否部署到 J6M、J6M `current` 指向
哪个 release，以及仍受哪些离线硬件或安全条件限制。不要把本机构建成功描述成远端
部署成功，也不要把静态检查描述成实机运行验收。
