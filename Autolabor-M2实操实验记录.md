# Autolabor-M2 实操使用指南

更新：2026-09-13｜工作区：`/home/robot/robot_ws`｜ROS Noetic / catkin

本文依据当前脚本和模块实现整理，保留已验收的 GNSS 导航用法。只说明现有操作，不把历史实验、开发设想或自动化测试结果作为实车验收结论。

## 1. 当前系统与环境

| 模块 | 当前用途与状态 |
| --- | --- |
| GNSS 导航 | 双天线 GNSS 提供位置与航向；MID360 点云用于避障；`move_base + TEB` 规划，M2 底盘执行 |
| 长距离目标 | 接收最终 WGS84 目标，默认以 15 m 前视滚动下发子目标，无需人工分段 |
| Qt 操作台 | 内嵌 RViz、经纬度目标、航向、相机/检测画面、模式切换、远程目标确认和录包 |
| 相机与 FOD 检测 | 海康 USB 相机驱动、FOD 专用 YOLO11 权重及可选曝光/增益控制已接入 |
| FOD 视觉回收 | GPS 启动时处于待机；操作员启用后执行像素对中与里程计直行，完成后恢复 GPS |
| 清扫装置 | 独立 TCP 开关脚本；尚未接入 ROS、Qt 控制或导航自动联动 |

导航不要求启动 Qt、相机、YOLO 或 RabbitMQ。GNSS 模式下 FAST_LIO 仅提供点云，不发布定位里程计与 TF；不要将该模式写成 GNSS/FAST_LIO 融合定位。

GNSS 串口持续读取，位置默认按 10 Hz 发布。导航保留 body 点云避障；世界坐标点云仅在有订阅者时生成，无地图模式不再启动旧 `viz_path` 节点。

所有命令均在工作区根目录执行。手动运行 `roslaunch`、`rostopic` 或 ROS Python 脚本的终端先加载环境：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

导航、Qt、录包等 `.sh` 入口会自行加载环境；已有构建可直接使用，不需要每次启动前重新编译。

| 设备 | 当前默认配置 |
| --- | --- |
| 底盘 CAN 转接器 | `/dev/ttyUSB0`；可用 `CAN_PORT` 覆盖 |
| 双天线 GNSS | `/dev/ttyUSB1`，115200；可用 `GPS_PORT`、`GPS_BAUD_RATE` 覆盖 |
| MID360 | 雷达 `192.168.1.125`，主机接收地址 `192.168.1.50`；见 [MID360 配置](src/perception_livox/livox_ros_driver2/config/MID360_config.json) |
| 海康相机 | USB，默认序列号 `DA7535899`；ROS 驱动运行时关闭 `hikrobot-mvs` 客户端 |
| 清扫装置 | 独立脚本实际连接 `192.168.0.197:50003`，与雷达不是同一网段 |

## 2. 启动导航：选择一种方式

### 原 GNSS 主入口

```bash
./scripts/bringup.sh gps 2.0 cruise
```

`cruise` 用于开阔道路和长直段；静态障碍场景可用 `./scripts/bringup.sh gps 1.0 obstacle`。速度单位为 m/s，表示规划上限，按现场既有配置选取，并受底盘上报上限约束。

`./scripts/bringup.sh gps` 无额外配置时默认为 `1.5 m/s + cruise`，启动独立 RViz。必须写明 `gps`；完全不带参数的 `bringup.sh` 默认是 `fast_lio`。

主终端显示 `Robot bringup is running in gps mode.` 表示整套导航启动完成。当前默认使用双天线航向，不需要手工输入初始车头方向。

### 一体化 Qt 入口

```bash
./scripts/operator_all_in_one.sh 2.0 cruise
```

启动 GNSS 导航、Qt 内嵌 RViz、相机/YOLO 和 RabbitMQ。Qt 会先打开，导航在后台初始化；窗口出现不等于导航已经就绪。该入口不带参数时默认为 `0.3 m/s + cruise`，与原 GNSS 入口不同。

可按需在命令前设置以下变量：

| 变量 | 作用 |
| --- | --- |
| `OPERATOR_START_VISION=false` | 不自动启动相机、YOLO 和图像质量控制器 |
| `OPERATOR_START_CAMERA=false` | 复用已启动的相机，仅启动后级视觉处理 |
| `OPERATOR_START_RABBITMQ=false` | 不启动远程目标桥接 |

例如只启动导航与 Qt：

```bash
OPERATOR_START_VISION=false OPERATOR_START_RABBITMQ=false \
  ./scripts/operator_all_in_one.sh 2.0 cruise
```

一体化入口默认启用图像质量控制，曝光上限为 `12000 μs`。日志位于 `log/operator_all_in_one_时间/`。关闭 Qt 或在该启动终端按 `Ctrl+C`，会停止此入口启动的整套进程。

### 分终端使用 Qt

```bash
# 终端 1：导航不另开 RViz
NAV_START_RVIZ=false ./scripts/bringup.sh gps 2.0 cruise

# 终端 2：只启动操作台，不重复启动导航
./scripts/operator_gui.sh
```

`operator_gui.sh` 本身不启动底盘、导航、相机或 RabbitMQ。它默认附带 GPS 静态误差监视器；已有监视器时使用 `./scripts/operator_gui.sh start_gps_error_monitor:=false`。

以上方式不要同时重复运行；已运行导航时添加界面用 `operator_gui.sh`，不要再运行一体化入口。

## 3. 下发目标与取消导航

- **GPS 目标**：Qt 输入 WGS84 纬度、经度，点击“发送 GPS 目标点”。不要混用经纬度顺序或其他地图坐标。
- **局部目标**：RViz 使用 `2D Nav Goal`，Fixed Frame 为 `camera_init`，发布到 `/move_base_simple/goal`。人工局部目标会替换正在执行的 GPS 管理路线。
- **航向显示**：双天线航向以正北为 `0°`、正东为 `90°`，顺时针增加；不是 ROS 局部坐标系的 yaw 数值。

不使用 Qt 时，可在已加载 ROS 环境的终端输入实际目标：

```bash
read -r -p "目标纬度（WGS84）: " M2_TARGET_LAT
read -r -p "目标经度（WGS84）: " M2_TARGET_LON
rostopic pub -1 /gps/goal_fix sensor_msgs/NavSatFix \
  "{header: {frame_id: gps}, status: {status: 0, service: 1}, latitude: ${M2_TARGET_LAT}, longitude: ${M2_TARGET_LON}, altitude: 0.0}"
```

GNSS 目标由长距离管理器转成 `/move_base/goal`；无需手工启动旧 `gps_goal.launch`。默认最终目标距离上限为 1000 m。

需要接收 RabbitMQ 目标且桥接尚未启动时：

```bash
./scripts/rabbitmq_gps_goal_bridge.py
```

接收消息只缓存 `TARGETS` 中最后一个有效目标，不会立即导航。终端输入 `1` 发送、`2` 清空；一体化入口中使用 Qt 的“确认发送缓存目标”和“清空缓存”。连接可通过 `RABBITMQ_HOST`、`RABBITMQ_PORT`、`RABBITMQ_USER`、`RABBITMQ_PWD`、`RABBITMQ_QUEUE` 等已有环境变量配置，本文不保存凭据。

取消当前导航：Qt 点击“取消当前导航目标”，或执行：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
```

取消导航不等于退出 FOD 视觉控制。结束整套作业时，在车辆停稳后退出对应主启动入口；紧急情况使用原有物理急停。

## 4. 可选作业模块

### 相机与 FOD 识别

相机/检测未运行时，在已加载 ROS 环境的新终端执行：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

只看相机可使用 `roslaunch hikrobot_mvs_camera fod_camera.launch`；相机已经启动时，上述检测命令改为 `start_camera:=false`。一体化入口已经启动视觉时，不再重复执行。

Qt 可查看原图、标注图及曝光/增益，也可单独查看：

```bash
rqt_image_view /fod/debug/image
```

原图为 `/fod_camera/image_raw`，结构化检测为 `/fod/detections`。当前使用 `src/yolo/fod_yolo11n_img640_e300_orig/weights/best.pt`，不是通用 COCO 权重。YOLO 环境为 `/home/robot/python_env/fod_yolo`；仅缺失时使用 `./scripts/setup_fod_yolo_env.sh` 安装。

固定手工曝光/增益前，先在 Qt 停用图像质量控制，再应用手工值。相机安装外参尚未确认，米制地面投影默认关闭；识别框不能直接视为已标定的地面坐标。

### GPS / FOD 视觉回收切换

GNSS 导航和相机/检测已运行时，在 Qt 点击“立即单独启动”，或使用：

```bash
./scripts/fod_mode.sh start
./scripts/fod_mode.sh status
```

此操作会让车辆进入视觉行驶，不只是启动相机。现有模式管理器暂停并保留 GPS 目标，停车后切换到视觉控制；视觉完成后自动恢复 GPS。视觉回收期间不接受新的 GPS 目标。

主动退出视觉模式并恢复原 GPS 路线：

```bash
./scripts/fod_mode.sh stop
```

`stop` 的含义是“恢复 GPS”，不是整车停车。视觉 `ABORT` 或故障时不会自动恢复，应根据状态处理后再决定是否恢复。

联动运行只用上述入口；不要并行启动独立 `visual_recovery.launch`，也不要直接调用 `/fod_visual_servo/set_enabled`。视觉回收不等于清扫机构已经开启。详细边界见 [视觉回收说明](src/application/autolabor_fod_control/README.md)。

### 清扫装置

仅在需要切换清扫机构状态时执行：

```bash
python3 src/SweepDeviceControl/SweepDeviceControl.py
```

脚本连接 `192.168.0.197:50003`，查询状态、发送开关切换指令并再次查询后退出。它会操作真实设备，不是只读查询，也不是固定“开机”命令；没有命令行开/关参数。目标地址在脚本 `__main__` 中设置，使用时需有到该设备网段的网络连接。

该模块目前未接入 ROS、Qt 或 FOD 自动流程；Qt“清扫”页仅为预留页面。

## 5. 录包与专项工具

按需在另一个终端记录本次运行：

```bash
BAG_PREFIX=m2_run ./scripts/record_rosbag.sh mode1
```

`Ctrl+C` 结束并保存，默认写入当前目录，可用 `BAG_DIR` 指定目录。`mode1` 覆盖规划/执行速度、GNSS 与底盘里程计、前轮角度、感知、导航目标、GPS/FOD 状态和 ROS 日志；Qt 也有“开始 mode1 录包”按钮，不必重复录制。录包和 `build/`、`devel/` 等生成目录不提交 Git。

以下工具均为可选，不是日常导航的启动步骤：

| 工具 | 入口与使用边界 |
| --- | --- |
| GPS 静态误差 | `roslaunch robot_diagnostics gps_static_error_monitor.launch`；Qt 默认已启动，不要重复运行 |
| GPS 实验菜单 | `./scripts/gps_test_tasks.py`；导航启动完成后运行，含前方 8 m 目标、随机目标、围栏和 FOD 测试记录 |
| 键盘底盘控制 | `./scripts/keyboard_drive.sh`；仅用于独立底盘操作，不与导航或视觉行驶并行 |
| 前轮中心标定 | [标定工具源码](src/tools/robot_diagnostics/scripts/steer_center_calibration.py)；专项运动调试，不在本指南中套用历史偏置值或自动运行 |

实验菜单会加载已保存的围栏，并在脚本运行期间实施越界干预；原 GNSS 主入口不会自动启用该实验围栏。导航重启后，实验菜单也应重新启动。

## 6. 其他模式与保留事项

- FAST_LIO 定位使用 `./scripts/bringup.sh fast_lio`；`fast_lio_gps` 是兼容入口，不是融合定位模式。其定位来自 FAST_LIO，TEB 速度反馈来自底盘 `/odom`；GNSS 主入口仍使用 `/gps/odom`。
- 已修复 GNSS 串口积压、启动路由误判及无效资源消耗，原启动命令和控制参数保持不变，未新增门控或看门狗。
- S 形行驶的实车结论仍待当次记录确认，见 [导航流程核查记录](NAVIGATION_STRAIGHT_LINE_AUDIT.md)；代码与离线验证不等于已经通过路测。
