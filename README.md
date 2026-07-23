# Robot WS 导航项目上手说明

本工作空间是 ROS Noetic/catkin 项目，核心功能是 Autolabor 底盘、Livox MID360、FAST_LIO、GPS 和 Arena `move_base + TEB` 的真车无地图导航。

## 目录速览

- `scripts/bringup.sh`：导航一键启动主入口。
- `scripts/operator_gui.sh`：启动 Qt 操作与诊断台（内嵌 RViz）。
- `scripts/keyboard_drive.sh`：只启动底盘和键盘遥控，用于底盘测试，不启动导航。
- `scripts/rabbitmq_gps_goal_bridge.py`：RabbitMQ 到 ROS GPS 目标的桥接脚本。
- `src/application/autolabor_operator_gui/`：Qt5 主窗口、状态页和内嵌式 RViz。
- `src/application/autolabor_operator_msgs/`：RabbitMQ 状态和远程目标的结构化 ROS 消息。
- `src/scripts/robot_bringup/launch/`：CAN、Livox、FAST_LIO、GPS、Arena 导航的封装 launch。
- `src/application/gps_module/`：GPS 定位和 GPS 经纬度目标转换。
- `src/perception_camera/hikrobot_mvs_camera/`：海康 MVS 工业相机 ROS 驱动。
- `src/tools/robot_diagnostics/`：一键启动中的 topic、TF、odom、CAN 检查工具。
- `src/navigation_arena/arena-rosnav-3D/`：Arena 导航、`move_base`、TEB、costmap 配置。

## 基础环境

每个手动开的终端都先进入工作空间并加载环境：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
```

一键脚本 `scripts/bringup.sh` 会自动 source 上面两个环境文件，所以直接运行脚本时不需要手动 source。

默认设备口：

- CAN 底盘：`/dev/ttyUSB0`
- GPS：`/dev/ttyUSB1`
- GPS 波特率：`115200`

可以用环境变量覆盖：

```bash
CAN_PORT=/dev/ttyUSB0 GPS_PORT=/dev/ttyUSB1 ./scripts/bringup.sh fast_lio
```

## 导航启动方式

一键启动入口：

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh fast_lio
```

`bringup.sh` 支持 3 种导航模式。

### 与海康相机同时使用

MVS SDK 自带的旧版 `libusb-1.0.so.0` 会覆盖 Ubuntu 系统 libusb，而
FAST_LIO 依赖的 PCL 需要新版符号。`bringup.sh` 只在 FAST_LIO 子进程中
优先加载系统库；相机及其他 ROS 节点的动态库环境保持不变。因此不需要
修改原来的导航命令，也不要在整个桌面会话中全局覆盖 `LD_LIBRARY_PATH`。

终端 1 正常启动导航：

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh gps 2.7 cruise
```

等待导航打印 `Robot bringup is running in gps mode.` 后，在终端 2 启动
相机。启动前先关闭会独占相机的 `hikrobot-mvs` 桌面客户端：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch hikrobot_mvs_camera fod_camera.launch
```

需要相机和 FOD 检测器一起启动时，终端 2 改用：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch start_camera:=true
```

### 1. FAST_LIO 定位导航

```bash
./scripts/bringup.sh fast_lio
```

启动内容：

- CAN 底盘：`/canbus_driver`、`/m2_driver`
- Livox MID360：`/livox/lidar`、`/livox/imu`
- FAST_LIO 定位：`/Odometry`、`/cloud_registered_body`
- 点云过滤和投影：`/scan`
- GPS 读数：`/gps/fix`
- GPS 目标转换节点：订阅 `/gps/goal_fix`，发布 `/move_base_simple/goal`
- Arena nomap 导航：`/move_base`
- TEB 输出控制：`/cmd_vel`

这是推荐的一键启动模式。定位来自 FAST_LIO；GPS 不发布定位 TF，只用于接收经纬度目标并转换成局部导航目标。因此 RabbitMQ 桥接脚本可以直接配合这个模式使用。

### 2. FAST_LIO GPS 兼容模式

```bash
./scripts/bringup.sh fast_lio_gps
```

该模式保留给旧启动习惯使用。目前它和 `fast_lio` 一样：车辆定位使用 FAST_LIO，GPS 只用于接收经纬度目标并转换成 `/move_base_simple/goal`。

如果经纬度目标方向和 FAST_LIO 坐标系有偏差，可以设置角度偏置：

```bash
FAST_LIO_GPS_YAW_OFFSET_DEG=10 ./scripts/bringup.sh fast_lio
```

### 3. GPS 定位导航

```bash
./scripts/bringup.sh gps
```

可以用第二个位置参数设置 GPS 定点导航的最大速度，用第三个位置参数选择 TEB 场景：

```bash
# 场景 1：空旷道路、园区主路、长直道
./scripts/bringup.sh gps 2.0 cruise

# 场景 2：仓库货架、固定设施、密集静态障碍
./scripts/bringup.sh gps 1.0 obstacle
```

不传第二个参数时仍使用默认 `1.5m/s`；不传第三个参数时默认使用 `cruise`。第二个参数覆盖 `GPS_NAV_MAX_VEL_X`，并保证倒车上限不会高于它；默认倒车上限仍是 `1.0m/s`，不会因为前进上限改为 `2.0m/s` 而自动提高。CAN 底盘启动后，`bringup.sh` 会读取 `/m2_driver/chassis_parameter`，把 TEB 的前进和倒车规划上限同时收紧到车辆上报的 `max_speed`；M2 驱动仍保留最终硬件钳制。这样规划轨迹和底盘真实能力使用同一速度上限。

两套场景都是在原始 dingo nomap 参数上叠加的小型覆盖文件：

| 场景 | 目标 | 主要调整 |
|---|---|---|
| `cruise` | 快速、稳定、长直道少摆动 | 局部滚动窗口 `16×16m`，TEB 前视 `6.5m`，固定当前目标的全局参考线，降低转向/路径回正增益，关闭多拓扑候选切换 |
| `obstacle` | 提前绕过固定障碍、路径稳定、少倒车 | 将局部滚动窗口扩为 `24×24m`，有效 TEB 前视设为 `10m`，规划最小半径 `1.35m`，保留 4 个同伦拓扑并提高障碍代价和前进约束 |

`cruise` 仍保持 `0.1m` 分辨率，因此局部代价地图由原先的 `200×200`
网格减少为 `160×160`，单层网格数量减少约 `36%`。TEB 代码只使用地图
半宽的 `85%`，理论边界为 `6.8m`；显式配置 `6.5m` 前视以留出边界余量。
正前方 `8m` 目标仍由全局规划器完整管理，TEB 随滚动地图连续执行局部段。
`obstacle` 模式继续使用 `24×24m`，不受该缩减影响。

高速 `cruise` 还专门抑制偏离路径后的左右过量修正：移动状态 GPS 位置滤波
系数由通用值 `0.25` 提高为 `0.70`；按 GPS 位置 10Hz、车速 2.7m/s 估算，
一阶滤波的稳态纵向滞后由约 `0.81m` 降为约 `0.12m`。全局规划频率设为
`0Hz`，因此控制过程中沿用下发目标时生成的参考路线，不会每秒从已经偏移的
车身位置重新画一条路线；收到新目标或规划失败时仍会重新规划。TEB 同时使用
2 个轨迹位姿平均控制量，并把巡航角速度/角加速度限制为 `0.85rad/s` 和
`0.45rad/s²`。这些设置不会增加途中停车条件；真正的急弯可能由 TEB 主动降速。
`obstacle` 保留 `position_filter_alpha=0.25` 和 `planner_frequency=1Hz`，继续
周期性适应障碍环境。

配置文件分别是 `config/teb_profiles/gps_cruise.yaml` 和 `config/teb_profiles/gps_obstacle.yaml`。也可以不用第三个参数，改用环境变量选择：

```bash
GPS_TEB_PROFILE=obstacle ./scripts/bringup.sh gps 1.0
```

#### 接近目标时平缓减速

GPS 模式默认在 TEB 和底盘之间启用目标减速器：

```text
move_base/TEB -> /cmd_vel_navigation -> gps_goal_speed_limiter -> /cmd_vel -> m2_driver
```

它根据当前目标距离施加 `v ≤ sqrt(2 × a × 剩余距离)` 的前进速度上限，默认舒适减速度 `a=0.4m/s²`、TEB 到点容差 `0.3m`、减速器硬停半径 `0.2m`、最低接近速度 `0.15m/s`。硬停半径必须严格位于规划器成功半径内，避免减速器先停车、TEB 却因严格距离判断一直保持 `ACTIVE`。以 `2.0m/s` 行驶时，约在距目标中心 `5.2m` 开始逐步限速；以 `1.5m/s` 行驶时约在 `3.0m` 开始。

这个限制只处理接近目标时仍然过高的正向线速度：

- TEB 为避障给出的更低速度或零速度立即通过。
- 倒车恢复速度不限制。
- 如果正向线速度被降低，`/cmd_vel.angular.z` 会按同一比例降低，保持阿克曼曲率，不会因为限速反而要求更大的前轮转角。
- 收到匹配当前 GoalID 的 `/move_base/cancel` 或终止状态时立即锁存完整零速度；只有新的带身份 `/move_base/goal` 才能解除，延迟的旧取消或 `/move_base/current_goal` 不能误停、误解锁。
- 首次进入目标中心 `0.2m` 后立即锁存完整零速度；后续 GPS 抖到阈值外也不会让旧目标重新起步。
- 首次进入 `1.0m` 后启动终点围栏；超过 `15s` 仍未到达，或相对最近点退离 `0.5m`，立即锁存停车，避免在目标附近无限徘徊。
- 当前目标期间 GPS odom 缺失、超时、坐标非有限或坐标系不一致时失效停车，不透传旧速度。

需要更柔和、提前更远减速时，减小舒适减速度，例如：

```bash
GPS_GOAL_COMFORTABLE_DECEL=0.3 ./scripts/bringup.sh gps 2.0 cruise
```

终点围栏也可按现场 GPS 精度调整：

```bash
GPS_GOAL_NEAR_COMMIT_DISTANCE=1.0 \
GPS_GOAL_NEAR_TIMEOUT=15.0 \
GPS_GOAL_NEAR_MAX_REGRESSION=0.5 \
./scripts/bringup.sh gps 2.7 cruise
```

如需现场对比旧行为，可临时关闭：

```bash
GPS_GOAL_SLOWDOWN_ENABLED=false ./scripts/bringup.sh gps 2.0 cruise
```

#### 现场导航录包

下一轮 GPS 实车测试建议在另一个终端启动标准录包：

```bash
cd /home/robot/robot_ws
BAG_DIR=/tmp BAG_PREFIX=gps_validation ./scripts/record_rosbag.sh mode1
```

`mode1` 同时记录 `/cmd_vel_navigation`、`/cmd_vel`、GPS/底盘 odom、原始
`/canbus_msg`、`/m2_driver/chassis_monitor`、`/m2_driver/control_timeout`、
左右轮速、前轮转角、目标和 TEB 计划，能够区分导航主动停车与底盘保护。

启动内容：

- CAN 底盘
- Livox MID360
- FAST_LIO 点云配准，但关闭 FAST_LIO odom/TF
- 点云过滤和投影：`/scan`
- 主 GNSS 定位：`/gps/fix`、`/gps/pose`、`/gps/odom`
- 双天线 GNSS 航向：`/gps/heading`
- GPS 发布 `camera_init -> base_link`
- Arena nomap 导航

这个模式下位置来自主 GNSS 的 GGA 经纬度，车头 yaw 来自双天线 `UNIHEADINGA` 航向，FAST_LIO 只提供点云给 `/scan`。双天线航向静止时也可用，比单天线依赖运动轨迹推航向更适合室外 GPS 导航。默认不使用底盘 `/odom` 积分位姿，避免底盘里程计坐标系和 GPS ENU 坐标系未对齐时把 `/gps/odom` 带偏；但会使用 `/odom.twist` 中新鲜的有符号线速度和角速度，让 TEB 能看到车辆真实运动。底盘 twist 超过 `0.5s` 未更新时会回退到 GNSS 运动估计。

`cruise` 和 `obstacle` 当前都默认关闭双天线航向跳变保护，导航直接使用每一帧
满足 `SOL_COMPUTED + NARROW_INT` 的实时航向。真车记录表明，旧保护在持续小偏差
下可能长时间保持旧 yaw，最终让 TF 方向错误并诱发 TEB 徘徊，因此正常巡航不要
启用它。原始 `/gps/heading` 仍可用于现场诊断。

为避免继承 shell 中可能残留的旧设置，也可以显式指定关闭：

```bash
GPS_HEADING_JUMP_GUARD_ENABLED=false \
./scripts/bringup.sh gps 2.7 cruise
```

航向数据超时或质量降级时，原有定位安全门和停车逻辑仍然有效。

如果现场确实需要临时启用轮式里程计的位姿积分，可以设置：

```bash
GPS_USE_WHEEL_ODOM=true ./scripts/bringup.sh gps
```

GPS 模式默认要求双天线航向解算质量为：

- `solution_status=SOL_COMPUTED`
- `position_type=NARROW_INT`

在默认严格双天线模式下，启动时只有收到新鲜且满足上述质量的航向后才会发布
`/gps/pose`、`/gps/odom` 和导航 TF；运行中航向超过 `1.0s` 未更新或质量降级时，
这些导航输出会暂停，目标减速器随后因 odom 超时输出完整零速度。`/gps/fix` 会继续
发布，便于诊断，但不会拿陈旧航向继续导航。

启动器默认等待最多 `120s` 让双天线从 `NARROW_FLOAT` 收敛到
`NARROW_INT`，可通过 `GPS_ODOM_STARTUP_TIMEOUT` 调整。等待期间 GPS
终端会打印当前基线长度和航向标准差；如果超时后仍是 `NARROW_FLOAT`，应检查
车辆是否位于开阔天空、两个天线和馈线是否连接可靠，以及接收机定向/固定基线
配置，不应在高速导航中放宽该质量门槛。

不启动 ROS、只读一帧 GGA 和双天线航向质量：

```bash
./scripts/test_gps_heading_reader.py --port /dev/ttyUSB1 --timeout 15
```

主 GNSS 天线安装在底盘中心后方 0.3m、右方 0.05m。按照 `base_link`
的 X 向前、Y 向左约定，默认参数为：

- `GPS_ANTENNA_OFFSET_X=-0.3`
- `GPS_ANTENNA_OFFSET_Y=-0.05`

GPS 节点会把 GGA 读到的主天线位置沿车体向前补偿 0.3m、向左补偿
0.05m，换算成 `base_link` 底盘中心位置，再发布 `/gps/pose`、`/gps/odom`
和 `camera_init -> base_link` TF。也就是说导航算法使用的是底盘中心位置，
不是天线位置。

默认启动即可使用双天线航向：

```bash
./scripts/bringup.sh gps
```

如果需要临时放宽或改回旧的单天线运动航向方式，可以覆盖：

```bash
GPS_HEADING_SOURCE=auto ./scripts/bringup.sh gps
GPS_HEADING_SOURCE=gps_course ./scripts/bringup.sh gps
```

## Qt 操作与诊断台

第一版界面已包含内嵌 RViz，以及 ROS、CAN、GNSS、双天线航向、雷达、
`move_base`、RabbitMQ 和录包状态。分页可查看 GPS/局部坐标、现有
静态漂移指标、RabbitMQ 缓存目标与消息计数，并提供车头正前方
`8m` GPS 测试、取消导航、重置静态误差和一键 `mode1` 录包。相机/
YOLO 与清扫装置页面已预留，后续可按独立 ROS 接口接入。

首次编译：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=''
source devel/setup.bash
```

使用内嵌 RViz 时，先只禁用原 launch 中的独立 RViz，其他原有节点不变：

```bash
# 终端 1：原导航流程，仅不再额外打开独立 RViz
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise

# 终端 2：Qt 操作台
./scripts/operator_gui.sh
```

不设置 `NAV_START_RVIZ=false` 时，`bringup.sh` 仍默认打开原来的独立 RViz，
所以旧流程完全保留。Qt 界面不发布 `/cmd_vel`；CAN、GNSS、雷达、
导航或 RabbitMQ 节点缺失时只显示离线，不会阻止界面启动。
内嵌 RViz 默认以地图为主，可用右侧按钮显示 Displays 调试面板；
它不提供绕过前置检查的 `2D Nav Goal`，人工任意点目标仍使用原独立 RViz。
RabbitMQ 桥接原有终端 `1/2` 确认流程仍可独立使用；界面另外使用
`/rabbitmq_bridge/publish_latest` 和 `/rabbitmq_bridge/clear_latest` 服务。

如果已经单独启动静态误差监视节点，可避免重复启动：

```bash
./scripts/operator_gui.sh start_gps_error_monitor:=false
```

## RabbitMQ GPS 目标接入

RabbitMQ 脚本负责接收队列消息、缓存并打印最新 GPS 点；只有操作员确认后才发布到 ROS：

```text
RabbitMQ 消息 -> scripts/rabbitmq_gps_goal_bridge.py 缓存并打印最新 GPS 点 -> 操作员输入 1 -> /gps/goal_fix -> gps_goal_node.py -> /move_base_simple/goal -> move_base/TEB -> /cmd_vel_navigation -> 目标减速器 -> /cmd_vel -> /m2_driver
```

当前 `scripts/rabbitmq_gps_goal_bridge.py` 的默认配置：

- host：`39.98.47.163`
- port：`5672`
- user/password：`caacsriUser`
- vhost：`/`
- queue：`collection_vehicle`
- ROS 发布话题：`/gps/goal_fix`

消息中需要有 `TARGETS`，每个目标至少包含 `LAT` 和 `LON`。例如当前外部设备发送：

```json
{
  "CMD": "1007",
  "DEVICE": "100",
  "TARGETS": [
    {
      "DATA": "1783068596237_0.jpg",
      "LAT": 30.674179252383237,
      "LON": 104.52607620185489,
      "TIME": "1783068596237.000000",
      "TYPE": "0",
      "URL": "http://gips2.baidu.com/it/u=195724436,3554684702&fm=3028&app=3028&f=JPEG&fmt=auto?w=1280&h=960"
    }
  ]
}
```

桥接收到有效消息后不会自动发送导航目标，而是：

- 提取并保存 `TARGETS` 中最后一个有效点。
- 在当前桥接终端打印 `LAT`、`LON`、`TYPE`、`TIME`、`DATA`、`URL` 和接收时间。
- 输入 `1`：把当前保存的点发布到 `/gps/goal_fix`，无人车开始处理该导航目标。
- 输入 `2`：清空当前保存的点。
- 新消息会覆盖内存中原来保存的点；退出或重启桥接程序后，内存缓存也会清空。

### 推荐 RabbitMQ 启动流程

终端 1，启动导航和 GPS 目标转换：

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh fast_lio
```

终端 2，启动 RabbitMQ 桥接：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

桥接连接成功后保持该终端在前台。收到消息并核对打印的经纬度无误后，在提示符 `GPS操作>` 后输入：

```text
1
```

只有这时目标才会从 `/gps/goal_fix` 进入导航链路。若不需要该点，输入：

```text
2
```

### GPS 定位模式接 RabbitMQ

终端 1：

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh gps
```

终端 2：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

GPS 定位模式会对 TEB 做更宽松的到点设置：

- `odom_topic=/gps/odom`
- `xy_goal_tolerance=0.5`
- `yaw_goal_tolerance=6.283`
- `max_vel_x=1.5`
- `max_vel_x_backwards=1.0`
- `min_vel_x=0.0`
- `min_vel_x_backwards=0.0`
- `penalty_epsilon=0.03`
- `weight_kinematics_forward_drive=20.0`

这样做是为了避免 GPS 定位噪声和无意义的终点朝向约束导致车辆接近目标后仍持续前进。

GPS 定位节点还会对原始 GPS 位置做滤波：

- 静止低速时，如果 GPS 抖动在 `stationary_hold_radius=0.8m` 内，保持当前位置不漂移。
- 正常运动时，`cruise` 使用 `position_filter_alpha=0.70` 降低高速控制相位滞后；`obstacle` 和直接启动定位节点仍使用 `0.25`。
- 明显跳点超过 `max_fix_jump=5.0m` 会被拒绝。
- 默认使用双天线 `UNIHEADINGA` 更新车头方向，发布 `/gps/heading`，并用于 `/gps/odom` 和 `camera_init -> base_link` TF。

如果没有双天线航向、临时改回 `GPS_HEADING_SOURCE=gps_course`，单天线 GPS 静止时仍无法知道车头绝对朝向。此时可以传初始弧度值，例如：

```bash
GPS_INITIAL_YAW=1.5708 ./scripts/bringup.sh gps
```

也可以直接传手机指南针读数，脚本会自动换算成 ROS 需要的弧度：

```bash
GPS_COMPASS_HEADING="东北45度" TERMINAL_MODE=split ./scripts/bringup.sh gps
GPS_COMPASS_HEADING_DEG=45 TERMINAL_MODE=split ./scripts/bringup.sh gps
```

指南针角度约定是 `0=北`、`90=东`、`180=南`、`270=西`，也就是手机指南针常见的从正北开始顺时针计数。脚本换算公式是 `ROS yaw = 90度 - 指南针角度`。如果只写方向不写数字，也支持 `GPS_COMPASS_HEADING=正北|东北|正东|东南|正南|西南|正西|西北`；如果同时设置了 `GPS_INITIAL_YAW` 和指南针输入，优先使用 `GPS_INITIAL_YAW`。

双天线航向在线检查：

```bash
rostopic echo /gps/heading
rosparam get /gps_localization/heading_source
rosparam get /gps_localization/heading_required_position_types
rosparam get /gps_localization/gps_antenna_offset_x
rosparam get /gps_localization/gps_antenna_offset_y
```

### GPS 电子围栏和避障测试任务

先启动 GPS 导航，并等待终端显示 `Robot bringup is running in gps mode.`：

```bash
cd /home/robot/robot_ws
./scripts/bringup.sh gps
```

另开终端启动测试菜单：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
./scripts/gps_test_tasks.py
```

测试脚本会绑定启动时的 ROS master。每次重启 `bringup.sh` 或 `roscore` 后，旧测试脚本都必须退出并重新启动；否则它不会自动注册到新的 ROS master。发布目标前，脚本会检查 `/gps/goal_fix -> /gps_goal -> /move_base_simple/goal -> /move_base`，并且只有实际观察到转换后的 `/move_base_simple/goal` 才打印“联动成功”。链路不完整时会拒绝发布并指出缺失节点，避免界面显示成功而无人车没有收到任务。

输入数字执行对应任务：

- `1`：读取当前 `/gps/odom` 位置和双天线航向，发布车头正前方 `8m` 的 GPS 目标到 `/gps/goal_fix`。
- `2`：以当前车体朝向为坐标系，保存前后左右各 `10m` 的矩形电子围栏。围栏文件永久保存在 `/home/robot/robot_ws/config/gps_test_fence.json`，下次重新启动测试脚本会自动加载。
- `3`：在当前位置前后左右各 `10m` 范围内随机生成 GPS 目标并发布，用于阻拦车辆测试局部避障。
- `4`：显示当前围栏。
- `5`：清除永久围栏文件。
- `6`：进入基于 `FOD_FINAL_TEST_TASKS.md` 的 FOD 回收装备最终测试菜单（T01～T08）。

电子围栏只由 `scripts/gps_test_tasks.py` 监控：正常只运行 `./scripts/bringup.sh gps` 时不会受这个围栏约束。测试脚本运行时，会拒绝围栏外目标；如果当前 `/gps/odom` 跑到围栏外，会取消 `move_base` 目标，由目标减速器锁存零速度。测试脚本不再直接发布 `/cmd_vel`，保证正式控制链始终只有一个速度发布者。

#### FOD 最终测试子菜单

在 GPS 测试主菜单输入 `6`：

```text
1: T01 设备与 GPS 检查（默认静止采样 120 秒）
2: T02 基础回收测试
3: T03 机坪处置效率测试（中心/左上/右下，共 3 次）
4: T04 滑行道处置效率测试（近/中/远，共 3 次）
5: T05 单个固定障碍物测试
6: T06 多个固定障碍物测试
7: T07 同一位置重复回收测试（共 3 次）
8: T08 异常与急停检查表
9: 显示测试记录和效率汇总
0: 返回 GPS 测试主菜单
```

T01 自动检查 `/canbus_msg`、`/gps/fix`、`/gps/odom`、`/gps/heading`、`/scan` 和 `/move_base/status`，并在车辆静止时采集 `/gps/odom`，计算相对首帧的位置 RMS、最大偏移和最大航向变化。清扫装置当前没有 ROS 状态接口，因此由现场人员确认是否已上电待机。

T02～T07 使用外部检测车和 RabbitMQ 桥接：

1. 在 FOD 测试终端选择测试项、完成现场布置，并按回车进入等待。
2. 检测车发送 FOD 消息。
3. 在 RabbitMQ 桥接终端确认经纬度后输入 `1`。
4. 测试脚本收到新的 `/gps/goal_fix` 时自动记录目标和开始时刻。
5. 车辆到达、清扫完成且确认 FOD 完全回收后，在测试终端按回车结束计时。
6. 按提示填写自主到达、避障、清扫、碰撞、越界、人工接管和停车结果。

测试记录持久化到：

```text
/home/robot/robot_ws/test_results/fod_final_test_records.jsonl
/home/robot/robot_ws/test_results/fod_final_test_records.csv
```

`test_results/` 是现场运行输出，已加入 `.gitignore`。T03 和 T04 使用大纲公式 `η = S / t_avg` 分别汇总，合格阈值为 `50m²/s`；T07 汇总三次导航成功率、回收成功率和平均时间。

T08 需要在低速且急停人员就位时完成。脚本只记录“通过/失败/跳过”，不会自动制造通信中断或触发实体急停。

`./scripts/bringup.sh gps` 打开的导航 RViz 已预置 `GPS Test Fence` 显示组，订阅 `/gps/test_fence_markers`。启动测试菜单并创建或加载围栏后，绿色线框会直接显示在同一个 RViz 中。

测试 `1` 会打印当前 `/gps/odom` 坐标、当前 yaw、目标坐标和直线距离。GPS 目标转换节点会把 `/gps/goal_fix` 转成 `/move_base_simple/goal`，目标姿态默认使用“当前位置指向目标点”的方向，不再固定为 yaw=0。测试前方 8m 时，目标姿态应接近当前车头方向。

如果测试 `1` 仍然调头或明显走反，优先检查：

```bash
rostopic echo /move_base_simple/goal
rostopic echo /gps/odom
rostopic echo /gps/heading
rosparam get /gps_goal/goal_yaw_mode
rosparam get /gps_goal/odom_topic
```

判断标准：`gps_test_tasks.py` 打印的目标 `x/y` 应和 `/move_base_simple/goal` 基本一致；目标 yaw 应接近从当前 `/gps/odom` 指向目标点的方向。如果目标点本身就在真实车头后方，说明 `/gps/odom` yaw 或双天线安装方向需要校正。

RViz 手动发送目标时：

- Fixed Frame 必须使用 `camera_init`。
- RViz 的 2D Nav Goal 应发布到 `/move_base_simple/goal`，消息里的 `header.frame_id` 应是 `camera_init`。
- GPS 转换节点和测试脚本不再 latch 发布目标，避免旧 GPS 目标在节点重连时覆盖 RViz 目标。
- 如果 RViz 目标点突然变化，先查 `/move_base_simple/goal` 当前有哪些 publisher。

```bash
rostopic info /move_base_simple/goal
rostopic echo /move_base_simple/goal
```

无障碍但车辆绕弯时，优先检查 TEB 和 M2 驱动的 `/cmd_vel` 语义：

- `m2_driver` 把 `/cmd_vel.angular.z` 当角速度处理，再换算成前轮转角。
- 倒车时换算必须保留 `/cmd_vel.linear.x` 的负号；同一个车体角速度下，
  前进和倒车所需的前轮转角符号相反。
- 因此 TEB 必须保持 `cmd_angle_instead_rotvel=False`，不能直接把 `/cmd_vel.angular.z` 当转角发布。
- GPS/nomap 的共用基础配置保留 `cmd_angle_instead_rotvel=False`；`cruise` 与 `obstacle` 都把 `min_turning_radius` 覆盖为 `1.35m` 并启用 `use_proportional_saturation=True`，给约 `1.22m` 的理论硬件极限留出转向余量。
- 当前 TEB fork 已让阿克曼 carlike 分支应用
  `weight_kinematics_forward_drive`。这个参数是前进软偏好，不是绝对禁止倒车。

现场确认：

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/planner_frequency
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/control_look_ahead_poses
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/use_proportional_saturation
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /move_base/local_costmap/width
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/max_number_classes
rosparam get /move_base/TebLocalPlannerROS/switching_blocking_period
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
rosparam get /gps_localization/position_filter_alpha
```

`cruise` 的关键期望值是：`cmd_angle_instead_rotvel=False`、
`planner_frequency=0.0`、`max_vel_theta=0.85`、`acc_lim_theta=0.45`、
`control_look_ahead_poses=2`、`min_turning_radius=1.35`、
`use_proportional_saturation=True`、`position_filter_alpha=0.70`；其 TEB 前视和
局部地图宽度应为 `6.5`、`16.0`。`obstacle` 的规划频率、角速度、角加速度、
位置滤波分别保持 `1.0`、`1.2`、`0.4`、`0.25`，前视和宽度为 `10.0`、
`24.0`。两者必须保持同比饱和，否则线速度单独被限幅时会把轨迹曲率放大到近满舵。

如果仍然出现前后振荡，记录这些信息：

```bash
rostopic echo /cmd_vel
rostopic echo /move_base/TebLocalPlannerROS/local_plan
rostopic echo /move_base/status
rostopic info /move_base_simple/goal
```

同时在 RViz 截图显示 `local_costmap`、`global_costmap`、`local_plan`、`global_plan` 和 `/scan`。

RViz 显示电子围栏：

- Fixed Frame 使用 `camera_init`。
- 导航 RViz 已预置 `GPS Test Fence` 显示组，Topic 为 `/gps/test_fence_markers`。
- 绿色线框是围栏边界，橙色点和文字分别标出 `front/back/left/right`。

### GPS 静止漂移监测

GPS 模式静止时，如果 `camera_init -> base_link` 或 `/gps/odom` 仍在漂移，会影响导航：move_base 会认为车在移动，目标距离、局部代价地图和避障控制都会抖动。

启动导航后，另开终端运行：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
roslaunch robot_diagnostics gps_static_error_monitor.launch
```

查看实时误差：

```bash
rostopic echo /gps/static_error/summary
rostopic echo /gps/static_error/current
rostopic echo /gps/static_error/rms
rostopic echo /gps/static_error/max
```

默认会等待 5 秒预热，然后把第一帧 `/gps/odom` 作为静止参考点。重置参考点：

```bash
rostopic pub -1 /gps/static_error/reset std_msgs/Empty "{}"
```

调试换算结果时可以不启动整车，只打印弧度值：

```bash
GPS_COMPASS_HEADING="东北45度" ./scripts/bringup.sh --print-gps-yaw
```

如果要现场调滤波强度，可从 `robot_bringup gps_localization.launch` 传参。

### 手动补启动 GPS 目标转换

正常使用 `fast_lio` 不需要手动补启动，因为一键启动已经包含 `gps_goal_node.py`。如果调试时只单独启动了部分 launch，缺少 GPS 目标转换节点，可以手动启动：

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
source /home/robot/robot_ws/devel/setup.bash
roslaunch gps_module gps_goal.launch frame_id:=camera_init
```

## 单独底盘键盘测试

不启动导航，只测 CAN 底盘和键盘控制：

```bash
cd /home/robot/robot_ws
./scripts/keyboard_drive.sh
```

该脚本会启动 roscore、CAN 底盘驱动，并运行 `autolabor_keyboard_teleop.py`。

## 一键启动内部检查

`bringup.sh` 会按顺序检查：

- CAN 设备是否存在且可读写。
- `/canbus_msg` 是否出现。
- `/livox/lidar`、`/livox/imu` 是否出现。
- FAST_LIO 模式下 `/Odometry`、`/cloud_registered_body` 是否出现。
- GPS 模式下 `/gps/fix`、`/gps/pose`、`/gps/odom` 是否出现。
- `camera_init -> base_link` TF 是否可用。
- `/scan` 是否出现。
- `/move_base/status` 和 costmap 是否出现。
- GPS 模式检查两段速度链路：`/cmd_vel_navigation` 应从 `/move_base` 到 `/gps_goal_speed_limiter`，`/cmd_vel` 应从减速器到 `/m2_driver`。FAST_LIO 模式仍是 `/move_base -> /cmd_vel -> /m2_driver`。

常用手动检查命令：

```bash
rostopic list
rostopic echo /gps/fix
rostopic echo /gps/goal_fix
rostopic echo /move_base_simple/goal
rostopic info /cmd_vel_navigation
rostopic info /cmd_vel
rosrun tf tf_echo camera_init base_link
rosnode list
```

## 关键坐标和话题

- FAST_LIO odom：`/Odometry`
- GPS odom：`/gps/odom`
- 点云输入：`/cloud_registered_body`
- 激光投影：`/scan`
- 导航目标：`/move_base_simple/goal`
- RabbitMQ GPS 目标中转：`/gps/goal_fix`
- GPS 导航原始控制：`/cmd_vel_navigation`
- 底盘最终控制：`/cmd_vel`
- 底盘控制器保护：`/m2_driver/chassis_monitor`（`*_stuck` 兼容字段实际表示协议 bit 2 电流超限）
- 底盘控制超时：`/m2_driver/control_timeout`
- 导航全局帧：`camera_init`
- 机器人底盘帧：`base_link`

导航流程里 CAN 底盘 launch 使用 `publish_tf:=false`，避免底盘自己的 `odom -> base_link` TF 和 FAST_LIO/GPS 的定位 TF 冲突。单独键盘测试脚本默认允许底盘发布 TF。

## 常见问题

### RabbitMQ 收到消息但车不走

新的桥接流程默认只保存目标，不会自动让车移动。先查看桥接终端是否已经打印最新 GPS 点，然后在 `GPS操作>` 后输入 `1`。

先检查是否有 GPS 目标转换节点：

```bash
rosnode list | grep gps_goal
```

再检查链路：

```bash
rostopic echo /gps/goal_fix
rostopic echo /move_base_simple/goal
rostopic info /cmd_vel_navigation
rostopic info /cmd_vel
```

如果是手动拆分启动的流程，确认已经启动：

```bash
roslaunch gps_module gps_goal.launch frame_id:=camera_init
```

### 一直等不到 /scan

检查 Livox 和 FAST_LIO：

```bash
rostopic echo /livox/lidar
rostopic echo /cloud_registered_body
rosnode list | grep laserMapping
```

`/scan` 来自 `robot_bringup scan_fast_lio.launch`，由 `pointcloud_self_filter` 和 `pointcloud_to_laserscan` 生成。

### move_base 有目标但速度没接到底盘

检查：

```bash
rostopic info /cmd_vel_navigation
rostopic info /cmd_vel
```

GPS 模式期望看到：

- `/cmd_vel_navigation`：publisher 为 `/move_base`，subscriber 为 `/gps_goal_speed_limiter`。
- `/cmd_vel`：publisher 为 `/gps_goal_speed_limiter`，subscriber 为 `/m2_driver`。

### GPS 模式下车一直往目标前进但不判定到达

先确认 GPS 模式下 TEB 参数被正确覆盖：

```bash
rosparam get /move_base/TebLocalPlannerROS/odom_topic
rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/yaw_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/max_vel_x_backwards
rosparam get /move_base/TebLocalPlannerROS/min_vel_x
rosparam get /move_base/TebLocalPlannerROS/weight_kinematics_forward_drive
```

默认 `cruise` 的期望值分别是 `/gps/odom`、`0.3`、`6.283`、`1.5`、`1.0`、`0.0`、`100.0`。如果以 `obstacle` 启动，最后一个值应为 `60.0`；如果传入了速度参数，`max_vel_x` 应等于该参数或底盘上报的更低 `max_speed`。如果不是，重新使用一键脚本启动：

```bash
./scripts/bringup.sh gps
```

再确认 GPS 定位和 GPS 目标转换使用的是同一个原点：

```bash
rosparam get /gps/origin_lat
rosparam get /gps/origin_lon
```

### 没有图形界面或不想自动开多个终端

```bash
TERMINAL_MODE=same ./scripts/bringup.sh fast_lio
```

### 串口不是默认值

```bash
CAN_PORT=/dev/ttyUSB2 GPS_PORT=/dev/ttyUSB3 ./scripts/bringup.sh fast_lio
```

### GPS 串口权限不足

如果启动时报：

```text
Device is not writable and non-interactive sudo is unavailable
```

先在本机终端执行：

```bash
sudo chmod 666 /dev/ttyUSB1
```

长期修复建议把 `robot` 用户加入串口设备组，然后重新登录：

```bash
sudo usermod -aG dialout robot
```

## 维护备注

当前根目录 `.git` 目录存在但内容为空，`git status` 不能正常使用。如果需要版本管理，需要重新初始化或恢复 `.git`。

`robot_bringup` 的 launch 里实际使用了 `pointcloud_to_laserscan` 和 `tf`，新环境部署时要确认这两个 ROS 包已经安装。
