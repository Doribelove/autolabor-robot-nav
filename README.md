# Robot WS 导航项目上手说明

本工作空间是 ROS Noetic/catkin 项目，核心功能是 Autolabor 底盘、Livox MID360、FAST_LIO、GPS 和 Arena `move_base + TEB` 的真车无地图导航。

## 目录速览

- `scripts/bringup.sh`：导航一键启动主入口。
- `scripts/keyboard_drive.sh`：只启动底盘和键盘遥控，用于底盘测试，不启动导航。
- `scripts/shu copy.py`：RabbitMQ 到 ROS GPS 目标的桥接脚本。
- `src/scripts/robot_bringup/launch/`：CAN、Livox、FAST_LIO、GPS、Arena 导航的封装 launch。
- `src/application/gps_module/`：GPS 定位和 GPS 经纬度目标转换。
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

启动内容：

- CAN 底盘
- Livox MID360
- FAST_LIO 点云配准，但关闭 FAST_LIO odom/TF
- 点云过滤和投影：`/scan`
- GPS 定位：`/gps/fix`、`/gps/pose`、`/gps/odom`
- GPS 发布 `camera_init -> base_link`
- Arena nomap 导航

这个模式下定位来自 GPS，FAST_LIO 只提供点云给 `/scan`。默认不再使用底盘 `/odom`
融合 GPS 位置，避免底盘里程计坐标系和 GPS ENU 坐标系未对齐时把 `/gps/odom`
带偏。如果现场需要临时启用轮速辅助，可以设置：

```bash
GPS_USE_WHEEL_ODOM=true ./scripts/bringup.sh gps
```

## RabbitMQ GPS 目标接入

RabbitMQ 脚本只负责把队列里的 GPS 目标发布到 ROS：

```text
RabbitMQ 消息 -> scripts/shu copy.py -> /gps/goal_fix -> gps_goal_node.py -> /move_base_simple/goal -> move_base/TEB -> /cmd_vel -> /m2_driver
```

当前 `scripts/shu copy.py` 的默认配置：

- host：`39.98.47.163`
- port：`5672`
- user/password：`caacsriUser`
- vhost：`/`
- queue：`collection_vehicle`
- ROS 发布话题：`/gps/goal_fix`

消息中需要有 `TARGETS`，每个目标至少包含 `LAT` 和 `LON`。

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
./scripts/shu\ copy.py
```

也可以这样执行带空格的文件名：

```bash
"./scripts/shu copy.py"
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
./scripts/shu\ copy.py
```

GPS 定位模式会对 TEB 做更宽松的到点设置：

- `odom_topic=/gps/odom`
- `xy_goal_tolerance=1.5`
- `yaw_goal_tolerance=6.283`
- `max_vel_x=0.1`
- `max_vel_x_backwards=0.1`
- `min_vel_x=0.0`
- `min_vel_x_backwards=0.0`
- `penalty_epsilon=0.03`

这样做是为了避免 GPS 定位噪声和无意义的终点朝向约束导致车辆接近目标后仍持续前进。

GPS 定位节点还会对原始 GPS 位置做滤波：

- 静止低速时，如果 GPS 抖动在 `stationary_hold_radius=0.8m` 内，保持当前位置不漂移。
- 正常运动时使用 `position_filter_alpha=0.25` 做低通滤波。
- 明显跳点超过 `max_fix_jump=5.0m` 会被拒绝。
- 速度低于 `heading_min_speed=0.05m/s` 时，不使用 RMC course 更新车头方向。

单天线 GPS 静止时无法知道车头绝对朝向。默认 `initial_yaw=0.0`，车辆开始运动后会用
RMC course 更新航向。如果现场已知初始车头方向，可以传弧度值，例如：

```bash
GPS_INITIAL_YAW=1.5708 ./scripts/bringup.sh gps
```

也可以直接传手机指南针读数，脚本会自动换算成 ROS 需要的弧度：

```bash
GPS_COMPASS_HEADING="东北45度" TERMINAL_MODE=split ./scripts/bringup.sh gps
GPS_COMPASS_HEADING_DEG=45 TERMINAL_MODE=split ./scripts/bringup.sh gps
```

指南针角度约定是 `0=北`、`90=东`、`180=南`、`270=西`，也就是手机指南针常见的从正北开始顺时针计数。脚本换算公式是 `ROS yaw = 90度 - 指南针角度`。如果只写方向不写数字，也支持 `GPS_COMPASS_HEADING=正北|东北|正东|东南|正南|西南|正西|西北`；如果同时设置了 `GPS_INITIAL_YAW` 和指南针输入，优先使用 `GPS_INITIAL_YAW`。

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
- `/cmd_vel` 是否连接：发布者应有 `/move_base`，订阅者应有 `/m2_driver`。

常用手动检查命令：

```bash
rostopic list
rostopic echo /gps/fix
rostopic echo /gps/goal_fix
rostopic echo /move_base_simple/goal
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
- 控制输出：`/cmd_vel`
- 导航全局帧：`camera_init`
- 机器人底盘帧：`base_link`

导航流程里 CAN 底盘 launch 使用 `publish_tf:=false`，避免底盘自己的 `odom -> base_link` TF 和 FAST_LIO/GPS 的定位 TF 冲突。单独键盘测试脚本默认允许底盘发布 TF。

## 常见问题

### RabbitMQ 收到消息但车不走

先检查是否有 GPS 目标转换节点：

```bash
rosnode list | grep gps_goal
```

再检查链路：

```bash
rostopic echo /gps/goal_fix
rostopic echo /move_base_simple/goal
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

### move_base 有目标但 /cmd_vel 没接到底盘

检查：

```bash
rostopic info /cmd_vel
```

期望看到：

- publisher：`/move_base`
- subscriber：`/m2_driver`

### GPS 模式下车一直往目标前进但不判定到达

先确认 GPS 模式下 TEB 参数被正确覆盖：

```bash
rosparam get /move_base/TebLocalPlannerROS/odom_topic
rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/yaw_goal_tolerance
rosparam get /move_base/TebLocalPlannerROS/min_vel_x
```

期望值分别是 `/gps/odom`、`1.5`、`6.283`、`0.0`。如果不是，重新使用一键脚本启动：

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
