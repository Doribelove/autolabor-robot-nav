# Robot WS 导航项目上手说明

本工作空间是 ROS Noetic/catkin 项目，核心功能是 Autolabor 底盘、Livox MID360、FAST_LIO、GPS 和 Arena `move_base + TEB` 的真车无地图导航。

## 目录速览

- `scripts/bringup.sh`：导航一键启动主入口。
- `scripts/keyboard_drive.sh`：只启动底盘和键盘遥控，用于底盘测试，不启动导航。
- `scripts/rabbitmq_gps_goal_bridge.py`：RabbitMQ 到 ROS GPS 目标的桥接脚本。
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
- 主 GNSS 定位：`/gps/fix`、`/gps/pose`、`/gps/odom`
- 双天线 GNSS 航向：`/gps/heading`
- GPS 发布 `camera_init -> base_link`
- Arena nomap 导航

这个模式下位置来自主 GNSS 的 GGA 经纬度，车头 yaw 来自双天线 `UNIHEADINGA` 航向，FAST_LIO 只提供点云给 `/scan`。双天线航向静止时也可用，比单天线依赖运动轨迹推航向更适合室外 GPS 导航。默认不再使用底盘 `/odom`
融合 GPS 位置，避免底盘里程计坐标系和 GPS ENU 坐标系未对齐时把 `/gps/odom`
带偏。如果现场需要临时启用轮速辅助，可以设置：

```bash
GPS_USE_WHEEL_ODOM=true ./scripts/bringup.sh gps
```

GPS 模式默认要求双天线航向解算质量为：

- `solution_status=SOL_COMPUTED`
- `position_type=NARROW_INT`

主 GNSS 天线安装在底盘中心后方 0.3m，默认参数为：

- `GPS_ANTENNA_OFFSET_X=-0.3`
- `GPS_ANTENNA_OFFSET_Y=0.0`

GPS 节点会把 GGA 读到的主天线位置换算成 `base_link` 底盘中心位置，再发布 `/gps/pose`、`/gps/odom` 和 `camera_init -> base_link` TF。也就是说导航算法使用的是底盘中心位置，不是天线位置。

默认启动即可使用双天线航向：

```bash
./scripts/bringup.sh gps
```

如果需要临时放宽或改回旧的单天线运动航向方式，可以覆盖：

```bash
GPS_HEADING_SOURCE=auto ./scripts/bringup.sh gps
GPS_HEADING_SOURCE=gps_course ./scripts/bringup.sh gps
```

## RabbitMQ GPS 目标接入

RabbitMQ 脚本只负责把队列里的 GPS 目标发布到 ROS：

```text
RabbitMQ 消息 -> scripts/rabbitmq_gps_goal_bridge.py -> /gps/goal_fix -> gps_goal_node.py -> /move_base_simple/goal -> move_base/TEB -> /cmd_vel -> /m2_driver
```

当前 `scripts/rabbitmq_gps_goal_bridge.py` 的默认配置：

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
./scripts/rabbitmq_gps_goal_bridge.py
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
- 正常运动时使用 `position_filter_alpha=0.25` 做低通滤波。
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

先启动 GPS 导航：

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

输入数字执行对应任务：

- `1`：读取当前 `/gps/odom` 位置和双天线航向，发布车头正前方 `8m` 的 GPS 目标到 `/gps/goal_fix`。
- `2`：以当前车体朝向为坐标系，保存前后左右各 `10m` 的矩形电子围栏。围栏文件永久保存在 `/home/robot/robot_ws/config/gps_test_fence.json`，下次重新启动测试脚本会自动加载。
- `3`：在当前位置前后左右各 `10m` 范围内随机生成 GPS 目标并发布，用于阻拦车辆测试局部避障。
- `4`：显示当前围栏。
- `5`：清除永久围栏文件。

电子围栏只由 `scripts/gps_test_tasks.py` 监控：正常只运行 `./scripts/bringup.sh gps` 时不会受这个围栏约束。测试脚本运行时，会拒绝围栏外目标；如果当前 `/gps/odom` 跑到围栏外，会取消 `move_base` 目标并向 `/cmd_vel` 发布零速度。

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
- 因此 TEB 必须保持 `cmd_angle_instead_rotvel=False`，不能直接把 `/cmd_vel.angular.z` 当转角发布。
- 当前 GPS/nomap TEB 按“直线优先但保留避障机动性”调参：`max_vel_theta=1.5`、`acc_lim_theta=0.5`、`min_turning_radius=1.2`、`global_plan_viapoint_sep=0.8`、`weight_shortest_path=4.0`、`weight_viapoint=8.0`，并开启多拓扑路径搜索 `enable_homotopy_class_planning=True`。后方障碍参与距离降到 `costmap_obstacles_behind_robot_dist=0.8`，减少后方突然障碍导致的前后振荡。

现场确认：

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
```

期望分别是 `False`、`1.5`、`0.5`、`1.2`、`0.8`、`4.0`、`8.0`、`True`、`0.8`。

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
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/max_vel_x_backwards
rosparam get /move_base/TebLocalPlannerROS/min_vel_x
rosparam get /move_base/TebLocalPlannerROS/weight_kinematics_forward_drive
```

期望值分别是 `/gps/odom`、`0.5`、`6.283`、`1.5`、`1.0`、`0.0`、`20.0`。如果不是，重新使用一键脚本启动：

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
