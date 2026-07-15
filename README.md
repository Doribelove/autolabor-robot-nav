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

可以用第二个位置参数设置 GPS 定点导航的最大速度，用第三个位置参数选择 TEB 场景：

```bash
# 场景 1：空旷道路、园区主路、长直道
./scripts/bringup.sh gps 2.0 cruise

# 场景 2：仓库货架、固定设施、密集静态障碍
./scripts/bringup.sh gps 1.0 obstacle
```

不传第二个参数时仍使用默认 `1.5m/s`；不传第三个参数时默认使用 `cruise`。第二个参数覆盖 `GPS_NAV_MAX_VEL_X`，并保证倒车上限不会高于它；默认倒车上限仍是 `1.0m/s`，不会因为前进上限改为 `2.0m/s` 而自动提高。M2 驱动还会按底盘上报的硬件最高速度钳制 `/cmd_vel.linear.x`，因此这里设置的是导航规划器上限，不会绕过底盘硬件限制。

两套场景都是在原始 dingo nomap 参数上叠加的小型覆盖文件：

| 场景 | 目标 | 主要调整 |
|---|---|---|
| `cruise` | 快速、稳定、长直道少摆动 | 纵向加速度 `2.5`，角速度上限 `0.8`，角加速度 `0.3`，提高时间/直线路径权重，关闭多拓扑候选切换 |
| `obstacle` | 提前绕过固定障碍、路径稳定、少倒车 | 将局部滚动窗口扩为 `24×24m`，有效 TEB 前视设为 `10m`，保留 4 个同伦拓扑，将拓扑切换锁定时间提高到 `10s`，提高障碍代价和前进约束 |

配置文件分别是 `config/teb_profiles/gps_cruise.yaml` 和 `config/teb_profiles/gps_obstacle.yaml`。也可以不用第三个参数，改用环境变量选择：

```bash
GPS_TEB_PROFILE=obstacle ./scripts/bringup.sh gps 1.0
```

#### 接近目标时平缓减速

GPS 模式默认在 TEB 和底盘之间启用目标减速器：

```text
move_base/TEB -> /cmd_vel_navigation -> gps_goal_speed_limiter -> /cmd_vel -> m2_driver
```

它根据当前目标距离施加 `v ≤ sqrt(2 × a × 剩余距离)` 的前进速度上限，默认舒适减速度 `a=0.4m/s²`、到点容差 `0.5m`、最低接近速度 `0.15m/s`。以 `2.0m/s` 行驶时，约在距目标中心 `5.5m` 开始逐步限速；以 `1.5m/s` 行驶时约在 `3.3m` 开始。

这个限制只处理接近目标时仍然过高的正向线速度：

- TEB 为避障给出的更低速度或零速度立即通过。
- 倒车恢复速度不限制。
- `/cmd_vel.angular.z` 原样通过，因此绕障转向不受影响。
- 收到 `/move_base/cancel` 时立即持续输出零速度，直到 `move_base` 接受新目标，因此测试电子围栏的取消停车仍有最高优先级。

需要更柔和、提前更远减速时，减小舒适减速度，例如：

```bash
GPS_GOAL_COMFORTABLE_DECEL=0.3 ./scripts/bringup.sh gps 2.0 cruise
```

如需现场对比旧行为，可临时关闭：

```bash
GPS_GOAL_SLOWDOWN_ENABLED=false ./scripts/bringup.sh gps 2.0 cruise
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
- `6`：进入基于 `FOD_FINAL_TEST_TASKS.md` 的 FOD 回收装备最终测试菜单（T01～T08）。

电子围栏只由 `scripts/gps_test_tasks.py` 监控：正常只运行 `./scripts/bringup.sh gps` 时不会受这个围栏约束。测试脚本运行时，会拒绝围栏外目标；如果当前 `/gps/odom` 跑到围栏外，会取消 `move_base` 目标并向 `/cmd_vel` 发布零速度。

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
- 因此 TEB 必须保持 `cmd_angle_instead_rotvel=False`，不能直接把 `/cmd_vel.angular.z` 当转角发布。
- GPS/nomap 的共用基础配置保留 `min_turning_radius=1.2` 和 `cmd_angle_instead_rotvel=False`；`cruise` 与 `obstacle` 再分别覆盖速度平滑、前视、障碍代价和同伦拓扑参数。

现场确认：

```bash
rosparam get /move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel
rosparam get /move_base/TebLocalPlannerROS/max_vel_theta
rosparam get /move_base/TebLocalPlannerROS/acc_lim_theta
rosparam get /move_base/TebLocalPlannerROS/min_turning_radius
rosparam get /move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /move_base/local_costmap/width
rosparam get /move_base/TebLocalPlannerROS/weight_shortest_path
rosparam get /move_base/TebLocalPlannerROS/weight_viapoint
rosparam get /move_base/TebLocalPlannerROS/enable_homotopy_class_planning
rosparam get /move_base/TebLocalPlannerROS/max_number_classes
rosparam get /move_base/TebLocalPlannerROS/switching_blocking_period
rosparam get /move_base/TebLocalPlannerROS/costmap_obstacles_behind_robot_dist
```

`cruise` 的关键期望值依次是 `False`、`0.8`、`0.3`、`1.2`、`1.0`、`8.0`、`20.0`、`8.0`、`12.0`、`False`、`1`、`5.0`、`0.5`；`obstacle` 依次是 `False`、`1.2`、`0.4`、`1.2`、`0.6`、`10.0`、`24.0`、`3.0`、`4.0`、`True`、`4`、`10.0`、`0.8`。

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

默认 `cruise` 的期望值分别是 `/gps/odom`、`0.5`、`6.283`、`1.5`、`1.0`、`0.0`、`100.0`。如果以 `obstacle` 启动，最后一个值应为 `60.0`；如果传入了速度参数，`max_vel_x` 应等于该参数。如果不是，重新使用一键脚本启动：

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
