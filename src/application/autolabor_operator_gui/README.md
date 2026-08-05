# Autolabor 一体化 Qt 操作台

ROS Noetic、Qt5 Widgets 与 `librviz` 构成的真车操作界面。界面本身不发布
`/cmd_vel`。GPS 模式的底盘控制经过 GPS/FOD 安全仲裁链，FAST_LIO 模式沿用
现有 `/move_base -> /cmd_vel -> /m2_driver` 控制链。

## 已集成功能

- 综合页同时显示内嵌 RViz、YOLO11 标注画面、GPS、双天线航向、局部坐标、
  导航状态和 GPS/视觉控制模式。
- 内嵌 RViz 工具栏包含 `2D Nav Goal`，固定发布到
  `/move_base_simple/goal`；Fixed Frame 为 `camera_init`。
- 可直接输入 WGS84 纬度、经度并发布到 `/gps/goal_fix`，也可填入当前位置或
  发送车头正前方 8 m 测试目标。
- 双天线 `/gps/heading` 在顶栏、综合页和 GPS 页实时显示；数值约定为
  `0°=北、90°=东`，从正北顺时针增加。
- 订阅 `/fod_camera/image_raw`、`/fod/debug/image` 和 `/fod/detections`，
  显示标注画面、类别、置信度、检测框、模型、推理耗时与消息频率。
- 调用 ZED 动态参数服务读取/设置联动自动或手工曝光、增益；可启停路面 ROI
  图像质量控制器。ZED 的曝光与增益单位均为 `0..100%`。
- “立即单独启动”只调用 `/fod_navigation_mode/set_fod_enabled`。仲裁器先
  屏蔽 GPS 输出、暂停并保留最终 GPS 路线、取消当前子目标、确认车辆停车，
  然后才使能视觉行驶。视觉完成后自动恢复 GPS；`ABORT`/故障时保持停车。
- RabbitMQ 缓存目标确认/清空、取消导航、GPS 静态误差重置及 mode1 录包。
- ROS master 或任一可选模块缺失时，窗口仍可打开并显示离线状态。

## 编译

```bash
cd /home/slam/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=''
source devel/setup.bash
```

显式清空 `CATKIN_WHITELIST_PACKAGES` 可避免 CMake 缓存停留在一次包级构建。

## 推荐：一条命令启动整套系统

GPS 定位模式：

```bash
cd /home/slam/robot_ws
./scripts/operator_all_in_one.sh 0.3 cruise
```

FAST_LIO 定位模式：

```bash
cd /home/slam/robot_ws
./scripts/operator_fast_lio_all_in_one.sh 0.3
```

FAST_LIO 入口的可选参数是 TEB 前进与倒车最大速度，默认 `0.3 m/s`。该入口
订阅 `/Odometry`，内嵌 RViz 显示 `/Odometry` 轨迹，并在定位就绪后把固定
坐标系从启动阶段的 `base_link` 切换到 `camera_init`。GPS 入口仍订阅
`/gps/odom`，命令和行为保持兼容。

两个入口都会先建立共享 ROS master，再并行启动导航、相机与 YOLO11、RabbitMQ
桥接和 Qt，并强制 `NAV_START_RVIZ=false`，因此不会出现第二个独立 RViz
窗口。Qt 会在导航初始化期间先显示出来；正常情况分别等待
`Robot bringup is running in gps mode.` 或
`Robot bringup is running in fast_lio mode.`。如果 CAN、GNSS 或其他导航
节点提前失败，脚本会以“降级操作台”方式保留 Qt；离线项显示故障，运动入口
仍受安全就绪门控。YOLO Python 环境缺失时也只跳过视觉侧。关闭 Qt 或在启动
终端按 `Ctrl+C` 会安全停止该入口启动的整套进程；日志分别保存在
`log/operator_all_in_one_时间/` 和
`log/operator_fast_lio_all_in_one_时间/`。

可选开关：

```bash
# 不启动 RabbitMQ
OPERATOR_START_RABBITMQ=false ./scripts/operator_all_in_one.sh 0.3 cruise
OPERATOR_START_RABBITMQ=false ./scripts/operator_fast_lio_all_in_one.sh 0.3

# 不自动启动相机/YOLO（视觉页仍可连接外部已启动节点）
OPERATOR_START_VISION=false ./scripts/operator_all_in_one.sh 0.3 cruise

# 相机已由别处启动，只启动 YOLO 与图像质量侧
OPERATOR_START_CAMERA=false ./scripts/operator_all_in_one.sh 0.3 cruise
```

默认保留 ZED 原生自动曝光/增益，不自动启动 ROI 图像质量控制。需要试验自定义
路面 ROI 控制时显式启用（曝光上限为 ZED 百分比，不是微秒）：

```bash
OPERATOR_IMAGE_QUALITY_CONTROL=true \
OPERATOR_IMAGE_QUALITY_EXPOSURE_MAX_PERCENT=100 \
./scripts/operator_all_in_one.sh 0.3 cruise
```

## 保留的分终端启动方式

```bash
# 终端 1（GPS 定位）
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise

# 或终端 1（FAST_LIO 定位）
NAV_START_RVIZ=false \
FAST_LIO_NAV_MAX_VEL_X=0.3 \
FAST_LIO_NAV_MAX_VEL_X_BACKWARDS=0.3 \
./scripts/bringup.sh fast_lio

# 终端 2
./scripts/operator_gui.sh odom_topic:=/gps/odom

# FAST_LIO 时终端 2 改为
./scripts/operator_gui.sh odom_topic:=/Odometry start_gps_error_monitor:=false

# 可选终端 3
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

相机和 YOLO 也可继续独立启动：

```bash
roslaunch autolabor_fod_vision zed_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_percent:=100
```

无 OpenGL 的 CI/远程环境可禁用 RViz：

```bash
./scripts/operator_gui.sh enable_rviz:=false
```

若静态误差监视节点已由别处启动：

```bash
./scripts/operator_gui.sh start_gps_error_monitor:=false
```

## 操作约束

- GPS 目标按钮要求 ROS、GPS 原点、2 秒内的当前定位里程计（GPS 模式为
  `/gps/odom`，FAST_LIO 模式为 `/Odometry`）与
  `/move_base/status`、`/gps/goal_fix` 订阅者均就绪；视觉模式中 GPS 处于
  休眠，界面会禁用这些入口。
- `2D Nav Goal` 属于局部地图人工目标，不是 WGS84 目标；GPS 经纬度必须使用
  综合页输入框。视觉模式期间出现的 move_base 目标会由安全仲裁器取消。
- “立即单独启动”只有在 `GPS_ACTIVE`、相机/检测消息不超过 1.5 秒且当前至少
  有一个检测目标时才可点击。第一次及调参试验必须在封闭净空区域进行，操作员
  全程手持物理急停。该 GPS/FOD 切换服务不在 FAST_LIO 启动链中，因此
  FAST_LIO 模式下此按钮保持禁用。
- 不要从 Qt 外直接调用 `/fod_visual_servo/set_enabled`；联动运行只使用
  `/fod_navigation_mode/set_fod_enabled`。
- 图像质量控制器运行时会持续调整曝光/增益，可能覆盖手工相机参数。需要固定
  手工值时，先点击“停用并恢复相机自动”，再应用曝光/增益。

RabbitMQ 按钮调用：

- `/rabbitmq_bridge/publish_latest`（`std_srvs/Trigger`）
- `/rabbitmq_bridge/clear_latest`（`std_srvs/Trigger`）
