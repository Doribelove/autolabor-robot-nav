# Autolabor 一体化 Qt 操作台

ROS Noetic、Qt5 Widgets 与 `librviz` 构成的真车操作界面。界面本身不发布
`/cmd_vel`，底盘控制仍经过现有 GPS/FOD 安全仲裁链。

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
- 调用海康相机运行时服务读取/设置连续自动或手工曝光、增益；可启停路面 ROI
  图像质量控制器。
- “启动相机控制回收”调用 `/fod_navigation_mode/set_fod_enabled`。GPS 模式下先
  进入等待，显示最终目标距离、清扫请求和有效 FOD 状态。距最终目标小于 5 米
  即启动清扫，无需检测或相机在线；没有有效 FOD 时按 GPS 回收，有有效 FOD
  才暂停保留的 GPS 路线、确认停车、切入标称 0.6 m/s 视觉回收。
  视觉完成后恢复 GPS；`ABORT`/故障时保持停车。
- RabbitMQ 缓存目标确认/清空、取消导航、GPS 静态误差重置及 mode1 录包。
- ROS master 或任一可选模块缺失时，窗口仍可打开并显示离线状态。
- “清扫”页通过异步进程调用 `scripts/sweep_test.py --json`，支持设备地址、端口、
  启动、停止、状态查询和通信日志。顶栏显示清扫状态；进入页面每 5 秒查询，
  请求不并发，忙碌时的停止请求排队执行，与相机自动回收共用设备操作锁。
  回收自动开启的清扫器在 GPS 到达、视觉完成或退出时自动关闭；手动开启的仍需手动停止。
- AGX 的 `fast_lio_indoor` 模式也提供视觉回收待命，无需 GNSS。退出/完成后返回
  导航待命，局部目标需重发；GNSS 模式保留原路线恢复。界面将内部 `GPS_ACTIVE`
  显示为“路径导航”，状态 topic 契约不变。
- ZED 已适配实际图像尺寸及光学坐标系，推理和回收共同使用
  `config/fod_zed_ground_roi.yaml` 的中间地面区域。海康曝光/增益控件仍仅供海康使用。

## 编译

```bash
cd /home/robot/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=''
source devel/setup.bash
```

显式清空 `CATKIN_WHITELIST_PACKAGES` 可避免 CMake 缓存停留在一次包级构建。

## 推荐：一条命令启动整套系统

```bash
cd /home/robot/robot_ws
./scripts/operator_all_in_one.sh 0.3 cruise
```

脚本先创建或复用 ROS master，再后台启动 GPS 导航，并立即启动相机与
YOLO11、RabbitMQ 桥接和 Qt，导航初始化与界面启动并行。它强制
`NAV_START_RVIZ=false`，因此不会再出现独立 RViz 窗口。后台等待
`Robot bringup is running in gps mode.`；Qt 窗口打开不等于导航已经就绪。
如果 CAN、GNSS 或其他导航节点在此之前失败，Qt 以“降级操作台”方式继续
运行；离线项显示故障，导航与视觉行驶按钮保持原有就绪控制。
YOLO Python 环境缺失时也只跳过视觉侧，不阻止 Qt。关闭 Qt 或在启动终端按
`Ctrl+C` 会安全停止这条命令启动的整套进程；日志保存在
`log/operator_all_in_one_时间/`。

可选开关：

```bash
# 不启动 RabbitMQ
OPERATOR_START_RABBITMQ=false ./scripts/operator_all_in_one.sh 0.3 cruise

# 不自动启动相机/YOLO（视觉页仍可连接外部已启动节点）
OPERATOR_START_VISION=false ./scripts/operator_all_in_one.sh 0.3 cruise

# 相机已由别处启动，只启动 YOLO 与图像质量侧
OPERATOR_START_CAMERA=false ./scripts/operator_all_in_one.sh 0.3 cruise
```

默认启用图像质量控制，实车当前入口使用曝光上限 `12000 μs`。可覆盖：

```bash
OPERATOR_IMAGE_QUALITY_CONTROL=false \
./scripts/operator_all_in_one.sh 0.3 cruise
```

## 保留的分终端启动方式

```bash
# 终端 1
NAV_START_RVIZ=false ./scripts/bringup.sh gps 0.3 cruise

# 终端 2
./scripts/operator_gui.sh

# 可选终端 3
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./scripts/rabbitmq_gps_goal_bridge.py
```

相机和 YOLO 也可继续独立启动：

```bash
roslaunch autolabor_fod_vision hikrobot_fod_detection.launch \
  start_camera:=true \
  enable_image_quality_controller:=true \
  image_quality_exposure_max_us:=12000
```

无 OpenGL 的 CI/远程环境可禁用 RViz：

```bash
./scripts/operator_gui.sh enable_rviz:=false
```

若静态误差监视节点已由别处启动：

```bash
./scripts/operator_gui.sh start_gps_error_monitor:=false
```

## ZED 2 原生相机设置

2026-09-29 增加“视觉 → ZED 原生参数”页签，直接使用
`/zed2/zed_node/parameter_updates` 与 `/zed2/zed_node/set_parameters`。
完整入口按 `OPERATOR_CAMERA_DRIVER` 自动选中相应页签。独立 Qt 可传入
`camera_driver:=zed` 或 `camera_driver:=hikrobot`。

先“读取 ZED 参数”，再调整自动曝光/增益、手动档位、白平衡、亮度、对比度、
饱和度、锐度、伽马及深度过滤参数，最后点击“应用 ZED 参数”。ZED 2 曝光和
增益的手动值均为 0～100；色温 28～65 对应 2800～6500 K。自动模式下所显示的
手动档位不是相机实时自动曝光读数。“恢复自动曝光与白平衡”只设置自动开关。
操作在后台执行；参数服务不可用时显示原因，不影响导航入口。

更完整的成像、GPU 与 TensorRT 配置见工作区 [ZED_PERFORMANCE.md](../../../ZED_PERFORMANCE.md)。

## 操作约束

- GPS 目标按钮要求 ROS、GPS 原点、2 秒内的 `/gps/odom` 与
  `/move_base/status`、`/gps/goal_fix` 订阅者均就绪；视觉模式中 GPS 处于
  休眠，界面会禁用这些入口。
- `2D Nav Goal` 属于局部地图人工目标，不是 WGS84 目标；GPS 经纬度必须使用
  综合页输入框。视觉模式期间出现的 move_base 目标会由安全仲裁器取消。
- GPS 模式可在远处或暂无检测时提前点击“启动相机控制回收”。进入等待后，
  “取消相机回收 / 恢复导航”取消本次请求并关闭本次自动开启的清扫器，不打断
  GPS 导航。每次开启只执行一次回收；无 FOD 时按 GPS 到达完成回收。
  原六类中置信度至少 0.30 的新鲜检测才算有效目标，低分预览框不会触发。
  室内模式仍要求相机/检测就绪且至少有一个目标，然后直接切换，不使用 5 米条件。
- 不要从 Qt 外直接调用 `/fod_visual_servo/set_enabled`；联动运行只使用
  `/fod_navigation_mode/set_fod_enabled`。
- 图像质量控制器运行时会持续调整曝光/增益，可能覆盖手工相机参数。需要固定
  手工值时，先点击“停用并恢复相机自动”，再应用曝光/增益。

RabbitMQ 按钮调用：

- `/rabbitmq_bridge/publish_latest`（`std_srvs/Trigger`）
- `/rabbitmq_bridge/clear_latest`（`std_srvs/Trigger`）
