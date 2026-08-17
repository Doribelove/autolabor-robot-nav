# Autolabor 室内 FAST-LIO Qt 操作台

ROS Noetic、Qt5 Widgets 与 `librviz` 构成的双机真车操作界面。当前版本只面向
FAST-LIO 局部坐标导航，不接入 GNSS、WGS84 目标或 RabbitMQ。界面本身从不发布
`/cmd_vel`。

## 已集成功能

- 顶栏显示 ROS、CAN、FAST-LIO、注册点云、IMU、避障雷达、move_base、控制模式、
  ZED、YOLO11 和录包状态。
- FAST-LIO 页给出 0–100 健康分、明确的异常依据以及以下原始指标：
  - `/Odometry` 数据年龄和频率；
  - `/cloud_registered_body` 数据年龄、频率和点数；
  - `/livox/imu` 数据年龄、频率和数值合法性；
  - `camera_init -> base_link` TF 连通性；
  - 位姿四元数、内部位置/yaw 协方差、近 2 秒单帧位姿跳变；
  - 车辆静止至少 5 秒后的窗口最大漂移。
- 综合页输入 `Δ前向 / Δ左向 / ΔYaw`。点击发送后，以当前车体姿态换算成里程计
  固定坐标系下的 `geometry_msgs/PoseStamped`，发布到 `/move_base_simple/goal`。
- 内嵌 RViz 的 `2D Nav Goal` 使用相同局部目标 topic，Fixed Frame 为
  `camera_init`。
- ZED/YOLO11 画面、检测结果、曝光/增益和图像质量控制完整保留。
- 视觉行驶只调用 `/fod_navigation_mode/set_fod_enabled`；局部路线暂停、停车确认
  和恢复都由安全仲裁器完成。
- ROS master 或任一可选模块缺失时，窗口仍可打开并显示具体离线原因。

## 健康度口径

本车实测基线为：里程计约 10 Hz、注册点云约 10 Hz、IMU 约 200 Hz。界面按以下
等级显示：

- `健康`：总分至少 85，三路关键数据新鲜、TF 连通且没有硬故障；
- `注意`：总分 65–84，常见原因是频率、协方差、连续性或静止漂移开始变差；
- `异常`：总分低于 65，或任一关键数据中断、IMU 出现非法数、位姿非法。

内部协方差表示估计器自己的不确定度，并不是外部真值误差。要验证绝对精度，仍需
已知测量点、闭环复位误差或全站仪等外部基准。

## 编译

```bash
cd /home/slam/robot_j6m_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=autolabor_operator_gui -j2
source devel/setup.bash
```

## 启动

完整双机项目推荐只使用工作区一键入口：

```bash
cd /home/slam/robot_j6m_ws
./scripts/start_dual_host.sh
```

仅调试 Qt：

```bash
source /opt/ros/noetic/setup.bash
source /home/slam/robot_j6m_ws/devel/setup.bash
roslaunch autolabor_operator_gui operator_gui.launch
```

无 OpenGL 的 CI/远程环境可以附加 `enable_rviz:=false`。

## 相对目标安全门

“发送相对目标”和测试页“正前方 2 m”按钮只有同时满足以下条件才启用：

1. ROS 接口在线；
2. FAST-LIO 显示 `健康`；
3. `/Odometry` 不超过 0.5 秒；
4. `/move_base/status` 新鲜；
5. `/move_base_simple/goal` 有订阅者；
6. 模式仲裁器允许局部目标。

这只是软件入口门控。实际运动还受双机配置中的 `MOTION_ENABLED`、授权标记、
NVIDIA 看门狗、CAN/M2 和实体急停约束。
