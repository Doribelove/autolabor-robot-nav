# Autolabor 室内 FAST-LIO Qt 操作台

ROS Noetic、Qt5 Widgets 与 `librviz` 构成的双机真车操作界面。当前版本采用
FAST-LIO 增量定位或固定三维地图定位，不接入 GNSS、WGS84 目标或
RabbitMQ。界面本身从不发布 `/cmd_vel`。

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
- 地图模式下 RViz 的 Fixed Frame 为 `map`；`2D Pose Estimate` 为 FAST-LIO
  固定地图匹配提供近似初值，`2D Nav Goal` 发布 `/move_base_simple/goal`。综合页在
  `/map` 到达后自动显示整张地图，并提供中文“显示整张地图/设置初始位姿”入口；
  初始位姿前视角不跟随尚未接入 `map` 的 `base_link`。若窗口初始化时漏接锁存地图，
  GUI 会自动重订阅，并在 MapDisplay 的宽、高和分辨率真实匹配后通过锁存话题
  `/autolabor_operator_gui/map_display_status` 报告 `READY`。
- 初始位姿工具退出并且 ICP 达到 `LOCALIZED` 后，综合页自动进入“③ 跟随车辆”视角；
  Fixed Frame 仍为 `map`，TopDownOrtho 的 Target Frame 改为 `base_link`，方便持续观察
  20 m × 20 m 局部代价地图和实时点云。“① 显示整张地图”可随时返回全图。
- 综合页“④ 显示静态三维先验”按需订阅
  `/fast_lio_localization/prior_map` 并切换为可旋转的 Orbit 视角；再次点击恢复二维全图。
  该显示默认关闭，因此无图模式和现有二维导航基线不变，也不会默认跨机拉取先验 PCD。
- `/fast_lio_localization/aligned_scan` 作为动态匹配点云显示；静态先验 PCD 与动态点云在
  名称上明确区分，避免把锁存先验误判成“局部点云停止更新”。
- 综合页和清扫页都以亮绿色二维矩形显示
  `/move_base/local_costmap/footprint`。该轮廓直接来自 move_base 当前 costmap：
  基础车体为 `1.04 × 0.70 m`，叠加 `0.10 m` footprint padding 后，实际显示并参与
  避障的安全外框约为 `1.24 × 0.90 m`；它随 `base_link` 位姿持续移动，不依赖 URDF。
  静态地图模式要在人工初始位姿以及新鲜里程计/TF 共同建立 `map -> base_link` 后才显示
  车框；无数据流启动时暂不显示属于正常状态。
- 综合页和清扫页默认以高对比前景层显示车辆周围 `20 m × 20 m` 的
  `/move_base/local_costmap/costmap`。重复的全局 costmap 叠层默认关闭，避免遮住实时
  障碍与膨胀区；需要调试时仍可在“显示 RViz 调试面板”中重新启用。
- 清扫页默认使用 `1.00 m` 有效宽度和 `15%` 重叠（车道中心距 `0.85 m`），显示
  覆盖导航状态、路线约束、VCU/TEB 运动学核对、完整障碍感知状态及去重后的覆盖面积
  估算；“底盘执行门”会显示 VCU 急停或 TCU/左右 ECU 故障并禁用启动/恢复。地图保留
  最近 `120` 个 `/Odometry` 位姿，白色侧栏同时显示当前全局/里程计
  位姿和最近 `10 s` 的样本数、累计距离与数据年龄。`SWEEPING`
  只显示为“覆盖路线执行中”；V1 未接入主刷、边刷、风机或喷淋状态，界面不会据此
  推断实体清扫机构已经工作。
- 两个地图页都显示路线图例：青色覆盖条带、蓝色全局参考路线、红色当前 TEB 局部轨迹、
  绿色覆盖执行记录。蓝/红路线只在活动目标期间启用，任务结束后清空缓存；不会再显示
  可能穿过障碍物的直线“条带连接预览”。
- 清扫页在轨迹生成、已就绪、启动准备和活动执行阶段均提供对应的取消入口。取消或任一
  任务终态会清除区域、计划/实走轨迹和本地计划 ID，随后可直接再次框定；规划请求使用
  UI 代际号，迟到的异步结果不会把已取消的草稿或计划重新写回界面。框定过程中切换页签
  再返回时会恢复 `PublishPoint` 工具。
- “开始录包”只记录相关话题；“录入静态地图/结束静态地图录入”独立生成 MID360
  三维图、双 LD19 二维图和高度切片融合图，保存在 `global_maps/map_sets/`。
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
# 或加载地图集：
./scripts/start_dual_host.sh --start --map-set global_maps/map_sets/latest
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

## 静态地图定位坐标系

静态导航使用以下唯一 TF 所有权关系：

```text
map --multiscale ICP--> camera_init --FAST-LIO--> body --static--> base_link
```

全局和局部 costmap 都使用 `map`。map_server 只加载二维静态图，不参与定位；
FAST-LIO 保持原始高频里程计，独立定位器加载 `map_3d/map.pcd`，等待
`2D Pose Estimate` 后以低频多尺度 ICP 校正 `map -> camera_init`。状态未达到
`LOCALIZED` 时速度门控持续输出零速度。

二维占据图的观测严格来自 `/dual_lidar/scan`，FAST-LIO `/Odometry` 只负责放置
扫描；MID360 水平切片只在停止建图后加入最终融合图。
