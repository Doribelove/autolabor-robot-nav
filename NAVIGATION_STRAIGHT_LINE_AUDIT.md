# GNSS 导航流程与直线行驶核查

日期：2026-09-13。范围：本项目的 GNSS / FAST_LIO 导航启动链、TEB 速度反馈与输出、GPS 速度转发、GPS/FOD 切换及 M2 底盘输入。

## 结论与证据边界

没有发现 FAST_LIO 直接向底盘发布零速度命令的实现。FAST_LIO 发布点云、位姿里程计和路径；本项目 GNSS 启动分支还明确关闭了它的里程计与 TF 发布，仅使用其点云。

前一轮已修复两处相关配置缺陷：FAST_LIO 导航把速度字段恒为零的 `/Odometry` 用作 TEB 速度反馈；基础 TEB 配置未启用同比例限幅，在不加载 GPS 场景配置时可能改变规划曲率。它们不能概括成“FAST_LIO 抢发停车命令”。本轮进一步修复了 GNSS 串口读取被发布限频阻塞的缺陷，并清理启动误判、重复检查及无效资源消耗，见下文。

用户此次故障的启动命令和对应录包尚未确定。本机现有导航录包来自 2026-07-03，早于当前 GPS 速度反馈、限幅和模式切换实现；它能证明那次运行的命令来源，不能证明当前 GNSS 主入口的 S 形行驶已由本次配置修改解决。

## 哪些组件会下发零速度

| 组件 | 代码中的实际行为 | 与当前问题的关系 |
| --- | --- | --- |
| FAST_LIO | `/Odometry` 只填位姿，没有填写 `twist`；无底盘 `Twist` 命令发布者 | 零速度反馈与零速度控制命令是两个不同问题 |
| 点云过滤、点云转激光 | 提供点云或 `/scan`，不发布底盘速度 | 感知结果可影响避障规划，但停车命令由后续控制链发出 |
| TEB / move_base | 轨迹不可行、优化失败、到达目标等情况下返回或发布零速度 | 旧录包中实际确认的中途零速度来源 |
| `gps_goal_speed_limiter` | 转发规划速度；在取消、终止、到达、近终点锁存、缺少有效位姿或命令过期等既有条件下输出零速度 | 需要比较 `/cmd_vel_navigation` 与 `/cmd_vel_gps`，才能确认当次是否由此改变命令 |
| `fod_navigation_mode` | 正常 GPS 模式只选择 GPS 命令；转换、故障或所选命令过期时输出零速度 | 待机视觉控制器在 `/cmd_vel_fod` 上的零速度不会自动覆盖 GPS；图检查故障会锁存停车 |
| M2 底盘驱动 | 接收 `/cmd_vel`，也接收独立的 `/ackerman_vel`；将速度和角速度换算为 CAN 指令 | 若有人额外启动手动控制程序，应同时核对两个输入，不能只检查 `/cmd_vel` |

关键实现位置：

- [FAST_LIO 发布实现](src/localization_fastlio/FAST_LIO/src/laserMapping.cpp)：`publish_odometry()` 与发布者初始化。
- [导航启动脚本](scripts/bringup.sh)：GNSS 分支使用 `odom_pub_en:=false tf_pub_en:=false`。
- [TEB 实现](src/navigation_arena/forks/navigation/local_planner/teb/src/teb_local_planner_ros.cpp)：`computeVelocityCommands()` 从 TF 取位姿，从 `odom_helper_` 取速度；`saturateVelocity()` 决定限幅是否保持曲率。
- [GPS 速度转发](src/scripts/robot_bringup/scripts/gps_goal_speed_limiter.py)：`timer_callback()`、目标与取消回调。
- [GPS/FOD 命令选择](src/application/autolabor_fod_control/src/autolabor_fod_control/mode_manager.py)：`CommandArbiter.sample()`。
- [M2 驱动](src/autolabor_core/autolabor_canbus_driver/autolabor_canbus_driver/src/m2_driver.cpp)：`handle_twist_msg()`、`handle_ackerman_msg()`、`send_odom()`。

项目原启动流程使用 `drive_only.launch`，不会启动键盘控制节点。仓库中的旧键盘控制程序在未按方向键时可以持续发布零速度；这是额外并行启动手动程序时需要区分的来源，不是本次已经确认的故障。

## 2026-07-03 录包证据

分析文件：`mode1_nav_all_20260703_180721_0.bag`，总时长约 224.76 秒。仅离线读取，未回放到底盘，也未修改录包。

| 指标 | 结果 |
| --- | --- |
| `/cmd_vel` 连接记录中的发布者 | 仅 `/move_base` |
| `/cmd_vel` 消息总数 | 1898 |
| 全程零 `Twist` 消息数 | 8 |
| 第一条至最后一条移动命令之间的零 `Twist` 消息数 | 7 |
| TEB 轨迹不可行警告数 | 7 |
| 每条中途零速度与最近一次不可行警告的时间差 | 均小于 0.6 毫秒 |
| 移动命令角速度正负切换次数 | 30，统计条件为 `abs(v) > 0.05 m/s`、`abs(omega) > 0.05 rad/s` |
| 1890 条移动命令中，请求转弯半径小于 1.2 米的数量 | 1850，半径按 `abs(v / omega)` 计算 |
| 最小请求转弯半径 | 约 0.2254 米 |
| `/gps/odom` 角速度 | 2246 条消息全部为零 |
| 同期底盘 `/odom` 最大角速度绝对值 | 约 0.135 rad/s |

这次记录直接支持“TEB 因轨迹不可行产生停车命令”，没有 FAST_LIO 与规划器竞争底盘速度的证据。大量交替角速度、不可执行的小转弯半径，以及缺失的 GPS 角速度反馈，说明还必须检查反馈和曲率；仅删除零速度不能解决这些控制误差。

上述 GPS 历史问题已有后续实现：当前 `/gps/odom` 默认读取新鲜底盘有符号速度，`gps_cruise.yaml`、`gps_obstacle.yaml` 均已启用同比例限幅，M2 驱动也已有相应换算修复。不能据旧录包断言这些问题仍存在于当前正常 GNSS 启动流程。

## 前一轮配置修复

1. 将 FAST_LIO 导航的 TEB 速度反馈默认值由 `/Odometry` 改为底盘 `/odom`。同时覆盖 `robot_bringup/navigation_arena.launch`、`arena_bringup/real_nav_nomap.launch`、直接 TEB launch 和 dingo 基础配置，避免不同入口仍使用错误默认值。
2. 在 dingo 基础 TEB 配置中设置 `use_proportional_saturation: True`。这样 FAST_LIO 和直接使用基础配置的导航也会在限幅时同比例缩放线速度与角速度，保留 `omega / v`。

示例：规划输出 `v=0.53 m/s`、`omega=0.243 rad/s`，若只把线速度限制到 `0.30 m/s`，转弯半径会从约 2.18 米缩小到 1.23 米；同比例限制后角速度约为 `0.138 rad/s`，半径保持约 2.18 米。

FAST_LIO 仍通过原 TF 提供定位，GPS 目标转换仍使用 FAST_LIO 的 `/Odometry` 位姿。此次修改没有把轮速积分位置改作 FAST_LIO 定位源。GNSS 主入口仍使用 `/gps/odom`，原 `cruise`、`obstacle` 配置及启动参数继续适用。

前一轮修改仅涉及 launch / YAML，无需为这些配置重新编译 C++。两轮均未新增节点、门控、看门狗或速度转发层，未删除既有停车条件，也未调整导航标定或场景调参项。现有导航进程在正常下次重启时读取修改。

## 前一轮配置验证

- 现有 `test_teb_profile_tuning.py` 的 26 项测试通过。
- 通过 ROS launch 加载器离线解析 7 种配置：FAST_LIO 主入口、兼容参数、直接 Arena 入口、直接 dingo TEB 入口、GPS cruise、GPS obstacle、自定义速度 topic。速度反馈源、同比例限幅、角速度消息语义和原 TF 坐标系均符合预期。
- 对现有 `CommandArbiter` 做离线检查：GPS 被选中时，连续 100 个待机 FOD 零命令不会替换 GPS 命令。
- 配置差异检查通过。未启动整车，未执行实车运动，也未把本次配置修复标记为实车验收通过。

## 本轮 GNSS 流程清理

| 已确认的问题 | 处理结果 |
| --- | --- |
| GNSS 每解析一条 GGA/RMC 就执行 `Rate(10).sleep()`；两类报文各 10 Hz 时消费能力不足，还会拖延同一串口的航向 | 串口持续读取，只限制位置输出频率；保留每条 RMC 的速度/航迹更新及原有双天线航向规则；坏的数值报文不再直接结束读串口循环 |
| 把 `/gps/goal_fix` 的所有订阅者都当作竞争控制器，录包、实验菜单也会触发启动失败 | 只计数本项目的两个目标转换器；允许只读订阅者，保留速度话题的唯一发布者要求 |
| 同一 TF 重复检查、GPS 话题重复等待，以及每轮逐话题重复查询 ROS 图 | 合并路由检查实现，移除重复等待；每轮只查询一次已发布话题列表 |
| 无地图导航启动旧 `viz_path`，但其固定查询 `map -> base_link`，与当前 `camera_init` 坐标系不匹配 | 移除该模式的无效启动项；保留 RViz、机器人模型和规划路径显示 |
| FAST_LIO 无消费者时仍转换并序列化整份世界坐标点云，还有未使用的 50 万点缓冲 | 按世界点云订阅情况处理，移除无用缓冲；导航使用的 body 点云、去畸变、建图和按需 PCD 保存不变 |
| PCL 的间接依赖被解析到海康 MVS 的旧 libusb，缺少 `libusb_set_option`，导致 FAST_LIO 重新链接失败 | 显式链接 PCL 已找到的系统 libusb；保留原启动脚本的进程级库隔离，不修改系统或相机环境 |

GNSS 默认仍按 10 Hz 输出位置；调度容纳 1% 周期抖动，不增加线程、定时器或运行许可条件。此次消除的是程序自身造成的串口积压，仍使用接收时刻作为 ROS 时间戳，不等于已经测量或消除了接收机自身延迟。实车报文频率和当次 S 形行驶的完整因果关系尚无对应记录确认。

本轮验证结果：

- 70 项相关测试通过：GNSS 定位/串口 37 项、导航配置/路由 30 项、话题等待 3 项。新增串口测试模拟 GGA、RMC、航向各 10 Hz，全部消费且无读取等待造成的积压；覆盖报文抖动、坏报文与缺航向情况。
- 离线解析 GPS cruise、GPS obstacle、FAST_LIO 导航及 GNSS 点云/激光配置，共 5 项；原速度话题、定位 TF 选择、避障输入和 RViz 开关保持一致。
- `bringup.sh` 语法及差异检查通过；`fastlio_mapping` 已增量编译成功，按原启动环境检查动态库无缺失或未解析符号。
- 未启动底盘或整车导航，未测量实车 CPU 降幅，也未宣称 S 形行驶已经修复或通过路测。

## 锁定当次 GNSS 问题还需要的信息

需要当次实际启动命令、速度及场景参数，以及该次运行的速度与定位记录。现有录包脚本已覆盖 `/cmd_vel_navigation`、`/cmd_vel_gps`、`/cmd_vel`、`/gps/odom`、底盘 `/odom`、航向、前轮角度、模式状态和 `/rosout`，可直接用于分辨哪个阶段首先出现偏差。

若 `/cmd_vel_navigation` 首先为零，结合同时刻 TEB、感知和 costmap 信息核查规划失败；若规划命令仍正常而后级变零，再核查已有转发或模式切换条件。若速度命令连续但角速度反复变号，应对比实际航向、轮角和规划曲率。后续调查围绕当次证据开展，不自行增加新的导航运行条件。
