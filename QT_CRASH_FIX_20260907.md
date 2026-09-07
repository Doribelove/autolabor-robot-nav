# Qt/RViz 与 ZED 闪退修复（2026-09-07）

本轮仅修改导航候选工作树；原工作区源码未改。实车主链通过统一入口停机，
修改和测试期间主运动与 FOD 门保持 false，未发送真实初值、导航目标或非零速度。

## 代码级证据

此前日志有两类退出：9 月 6 日 Qt 自身 pure-virtual/SIGABRT、Ogre
PassGroupRenderableMap 断言和 SIGSEGV；9 月 7 日是 ZED 跟踪发散后 SIGSEGV，
由现有 required/wait-n 生命周期级联关闭 Qt。单纯重启后 ZED 又发散四次。

本轮进一步找到了可重现的 Qt 并发缺陷：应用 `AsyncSpinner(2)` 消费全局回调队列；
RViz 1.14.26 的 `VisualizationManager::getUpdateQueue()` 也返回同一全局队列，
并依赖 GUI 线程的 `onUpdate()` 调用 `ros::spinOnce()`。`MapDisplay::incomingMap`
直接覆盖地图缓冲，其他 Display 回调也依赖 GUI 线程语义。因此应用 spinner 会在
RViz 渲染或销毁期间抢走这些回调。原版本的实际 GUI 回归测试 3/3 失败，明确记录
“Application spinner stole RViz's GUI callback”和后台线程执行证据。

参考 RViz 上游同版本实现：
[visualization_manager.cpp](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/visualization_manager.cpp)、
[display.cpp](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/display.cpp)、
[map_display.cpp](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/default_plugin/map_display.cpp)。

第一版私有队列修复消除了该线程错误，但隔离的真实 X11/Ogre 压力测试又在
销毁后第二次创建 VisualizationFrame 时重现 SelectionTexture2 纹理创建失败及
SIGSEGV。这证明仅停止渲染计时器后删除重建仍不够。最终方案保留原生渲染窗口、
VisualizationManager 和现有显示对象，ROS 重连只重置显示缓存和恢复本进程的
topic 注册/连接，不再在同一进程中重建 Ogre 上下文。

## 修改范围

- GUI 业务 NodeHandle、TF、BPU 等订阅绑定私有 `telemetry_queue_`，双线程只消费
  此队列；RViz 全局更新队列由 GUI 线程独占处理。
- RViz 初始化增加重入保护；关闭时先停止业务回调，再在 C++ 成员与 ROS 尚存活时
  停止渲染并销毁 RViz，避免 QObject 子对象延迟析构访问已销毁的窗口成员。
- ROS master 恢复保留同一原生窗口，使用本节点 slave API 枚举其真实 topic/type，
  重新注册发布/订阅，并重建输入连接以接收锁存地图和静态 TF。不发布任何目标消息。
- 增加 Qt 事件/页签/初始化日志，Release 可执行文件保留调试符号及帧指针。
  未改系统 core_pattern、ToDesk 或显卡驱动。
- `robot_bringup/zed2_camera.launch` 默认关闭 `pos_tracking_enabled`，并将
  `depth_stabilization` 置 0；后者也会隐式开启位置跟踪，必须同时关闭。
  运行态已确认 ZED odom/pose 没有订阅者。保留 QUALITY 深度、RGB、内参与 TF 禁发配置。
- 保留 MapDisplay 自动重订阅和真实宽、高、分辨率 READY 判定；不改变运动安全门
  或原有故障级联停机策略。

## 验证记录

- 停机：`validation/qt_crash_fix_stop_20260907.log`，返回 0、双机归属与残留校验通过。
- 修复前回归：`validation/qt_crash_fix_baseline_test_20260907.log`，3 项全部失败。
- 第一版真实渲染压力：`validation/qt_crash_fix_rviz_stress_20260907.log`，在删除后重建
  RViz 时复现 SIGSEGV；这是隔离本地 ROS master 的合成测试，不是实车移动。
- Qt 静态合约 52 项、ZED 合约 5 项通过；实际 launch 展开结果保存在
  `validation/qt_crash_fix_zed_launch_params_20260907.yaml`。
- 本机全量 Release 构建及最终 GUI/测试目标重新编译通过，二进制含 `.debug_info`
  和 `.debug_line`。最终 Qt SHA-256 为
  `4e51403a4444e37b5b01280a8fd0226f52c920ede9b7e2ea2bf11aa24678e018`。
- 最终 5 项回调/重连测试全部通过，包含真实业务订阅在 GUI 不泵事件时继续接收，以及
  主动从隔离 master 删除本节点端点注册后完整恢复：
  `validation/qt_crash_fix_final_callbacks_20260907.log`。
- 真实 X11/Ogre 压力测试通过：1000 次实际页签切换（日志计数核对）、100 次代价图
  显隐操作、50 次二维/三维视图操作、5 次保留原窗口的重连、1000 次更新队列回调
  均在 GUI 线程。每次重连后 MapDisplay 宽/高/分辨率重新满足 READY，测试进程正常退出。
  证据 `validation/qt_crash_fix_verified_stress_20260907.log`。
- 压力测试中间版本 `qt_crash_fix_final_stress_20260907.log` 未崩溃，但测试只调用一次
  READY 探针（该调用只启动自动重订阅）而失败；最终测试显式等待异步握手完成，
  不放松 READY 条件。保留该失败日志供审计。
- `catkin_test_results build/test_results` 当前汇总 283 项、0 错误、0 失败、0 跳过。
  数量是 catkin 汇总口径，不代表实车运动场景。
- 最终静态检查通过：`validation/qt_crash_fix_final_static_20260907.log`。
- J6M 原生部署成功，独立 `current` 指向
  `/opt/autolabor/dual_host/releases/20260907_191248/install`，板端静态检查通过。
  本次共享 `robot_bringup` 相机 launch 两端 SHA-256 均为
  `3fa45e0142d0c43b7ab4c34a510bb90bed36b649fee1473852ef4825371fb17e`。
  Qt 是 NVIDIA 本地产物，未把本机 GUI ELF 复制到 J6M。
  发布记录：`validation/qt_crash_fix_deploy_20260907.log`、
  `validation/qt_crash_fix_j6m_health_20260907.log`。

## 本次完整程序交付状态

统一入口带原 `map_20260829_221335_gimp_cleaned` 冷启动返回 0；独立严格运行态检查
返回 0。候选服务 active/MainPID=2161491，Qt PID=2169173，ZED PID=2167341。
`/proc/2169173/exe` 的 SHA-256 与上述新 Qt 二进制完全一致，证明运行的是修复版。
日志目录 `log/nvidia_ui_20260907_191428`。

- Qt 地图 `READY;width=1311;height=670;resolution=0.100000`，截图
  `validation/qt_crash_fix_running_20260907.png` 已确认实际地图和 BPU 框图显示。
- ZED 5000M USB 3.x；实际参数 `pos_tracking_enabled=false`、
  `depth_stabilization=0`，SDK 输出 `Positional tracking -> DISABLED`，连续诊断
  `Pos. Tracking status=INACTIVE`。没有出现 `Starting Positional Tracking`、
  位置跟踪发散或进程异常退出。真实 RGB/深度、正式检测、FCOS RGB-D、CenterPoint
  感知与 Qt 都在线；CenterPoint 仍仅候选观察，不接导航控制。
- 19:16:13 起 60.09 秒只读观测：2907 条最终 `/cmd_vel`、1199 条上游安全指令、
  左轮 2443 条与右轮 2358 条反馈全部为零，`faults=[]`，覆盖 inactive。
  证据 `validation/qt_crash_fix_stationary_20260907.json`。
- 本次新会话仍 `WAITING_INITIAL_POSE`；需要操作员按真实位置设置初值，未复用
  旧初值或旧目标。主运动与 FOD 门均保持 false，候选授权标记仍不存在。
- Qt 起初提示 TEB 参数服务等待超时：静态地图尚未设置初值时 move_base 的局部规划器
  未完成初始化，不能把此提示写成 Qt 崩溃。实体麦克风仍未枚举，语音采集不可用。

启动、严格检查和诊断证据：`validation/qt_crash_fix_start_20260907.log`、
`validation/qt_crash_fix_runtime_20260907.log`、
`validation/qt_crash_fix_live_params_20260907.log`、
`validation/qt_crash_fix_diagnostics_20260907.log`。

本轮验证证明已修复可复现的回调线程错误和重建渲染窗口故障，并关闭了已观察到的
ZED 跟踪发散入口。未做 ToDesk 服务重启/物理显示热插拔、小时级老化或实车移动验收，
不能把短时通过写成任何场景下绝不再崩溃。

## 回归入口

普通离屏测试由 `catkin_make run_tests_autolabor_operator_gui` 执行。
真实渲染压力需本机可用 X11/OpenGL，在候选隔离入口中加载 setup 后执行：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11486 ROS_IP=127.0.0.1
rostest autolabor_operator_gui rviz_render_stress.test
```

rostest 创建独立本地 master，仅测试进程发布合成地图、路径和显示用 TF；不连接 J6M。
