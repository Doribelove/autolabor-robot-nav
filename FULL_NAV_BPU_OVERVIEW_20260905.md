# 综合页显示 J6M BPU：接续记录（2026-09-05）

22:42 更新：用户明确要求一直运行；旧会话确因 `time_limit` 到期结束。
已新增两端持续模式，实测超过 696 秒/340 帧仍在更新。当前入口不加 `--seconds 600`，详见
[持续预览修复记录](CONTINUOUS_BPU_20260905.md)。下文保留先前限时显示试验，不是新的时限。

## 本轮范围与状态

用户明确要求启动完整导航、保持完全静止，随后要求综合页显示 BPU 图。
仅改独立候选 `/home/slam/robot_j6m_ws_optimized_20260905` 的 Qt；没有修改原版。
22:08：统一冷启动与运行态检查通过，候选完整导航及 BPU 伴随桥正在运行，原版 inactive。
服务 MainPID=4193151，Qt PID=7510，桥 PID=22328，J6M worker PID=1506192。
进程号只是本次记录，后续不得凭旧 PID 停进程。两个运动门仍为 false。
综合页截图已目视核实为 BPU 框图，结果连续递增；没有恢复历史位姿或发送导航目标。
全局定位状态为 `WAITING_INITIAL_POSE`，地图纹理 READY，尚不能执行导航运动。
当前桥接会话 `qt_20260905_220615_867705` 从 22:06:15 起最长十分钟，约 22:16:16 自动结束；
结束后 GUI 保留最后一帧并标记超时，导航与相机不因此停止。
不能将本记录中的构建/静止链路检查解释为实车导航运动验收。

本轮修改：

- 综合页右侧改为 `/fod/bpu_preview/image` 的 BPU 检测源帧及框，不再被相机原图覆盖。
- 保留上次结果并显示源帧年龄、过期/断流警告；不将旧框画在新的相机帧上。
- 综合页或 BPU 预览页可见时订阅；其他页/最小化暂停实验推理。
- 保留“BPU 大画面”和“相机与原视觉控制”入口；正式检测、材质分类和控制链未切换。
- 源码只有 `main_window.cpp`、`main_window.h` 和 Qt 契约测试；修改前备份在
  `experiments/j6m_bpu_20260905/qt_before_overview_20260905/`。

## 已完成验证

- 候选 Qt 单任务低优先级构建成功。
- 52 项 Qt 契约测试和 11 项已有区域库 C++ 测试通过；静态检查汇总 952 项全绿，
  但本轮只重跑 Qt 相关测试，其余为已有结果。
- 原项目 1287 个文件/配置/地图/软链接快照核验：`unchanged=true`。
- 21:44 完整导航初次启动后，30 秒观察 1479 条 `/cmd_vel` 与 600 条安全速度均为零，
  两轮实测轮速为零；MID360/IMU/FAST-LIO/LD19/融合避障/ZED RGB、深度均持续更新。
  这是 `validation/full_start_20260905_2144_observation.json` 的窗口，不是全程证明。
- 21:46 RGB/深度约 15 Hz，150 对相同时间戳；相机内参每帧两份、只有一个标定值版本，
  验证了此前相机内参别名修复。深度有效比例约 79%，未做深度精度实物标定。
- 21:50 BPU 与完整导航短测 15 秒无观察器故障，速度命令/轮速均为零；首轮 5 个 BPU 结果
  往返约 1.2 秒。不是长时资源竞争或运动测试。
- 综合页改动后，22:06:16 起 20.15 秒实测：987 条 `/cmd_vel`、401 条安全速度全零，
  左/右轮 825/796 次反馈全零，观察器 faults=[]；RGB 和深度均约 14.76 Hz，
  避障来源持续为 `mid360+dual_ld19`，402 条定位状态均为 WAITING_INITIAL_POSE。
- 22:07:26 时新会话已有 34 个连续 BPU 结果，往返均值 1231.46 ms、最大 1247.33 ms，
  发布时源帧年龄均值 1351.11 ms。Qt 是 BPU 图像唯一订阅者，实验桥是唯一发布者；
  `/cmd_vel` 唯一发布者仍为 NVIDIA watchdog，没有新运动控制源。

实物显示证据：
[综合页截图](validation/overview_bpu_running_20260905.png)，
[20 秒静止实测](validation/overview_bpu_stationary_20260905.json)，
[实际 ROS 拓扑与运动门](validation/overview_bpu_graph_verified_20260905.txt)，
[本轮 BPU 逐帧日志](experiments/j6m_bpu_20260905/live/qt_20260905_220615_867705/results.jsonl)。

本轮证据：`validation/overview_bpu_contract_20260905.log`、
`overview_bpu_build_20260905.log`、`overview_bpu_catkin_tests_20260905.log`、
`overview_bpu_static_20260905.log`、`overview_bpu_cold_stop_20260905_2152.log`、
`overview_bpu_full_start_20260905.log`。停机已确认本机 CAN 无占用、J6M 所属进程清除，
原版与候选服务都 inactive 后才重新启动，未使用宽泛杀进程。

## 启动与停止

候选版已有完整栈时不要重复启动。以下命令在 NVIDIA 执行，保持两个运动门 false：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
./scripts/optimized.sh start --map-set /home/slam/robot_j6m_ws_optimized_20260905/global_maps/map_sets/latest </dev/null
# 完整启动检查通过后，另一个终端运行实验伴随桥（保持终端打开）。
bash experiments/j6m_bpu_20260905/tools/start_qt_bridge.sh --seconds 600
```

在 Qt 选择“综合”即可看右侧 BPU 检测图，“BPU 大画面”可打开独立预览页。
伴随桥不会自动随导航启动，最多十分钟；到期或 Ctrl-C 只停止 BPU 桥，Qt 保留旧图并警告，
导航/相机不因此退出。当前采用限频至 0.5 Hz 的演示管线，不能承诺视频帧率或实时控制。

完整停车和退出：先在伴随桥终端 Ctrl-C，再执行：

```bash
./scripts/optimized.sh stop </dev/null
```

启动会重新等待现场初始位姿，不猜测或复用旧值；不发送目标、不启用视觉行驶或开放运动。
只有最终 watchdog 可发布 `/cmd_vel`，避障、急停、失联停车保护保留。

## 切回原版与未完成项

停止本次伴随桥和候选完整栈、等待统一入口报告清理完成后，原项目无需回滚文件。
但原版既有配置可能放行运动，当前只有静止授权：不得直接启动原版带运动放行的配置。
需要原版实际启动时，应先按原版流程明确现场安全和运动门选择，不能暗中改原配置。
原入口为 `/home/slam/robot_j6m_ws/scripts/start_dual_host.sh`，不能与候选同时运行。

J6M 导航 release 未重新部署：候选 `20260905_160700/install`，原版
`20260905_122153/install`。Qt 为 NVIDIA 本机新构建，BPU 仍在独立远端实验目录运行。

18:22 左轮 ±0.01237 m/s 脉冲的根因仍未查明；本次短测零值不能证明该问题已修复。
全局定位稳定性仍未验收，ZED 自标定警告尚未解决；出现新异常应立即统一停车排查。
本次 Qt 还有启动时规划参数事务同步超时警告：等待 TEB 动态参数服务超时。
截图和 GUI 行为保留该诊断；尚未设置初始位姿，不自动改参数或据此声称规划执行链验收。
FCOS 仍是通用 80 类参考检测，不能替代原垃圾检测 + 五材质模型。
未完成原模型 BPU 量化/生产切换、GPU 受控 A/B、长期运行和故障注入、深度精度及运动导航验收。
当前仍运行 NVIDIA 正式后端，不能把本次界面替换表述为已经释放其 YOLO GPU 资源。
