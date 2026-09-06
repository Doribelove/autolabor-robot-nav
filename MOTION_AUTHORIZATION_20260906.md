# 主运动授权开启记录（2026-09-06）

用户明确要求开启运动授权，并确认测试区封闭净空、人员远离车辆、实体急停有效。
用户指定最高线速度 1.6 m/s；此前称遥控停车已多次测试。现场安全条件由操作员确认，
本轮没有由助手执行移动、急停制动距离或失联停车的实车测试。

后续最新状态：已经收到初始位姿，实测 `state=LOCALIZED;overlap=0.9994;rmse=0.1732;inliers=5156`。
主运动门仍 true，BPU 私有 tmux 存活，原项目 1287 项再次核验 unchanged=true。
下文 WAITING_INITIAL_POSE 是冷启动/零速观察时的历史状态，不是最新定位状态。

## 已完成

- 仅候选 `config/dual_host.env`：`MOTION_ENABLED=false → true`；
  `CMD_VEL_MAX_LINEAR_SPEED=1.70 → 1.60`。导航启动默认 0.80 m/s、倒车 0.30 m/s、
  `FOD_MOTION_ENABLED=false` 均未改。最终上限不等于巡航设定速度。
- 根据 README 中封闭净空区授权路径，经现有 `authorize_motion.sh` 创建候选独立
  `runtime/motion_authorized.ok`。脚本参数保留历史命名，没有把现场记成车辆架空。
- 完整冷重启两端，保留 `map_20260829_221335_gimp_cleaned` / fused 静态地图。
  没有恢复旧目标或猜测/重发旧初始位姿。
- 主运动门在实际 watchdog 中为 true，最终线速上限为 1.6，指令超时仍为 0.25 秒，
  `/cmd_vel` 唯一发布者仍是 `/nvidia_cmd_vel_watchdog`。避障、定位门、急停和失联停车未删除。
- NVIDIA 候选服务 active，MainPID=577140，Qt PID=584969；原版服务 inactive。
  未构建/部署新的 J6M 程序 release；current 仍为
  `/opt/autolabor/dual_host/releases/20260905_160700/install`。正常冷启动同步了候选运行配置。
- BPU 已在私有 tmux 中恢复持续运行：`qt_20260906_001122_676676`，桥 PID=595084；
  00:13 检查已更新 47 帧/96 秒。旧会话正常结束于 902 帧/1849 秒，日志保留。
  Qt 综合页已目视确认静态地图和 BPU 框图；BPU 仍仅显示，不是生产控制后端。

## 验证及边界

- 配置/授权脚本 shell 语法通过，watchdog 针对性单元测试 5 项通过。
- 统一启动返回 0，完整 runtime 检查通过；启动器检查的已有测试结果汇总为
  952 项、0 错误、0 失败，不表示本轮重新执行了全部 952 项测试。
- 00:02:33 起 12 秒开启前观测：240 次定位均 LOCALIZED，最终指令和两轮反馈为零，
  必需传感器数据连续、控制发布者唯一。
- 冷启动后的 00:12:41 起 10.07 秒：主门 true、FOD false、498 条最终指令及
  左/右轮 417/411 次反馈全零；融合避障持续 active，无保留/活动目标。
  定位为 WAITING_INITIAL_POSE，因此 move_base action server 的完整验收留待操作员定位后。
- 新增 `validation/verify_motion_enable_20260906.py` 仅作一次性只读检查，不是控制节点。
  首次探针误将事件型暂停状态当心跳，并要求初始位姿前已有 action 状态，返回了两项
  观测缺失；保留首次 JSON。按源码/实际启动语义修正诊断后复测通过，没有修改任何生产安全门。
- 原项目 1287 项快照核验 unchanged=true。只修改候选配置、诊断/交接文档及候选运行数据；
  没有修改系统服务、驱动、依赖或原项目文件。

## 当前操作

主运动授权已开启，不需要再改 true，本次初始位姿已收到且已 LOCALIZED，可由操作员
发送新目标。以后冷启动需再用 Qt“② 设置初始位姿”指定真实位置和车头方向。
首次定位前显示“导航未启动”是
move_base 尚在等待 map TF，并不是主运动门仍为 false。没有自动发车。

本轮未验证移动导航、1.6 m/s 跟踪/避障/制动距离或动态障碍场景，不把授权成功写成实车验收。

候选完整启动（停车后使用）：

```bash
/home/slam/robot_j6m_ws_optimized_20260905/scripts/optimized.sh start \
  --map-set /home/slam/robot_j6m_ws_optimized_20260905/global_maps/map_sets/map_20260829_221335_gimp_cleaned </dev/null
```

停车并完整停止：

```bash
/home/slam/robot_j6m_ws_optimized_20260905/scripts/optimized.sh stop </dev/null
```

停止不静默撤销现场授权。切回原版必须先完整停止候选并核实 CAN/ROS 所有权释放，
再用原版已记录的启动入口；不能同时启动两套底盘控制。BPU 伴随预览须在新的 Qt/master
建立后启动，具体私有 tmux/持续运行方式见 `CONTINUOUS_BPU_20260905.md`。

证据：

- `validation/motion_enable_config_before_20260906.env`：改前配置。
- `validation/motion_enable_preflight_20260906_000232.json`：开启前静止观察。
- `validation/motion_enabled_restart_20260906_0005.log`：完整冷重启和 runtime 检查。
- `validation/motion_enabled_runtime_20260906_0011.json`：首次探针的预定位观测缺失。
- `validation/motion_enabled_runtime_20260906_0013.json`：修正诊断后的 10 秒授权/零速观察。
- `validation/motion_enabled_qt_20260906_0013.png`：实际新 Qt、静态地图与 BPU 画面。
