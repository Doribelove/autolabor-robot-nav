# 导航优化交付记录（2026-09-07）

## 工作区与部署边界

- 独立工作树：`/home/slam/robot_j6m_ws_navigation_20260907`
- Git 分支：`feature/navigation-optimization-20260907`
- 来源：`robot_j6m_ws_optimized_20260905` 在开工时的源码状态，包含当时已有的 tracked 和
  untracked 修改；原工作区未清理、重置或修改。
- J6M 独立运行根：`/map/robot_j6m_navigation_20260907`
- J6M 原生 ARM64 release：`20260907_212100`
- 候选 `current`：`/opt/autolabor/dual_host/releases/20260907_212100/install`

最初的 `20260907_141331` 部署没有启动导航、恢复旧目标或执行车辆运动；17:10 追加修复后
已按下文记录启动完整 Qt。新工作区没有
`runtime/motion_authorized.ok`，本机和 J6M 配置中的 `MOTION_ENABLED`、
`FOD_MOTION_ENABLED` 均为 `false`。

## 处理逻辑

覆盖管理器不再通过同步 `precompute_transitions` 服务执行正式 Hybrid A* 调用，而是向
`/move_base/CoverageGlobalPlanner/plan_hybrid_transitions` 提交 action：

1. action 的执行线程先锁定并复制一份全局代价地图，后续搜索只读这份快照。
2. 先裁出同时容纳起点、终点和车体 footprint 的 `6 m × 6 m` 地图，预算 `0.75 s`。
3. 失败或端点放不下时改用 `10 m × 10 m` 地图，预算 `1.25 s`。
4. 再失败则使用完整地图，单层上限 `3.00 s`；滚动和恢复请求的总预算为 `5.00 s`。
5. 每层同时运行两条独立搜索。标准路使用启发权重 `1.05`，快速路使用 `1.35`。
6. 第一条可行路径完成后再保留 `0.10 s`，比较两条结果，先选换向次数少的路径；换向次数
   相同时选择预计执行时间短的路径。
7. 障碍距离场、解析 Reeds-Shepp 连接和格点主循环都检查取消标志。任务取消、plan ID 变化或
   外层截止时间到达时，覆盖管理器取消 action，并最多等待 `0.50 s` 让旧搜索退出，再允许
   下一次重规划进入。

action 在 move_base 插件内的独立执行线程运行，因此 move_base 的 ROS 主回调仍能处理状态、
取消、代价地图更新通知和新请求。覆盖任务线程使用 50 ms 的短等待观察结果。后台规划超时后，
本次 action 返回 `OUTCOME_TIMEOUT`；覆盖状态机保持可响应并进入现有重试/恢复分支，旧结果不会
在新的 plan ID 上生效。旧同步服务只保留给旧客户端和既有测试，生产覆盖管理器不再调用它。

## 主要修改位置

- `src/application/autolabor_coverage/action/PlanHybridTransitions.action`
- `src/application/autolabor_coverage/src/coverage_global_planner.cpp`
- `src/application/autolabor_coverage/src/hybrid_a_star.cpp`
- `src/application/autolabor_coverage/scripts/coverage_manager.py`
- `src/application/autolabor_coverage/config/coverage.yaml`
- `src/scripts/robot_bringup/launch/navigation_j6m.launch`
- `scripts/deploy_j6m.sh` 与 `deploy/j6m/health_check.sh`

新候选版还使用单独的本机 supervisor `autolabor-navigation-20260907.service`。启动前会拒绝
原版 `autolabor-dual-host.service`、旧候选 `autolabor-optimized-20260905.service`，并检查两套
旧 J6M 运行根的 PID 记录，防止多个版本共同占用 ROS master 或底盘。

## 验证结果

- AGX/NVIDIA：24 个 ROS 包 Release 全量编译到 100%。
- 覆盖规划：187 项测试通过，包括 15 项契约、3 项 mux、135 项状态机、26 项几何和
  8 项 C++ Hybrid A*；新增取消测试通过。
- 全工作区测试结果：195 项，0 errors、0 failures、0 skipped。
- 隔离与生命周期：10 项测试通过，包含新/旧运行根归属和 PID 记录格式检查。
- 静态健康检查：通过；FOD 模型、CUDA、ASR 三套 checkpoint、ROS 包和 launch 均可解析。
- J6M：在板端 `catkin_make install -j2 -l2` 原生构建，action MD5、共享库依赖、launch 展开、
  6x6/10x10 二进制标记和安装配置全部通过。
- 17:48 冷重启后：`current=20260907_170157`，候选 supervisor active/MainPID=1667551，
  Qt 地图 `READY`，move_base、coverage_manager、FAST-LIO、ZED、双 BPU 感知链均在线，
  195 项检查及严格 runtime 检查通过。

没有做实车运动、真实转场耗时或复杂障碍场景的性能验收。首次动态验证需由现场重新完成本候选版
的运动授权、真实初始位姿和安全条件，再通过 README 顶部的统一启动入口进行。

## 17:10 完整 Qt 启动与 BPU 独立路径修复

首次启动发现本候选复制了 BPU 调用代码，但 AGX companion、共享协议和 J6M 私有工具仍硬编码
旧候选 `/map/robot_j6m_optimized_20260905`。新候选只挂载自己的 chroot，导致 CenterPoint ROS
客户端退出，并由 UI 生命周期的 `wait -n` 正常收掉 ZED、检测器、AI 和 Qt。将 BPU 私有工具改为
相对当前运行根解析，并把所需 vendor runtime、FCOS HBM、原生库和 Python 工具复制到
`/map/robot_j6m_navigation_20260907`；第二次独立测试又发现资产清单遗漏直接依赖
`live_protocol.py`，补齐后双模型服务 `ready=true`，模型哈希与既有验收值一致。

旧候选遗留 BPU worker 只通过其 `service_manager.py` 中的 PID/启动身份记录精确停止；没有宽泛
杀进程或修改旧工作区源码。修复后完成 BPU 包 21 项测试、AGX 全量 Release 构建、195 项静态
检查、J6M 原生部署和板端静态健康检查。统一静态地图启动返回 0，服务 MainPID=1443773；Qt
报告 `READY;width=1311;height=670;resolution=0.100000`。完整链路实测 FCOS BPU topic 约
7--10 Hz、CenterPoint 约 5 Hz；该短窗口只证明链路存活，不是感知精度或持续性能验收。

本次冷启动当前为 `WAITING_INITIAL_POSE`，尚未由操作员提交真实初值。两个运动门均为 `false`、
候选没有运动授权标记，10 条最终 `/cmd_vel` 样本全零；助手没有发布初值、目标或非零速度。
当前未枚举实体麦克风，Qt、AI/MCP 仍在线，但 ASR 采集不可用。CenterPoint 保持候选观察，学习型
障碍物控制入口仍关闭。

## 17:48 Qt 伴随链退出与完整冷重启

用户准备继续实验时发现 Qt 已关闭。最新 `vision.log` 显示 ZED 先报告位置跟踪发散并重新初始化
里程计，随后必需节点 `/zed2/zed_node` 以退出码 `-11`（SIGSEGV）死亡。该必需节点触发其
roslaunch 关闭视觉链；UI 启动器采用伴随生命周期，因此 Qt 同时退出。日志中未出现 Qt/Ogre
自身断言，不能把本次现象写成 Qt 直接崩溃。

按候选统一入口带 `map_20260829_221335_gimp_cleaned` 执行完整冷重启，未单独重启 Qt。命令
退出码 0，服务 active/MainPID=1667551；195 项检查及严格 runtime 检查通过，地图状态为
`READY;width=1311;height=670;resolution=0.100000`。ZED、正式检测器、Qt、AI、FAST-LIO、
move_base、覆盖管理和双 BPU 感知节点均在线。新会话定位为 `WAITING_INITIAL_POSE`，watchdog
报告 `motion_enabled=false`、`active=false`，10 条最终 `/cmd_vel` 样本全零。助手未发送初值、
目标或非零速度。本轮只做故障诊断与重启，没有构建、部署或改变 J6M `current`。

## 本地回退版本

2026-09-07 15:03 将本交付状态保存为本地标签
`local-rollback/navigation-optimization-20260907-150355`。工作区外备份目录为
`/home/slam/local_rollback_versions/navigation_optimization_20260907_150355`，其中包含 Git
bundle、源码归档、Git 忽略的地图与运行资源归档、SHA-256 清单和独立目录恢复脚本。
恢复脚本只创建新目录，不覆盖现有工作区。
