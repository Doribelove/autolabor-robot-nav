# 导航优化交付记录（2026-09-07）

## 工作区与部署边界

- 独立工作树：`/home/slam/robot_j6m_ws_navigation_20260907`
- Git 分支：`feature/navigation-optimization-20260907`
- 来源：`robot_j6m_ws_optimized_20260905` 在开工时的源码状态，包含当时已有的 tracked 和
  untracked 修改；原工作区未清理、重置或修改。
- J6M 独立运行根：`/map/robot_j6m_navigation_20260907`
- J6M 原生 ARM64 release：`20260907_141331`
- `current`：`/opt/autolabor/dual_host/releases/20260907_141331/install`

部署没有启动导航、恢复旧目标或执行车辆运动。新工作区没有
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
- 部署后：`current=20260907_141331`，`/map` 尚余约 4.0 GiB；11311 无监听，move_base、
  coverage_manager、FAST-LIO 均未运行。

没有做实车运动、真实转场耗时或复杂障碍场景的性能验收。首次动态验证需由现场重新完成本候选版
的运动授权、真实初始位姿和安全条件，再通过 README 顶部的统一启动入口进行。

## 本地回退版本

2026-09-07 15:03 将本交付状态保存为本地标签
`local-rollback/navigation-optimization-20260907-150355`。工作区外备份目录为
`/home/slam/local_rollback_versions/navigation_optimization_20260907_150355`，其中包含 Git
bundle、源码归档、Git 忽略的地图与运行资源归档、SHA-256 清单和独立目录恢复脚本。
恢复脚本只创建新目录，不覆盖现有工作区。
