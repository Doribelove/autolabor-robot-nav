# 清扫碰撞检查范围调整（2026-09-09）

11:22 后续：操作员已自行启动本 release 并设置本次初值，运行参数 5.0 m / 4.0 m
已只读核对，三轮清扫最终完成；第三轮有执行重试暂停后恢复。最新实测和剩余问题见
[实验分析](HYBRID_CPU_EXPERIMENT_20260909.md)。下方 00:36 停止状态是部署当时的记录。

用户确认此前阻挡栅格处曾有人，后来已经离开；因该位置远离当前局部代价地图，
全局层保留了旧障碍。用户要求先保存当前代码到 `0908` 分支，再将清扫入口前方
障碍检查范围改为可配置的 5 米并部署到 J6M。

修改前源码与交接文档已保存至本地 `0908`，提交 `9c69b7d`；未推送。
日志、地图、私有配置和授权标记继续保留在本地原位置，未强制加入 Git。
后续本轮变更仍在该分支的工作树中，可用上述提交对照修改前状态。

## 行为与配置

配置文件：`src/application/autolabor_coverage/config/coverage_global_planner.yaml`。

```yaml
sweep_collision_check_horizon: 5.0
```

`navigation_j6m.launch` 将此文件加载到 `/move_base/CoverageGlobalPlanner`。
参数在插件初始化时读取；修改后需正常部署/重启加载。零、负数、NaN 和无穷值被拒绝，
不会解释为关闭检查。

ENFORCED_SWEEP 现在使用有限检查距离：首次从入口附近实时位置沿线检查前方 5 米，
之后每次规划随车辆位置推进，不会永远只检查原入口前 5 米。距离包含实时车辆到最近
路径采样点的连接长度，窗口内每个采样姿态仍按带 padding 的完整车体轮廓检查；
因此中心位于 5 米边界时，车头外廓仍需无碰撞。

远端致命障碍不再立即拒绝整条清扫线；近端或当前车体碰撞仍拒绝，日志会记录阻挡
发生在前方多少米和当前检查范围。完整剩余路径与原清扫终点保留，未知空间、地图边界、
TEB 局部碰撞检查与全部运动门保留。覆盖管理器准入/完成/重试逻辑未修改，Hybrid 的
3 米碰撞检查与 entry_recovery 的 4 米总长限制均不变。

## 验证与部署证据

指针：`validation/sweep_horizon_fix_latest.txt`。隔离真实 ROS 插件场景由
`src/application/autolabor_coverage/test/run_sweep_collision_simulation.sh` 运行，
独立网络/PID namespace 只有 lo，通过 make_plan 服务查询，不发 action 目标或非零速度。

旧版已复现 15 米外障碍拒绝完整路径。本机 Release 构建与覆盖包 193 项 Python
测试通过。新增真实插件 36 项集成检查通过：5 米和 2 米配置各 14 项，以及零、负数、
NaN、无穷值各 2 项（含无运动输出断言）。近端/远端障碍、随车推进、窗口边界车头外廓、
侧边外廓、未知空间、地图边界、短线终点、相反地图方向及平移后的现场单栅格几何均覆盖。
测试使用实际 move_base 和 CoverageGlobalPlanner；move_base 的 make_plan 服务可能
返回失败调用留下的部分路径，判定按是否真正到达指定终点，不能只检查 poses 非空。

本机静态检查通过，其中 438 项为既有 catkin 测试汇总，不与本次 193/36 项重复相加。
Shell/Python 语法检查、git diff --check 通过。本机实际 navigation_j6m.launch 参数展开
验证 `/move_base/CoverageGlobalPlanner/sweep_collision_check_horizon=5.0`。

通过候选部署入口进行 J6M 原生 Release 编译，60 秒完成、退出 0，发布
`20260909_003347`；current 已原子切换至该 install。板端静态检查通过。配置、安装头文件、
navigation_j6m.launch、原生编译输入 cpp 及未改动的 coverage_manager.py 共 5 项 SHA256
与本地匹配；安装的 libcoverage_global_planner.so 包含新参数和阻挡距离日志。板端实际
安装 launch 展开的检查距离为 5.0 m，入口恢复总长仍为 4.0 m。

部署前当前任务已由操作员暂停；已保存最终快照并通过统一入口停净双机栈。停机前 MID360
点云/scan 已约 6586 秒未更新，底盘反馈约 5921 秒未更新；00:36 固定 MID360 网口 eth0
（50:54:7B:E3:C9:10）carrier=0，J6M 通信正常。完整导航保持 inactive/MainPID=0，
本轮未完整冷启动、没有实车移动验收，也没有当前运行态参数结论；上述 5 米为安装文件
和 launch 展开的校验。现场恢复链路后需完整启动并由操作员设置本次真实初始位姿，
不能自动恢复已暂停的旧清扫任务。

私有配置与修改前完全相同，主门 true、FOD=false、1.60 m/s 上限和已有授权标记保留。
未发送实车目标或非零速度。Git 的修改前备份提交保持不变；本轮修复和新增记录留在
0908 工作树，未额外提交或推送。
