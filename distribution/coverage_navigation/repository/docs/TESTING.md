# 构建、测试与证据边界

## 可复现入口

1. `python3 tools/geometry_demo.py`：标准库调用实际几何算法，生成 JSON/SVG。
2. `bash tools/build.sh`：Release 编译三个源包，包含 ROS 消息、两个覆盖库和定制 TEB。
3. `bash tools/test.sh`：运行 catkin 已注册的测试，并检查 XML 结果；另外执行所有权桥测试。
   `tools/test_summary.py` 对嵌套 GoogleTest XML 按 testcase 计数，避免重复计算。
4. `bash tools/run_demo.sh sweep`：真实 move_base/插件，合成静态地图，默认 5 m 窗口。
5. `bash tools/run_demo.sh sweep --horizon 2`：验证可配置距离，而非固定写死 5 m。
6. `bash tools/run_demo.sh preview`：真实覆盖管理器、Navfn、TEB 和所有权桥的首入口场景。
7. `bash tools/run_demo.sh yaw-free`：真实move_base/Navfn/定制TEB/所有权桥，验证单目标
   自由终点朝向、固定朝向隔离、新鲜物理停车和策略时序。

ROS 场景通过仅 lo 的网络 namespace，或 Docker `--network none` 运行。它们断言网络
隔离，并记录不存在运动目标/非零速度；所有测试进程退出后关闭。容器的结果目录是
`/coverage_ws/validation`，可挂载到宿主机保存。不要在现有实车 ROS master 下直接运行
这些会发布合成状态/地图的 fixture。

## 测试覆盖

| 层级 | 测试 | 能证明什么 |
|---|---|---|
| 几何 | `test_coverage_geometry.py` | 多边形、可达性、扫掠、方向/顺序和时间代价 |
| C++ 搜索 | `test_hybrid_a_star.cpp` | 运动学、已知自由区域、解析连接、倒车、取消和预算 |
| 状态机 | `test_coverage_manager_state_machine.py` | 入口、任务代次、暂停取消、恢复、目标交接，使用 mock 执行器 |
| 控制与桥 | mux、pause bridge、底盘、TEB gtest | 挡位、曲率、目标所有权及对应基础约束 |
| 真实插件 | sweep fixture | 近/远障碍、窗口推进、外廓边界、未知空间、无效参数 |
| 真实 ROS 组件 | preview fixture | 实际 Navfn 首入口路径、绕墙、取消、换图和保留规则 |

`sweep` 每个有效距离参数有 14 项检查；无效参数 0、负数、NaN、无穷值各 2 项：

```bash
bash tools/run_demo.sh sweep --horizon 0
bash tools/run_demo.sh sweep --horizon -1
bash tools/run_demo.sh sweep --horizon nan
bash tools/run_demo.sh sweep --horizon inf
```

这些命令期望插件拒绝初始化，测试本身应返回成功。两组有效距离加四组无效参数共 36 项。
不要把 move_base 服务返回的部分路径误判为完整成功，fixture 校验是否到达请求终点。

首入口 fixture 可通过 `--baseline /path/to/baseline_coverage_manager.py` 追加历史版本
对照，旧文件须匹配脚本内 SHA-256。公开导出不携带该本地历史资产；缺省仍执行全部
当前版本场景，并明确记录 historical_baseline_not_supplied。不能将缺少旧版对照写成
完成了旧版复现。

自由朝向fixture使用合成位姿而非驱动车辆，包含10项检查：非法/旧策略拒绝、90°与180°
航向差下的XY完成、同点固定朝向不继承策略、显式匹配朝向、运动中只制动不完成、新鲜
停车后完成、陈旧零速不能完成，以及重规划过程中策略标记保持。

## 与原整机工作区的区别

`test_coverage_contract.py` 同时检查部署、Qt、robot_bringup、底盘等整机文件，
不适用于这个三包算法导出。源项目默认继续运行；公开构建明确传入
`-DAUTOLABOR_COVERAGE_WORKSPACE_CONTRACT_TESTS=OFF`，不会将其跳过数伪装为通过数。
其余几何、状态机、mux、C++ 与插件测试仍运行。该选项不改变运行时算法。

历史源项目曾通过 193 项覆盖 Python 检查和 36 项 sweep 插件检查，但公开导出采用
不同测试集合，实际数量必须以本次 `catkin_test_results` 和各 fixture 的 results.json
为准。[VALIDATION.md](VALIDATION.md) 记录本次独立副本实测环境和结果。

## 实车数据不等于回归通过

2026-09-09 三轮操作员实验最终完成，第三轮有人工恢复；第一、二轮较重 Hybrid 搜索
耗时约 0.55/0.70 秒和 0.41/0.44 秒，搜索短时并行使用接近两个核。第三轮出现大量
接续重规划和无进展，未作无干预通过结论。不同机器、路线和障碍条件下不可套用这些性能。
原始实车地图、日志、perf 和个人配置留在源工作区，不随本公开算法仓库提交。

2026-09-14的当前版本又完成一轮10段覆盖任务，用时275.604秒；7次短暂
`WAITING_OBSTACLE`中有一次跨过整轮重试耗尽后继续自动规划，最终没有进入`PAUSED`。
26次Hybrid action均返回SUCCEEDED，其中3次目标footprint占据/未知后由后续重试恢复；
另有9次2秒低进展取消。该轮整板CPU平均28.55%、峰值53.42%，move_base峰值约1.51核。
这是单一场地的一次操作员实验，不等于所有障碍、人工恢复或定位重定位场景都已验收。
