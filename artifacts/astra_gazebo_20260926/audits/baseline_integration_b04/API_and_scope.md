# B04 静态后轴基线接线（作者离线组件）

本包把已冻结 B03 两个方法接入新的 B04 静态采集器和评估器，不启动 ROS/Gazebo、不分配实验身份、不写 execution release。`actual_teb_rear_reference_b03` 和 `dwa_rear_reference_b03` 保持 B03 方法身份；本包不是 baseline 调参或性能结果。

TEB 和 DWA 均从真实原始 `/odom` 派生 `/astra/rear_odom`，参考点为原 base_link 局部 x=-0.325 m。冻结 B03 adapter 发布唯一 `base_link -> rear_axle_kinematic` 静态 TF，保留完整 quaternion、三维点速度及协方差和真实横向速度；最多 200 ms 时间可用性界限不改变、不重新打源时间。每个输入的 source/received/available 时间与实际输出留存。后两者是本机 ROS 回调/可用时刻，不是额外硬件测量源时间。

公共原地图和完整有向 guide 不变；后轴完整 polyline 与终点按各点方向变换。冻结 FixedRouteGlobalPlanner 调用 `/astra/baseline_rear/get_plan` 并发布实际 provided_path。collector 同时记录 served/provided 两份路径、单个 action goal、适配器 identity/status、TF 和原/后轴里程计。TEB/DWA 的 costmap base frame 和 odom 参数均指向后轴；64 点外扩 costmap footprint 和 TEB 16 点原始多边形整体在局部 x 方向加 0.325 m，原 padding / 0.33 m 工程距离不改变。完整实际参数分别在暂停启动和终态读取核对。

公共 `/cmd_vel_navigation -> relay -> cmd_vel -> P05` 链、Kd6 候选物理执行器及速度/加减速/转向限制不变。私有 move_base 命令只进入 `/astra/method_cmd`。公共 gate、0.25 m / 0.25 rad goal 和 22 s deadline 使用原始 base_link source odometry；dense world base_link 是离线实际几何复核。私有后轴 goal tolerance 为冻结 B03 的 0.15 m / 0.10 rad，DWA 不 latch XY。私有 action SUCCESS/ABORT/PREEMPT 的回调接收时刻仅是诊断，不能制造任务成功；四个真实节点假传输场景验证 private-first 和 base-first 的终态源时间相同。

B04 collector 在实际注册图检查后先 unpause、等正时间 TF/规划器/回执和 bag 连接，再在固定 5 s 之前 arm。bootstrap 检查不强求晚初始化规划器的 provided_path/goal/private command transport，避免暂停时钟启动死锁。collector required=true；本包不新增第二个 spawn/unpause/pause 调用者。

暂停后沿用同一 5 s drain 预算及稳定真实世界 watermark。新增原/后轴/回执 50 Hz 源序列从 gate 到 watermark 前最后实际 slot 的完整 join。最终 EvidenceStore 同锁冻结回调；错误/缺包/重复/回退/错 frame/错 caller/错误三维变换/裁掉 vy/错协方差/错路径/错私有目标/错有效参数都会使 capture integrity false。离线 assessor 另从 bag 重建这些消息，显式读取 ROS bag connection headers（bytes 严格 UTF-8 转换），独立审核并与 collector 的三条正式源流逐字段匹配；bag 接收时刻不强求等于 collector 接收时刻。

B03 `publish_pending()` 部分输出后第二条过期的已知行为保持：第一条后轴输出可能没有回执。实际函数反例在 B04 重现并判缺证失败；不补造回执、不把不完整证据当算法成功。失败及没有私有 SUCCESS 的情况不被排除，图像仍为独立的辅助覆盖信息。

## 调用与文件

- `baseline_rear_evidence_b04.py`: 纯消息提取、图/参数/同源几何与 bag 对应检查；无 ROS 执行。
- `navigation_evidence_b04.py`: N02 静态公共证据链的新派生版本；增加闭合 rear reference 合同和完整性门，保持 dense 物理评价和 base goal 数值算法。
- `astra_navigation_collector_b04.py`: 新独立 collector；只允许显式 loopback 14832（对应未来 Gazebo 14833 尚未预约）。
- `astra_baseline_rear_b04.launch`: 仅固定使用 B04 collector，B03 adapter/coordinator/config 未改；两个 actual planner class、全 remap/参数和 paused world 均经 `roslaunch --dump-params` 及实际 ROSLaunchConfig 解析。
- `prepare_offline.py`: 仅生成 audit 目录下参数展开夹具；seed=1 是未登记的语法占位，无运行预算/身份。输出目录参数指向不存在的 run 路径以验证形状，从未创建或使用。
- `derive_sources.py --check`: 比对由冻结 N02/B03 输入机械派生的三份 B04 输出，检查模式不改任何文件。

## 尚未解决，不能作为 live 资格

1. 只接受显式展开的 static world；动态 actor 插件、`<actor>` 和未展开 `<include>` 拒绝。冻结 N05 C++ actor 方法白名单尚不支持 B03 名称，动态六族公平对照明确阻断；不得据此声称动态兼容。
2. 未建 B04 execution manager/release/seed schedule。将来的独立源审查、实际 ELF/interpreter/dependency 完整绑定、CAS、隔离清理、URDF→SDF 与 runtime model 闭包、真实 TF/odom/goal 传输资格、native PNG/失败保存仍需单独完成；本报告的资源清单不是执行 release。
3. 完整三维刚体后轴变换不使 yaw-only costmap polygon 在 roll/pitch 下精确等同物理投影。B03 独审保存过 0.0004061653716359581 m 示例差异，未增 margin 或掩盖。倾斜几何和刹停/转弯必须另行资格。
4. 原始有向 polyline 各点后轴变换可能改变弦长/折线曲率；不是整体 Ackermann 可行性定理。当前展开用继承静态 clear fixture，不是六族或新场景统计样本。
5. DWA 残余 rear vy、不完整约束 rollout、stop-and-rotate 和冻结 acc_lim_y=2.5 仍可能导致命令失败或 timeout。TEB/DWA 私有较严格目标容差与异步 callback 次序也可能让 planner 停止早于公共采样观察；保留任务失败，不宣称已经通用修复。
6. 注册图/消息 caller 和可信冻结源码是观察证据，不是对恶意同进程/插件的即时强制隔离。没有性能、形式安全、连续时间物理安全或实车资格。
