N01 DWA 的首个故障由有限、非零的侧向 `linear.y` 触发。它仍是平面运动分量，只是不符合本项目 Ackermann 命令契约；日志中的 `nonplanar or nonfinite` 是合并错误名称。这不是 NaN、未配置 Y 限制，或把严格门限放宽即可解释掉的纯序列化误差。

| bag 接收时刻（s） | vx | vy | vz / wx / wy | wz |
|---:|---:|---:|---:|---:|
| 7.442 | 0 | 0 | 全0 | 0 |
| 7.706 | 0.8070158958435059 | 0.000012670447176788002 | 全0 | 0.0010689642513170836 |
| 7.864 | 0.6076943278312683 | 0.00045139779103919864 | 全0 | 0 |
| 7.872 | 0.47857972979545593 | 0.0002892707707360387 | 全0 | 0.0007465453818440436 |

bag connection 的命令发布者为 `/move_base`。三条非零 vy 分别精确等于源时 7.48、7.70、7.86 s 的真实 odom vy 转成 float32 后的值。Twist 没有 Header，以上接收时刻和数值匹配不能冒充规划器生成时刻或精确输入同步证明。首个 fault trace 接收于 7.706 s，当时 coordinator 最新 odom 源时为 7.700 s；gate 源时为 7.440 s，因此原终态窗口为 0.260 s。拒绝后公共输出为零。

实际启动参数已经设置 `min_vel_y=max_vel_y=0`、`acc_lim_y=0`、`vy_samples=1`、`holonomic_robot=false`。最后这个标志不存在于本版 DWA 的生成参数/limits 接口，不能替代 Y 采样约束。已安装 ROS 包为 1.17.3；本机 VelocityIterator 头文件与上游对应版本字节相同。实际日志确认固定采样周期 0.05 s。

动态窗口的 Y 下界为 `max(0, vy-a*0.05)`，上界为 `min(0, vy+a*0.05)`。当 a=0 且真实 vy 非零时，下界大于上界；VelocityIterator 对不相等端点把请求的1样本提升至至少2样本，保留非零端点。DWA 模式的 generator 接受该非零目标，ROS 包装层随后复制选中轨迹的 Y 速度。[轨迹生成器](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/src/simple_trajectory_generator.cpp)、[采样器](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/include/base_local_planner/velocity_iterator.h)、[命令输出](https://github.com/ros-planning/navigation/blob/1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp)。

离线 C++ probe 直接链接当前安装的 `libbase_local_planner.so`，没有 ROS 初始化、master、节点、Gazebo 或命令发布。1212 个输入完成，17 项验证通过；这验证 generator 机制，不重放 costmap 打分和赢家选择。393 个真实 odom 输入在 a=0 时均存在非零 Y 样本；a=0.60 时仍有1个输入失败（6.24 s，vy=0.036252974741212744）；a=2.5 时全部只有零 Y 样本。合成 vy=0.125001 和 -0.13 仍产生非零轨迹，保留为适用域反例。实际 probe 的 `/proc/self/maps` 已保存；当前库证据不能补造原运行缺失的末态 DWA ELF 闭包。

Root 选择新增 `benchmark_dwa_nonholonomic_r1.yaml`，唯一字段变化是 `DWAPlannerROS.acc_lim_y: 0.0 → 2.5`，其余文本精确不变，详见 `configuration_unique_diff.json` 和 `.diff`。2.5 来自[上游默认参数](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/src/local_planner_limits/__init__.py)。它是采样器允许侧向模型速度回到零的虚拟变化能力，不能解释为机器人横向驱动或实测加速度上限。在固定50 ms 周期下，零 Y 样本保证仅覆盖相应浮点条件下的 `|vy|≤0.125 m/s`。本次最大观测值不是未来上界。接近目标的 stop/rotate 同样使用这个加速度参数，未来资格应包含停止段。[停止控制源码](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/src/latched_stop_rotate_controller.cpp)。

修复方案的取舍：

- **新配置（已生成，未接入）：** 保留真实 odom、原生 DWA 代码和严格命令门，改动最小；候选集合及最终排名仍可能改变，必须作为新配置因素资格，不能重判 N01。
- **显式规划器 odom 适配：** 仅在另一个明确命名的规划器输入中把 vy 置零，并保存原始/适配源时对；会丢弃规划模型中的实际侧滑并影响停止语义，需新适配器身份和滑移范围。不能替换评价或驱动 odom。
- **命令后投影：** 直接丢弃已选命令的 vy 会使执行轨迹偏离打分轨迹，需重新验证投影后轨迹，不建议作为最小修复。

新的配置只供后继 N03 单 DWA 诊断资格提案；候选 seed034 未由本工作分配，没有运行或执行授权。已封 N01/N02 coordinator 字节复核不变。N01 的短终态图像不足、后续缺失 final maps 和原 assessment FileNotFound 保持原样，与先发生的 DWA 命令故障分别记录；不据此补判成功、安全或完整 DWA 运行资格。
