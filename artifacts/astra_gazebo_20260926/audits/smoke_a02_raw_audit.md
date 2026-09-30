# A02 原始证据独立离线审计

independent_raw_audit_completed

Read-only raw audit. No simulation or historical rerun. Headerless message times are receiver-clock approximations. No general physical safety or contact coverage claim.

| 方法 | 原始核查 | scan Hz | odom Hz | 复算加速度范围 m/s² | 最小墙体投影净空 m |
|---|---|---:|---:|---|---:|
| pct_scope_rl | 所列检查通过 | 14.7059 | 50.0000 | -0.900000…0.600000 | 1.999691 |
| analytic_prior | 所列检查通过 | 14.7059 | 50.0000 | -0.900000…0.600000 | 1.999694 |
| actual_teb | 所列检查通过 | 14.7059 | 50.0000 | -0.900000…0.600000 | 1.999723 |
| dwa | 所列检查通过 | 14.7059 | 50.0000 | -0.900000…0.208559 | 1.999707 |

| 方法 | cmd 接收频率 Hz | 源 dt 复算最大绝对 jerk m/s³ | 四轮策略窗高度极值 mm | 策略窗 TF 事后覆盖 | recorder 当时已有 TF |
|---|---:|---:|---|---:|---:|
| pct_scope_rl | 20.0394 | 45.0000 | -0.048933…0.008509 | 134/134 | 4/134 |
| analytic_prior | 20.0263 | 45.0000 | -0.048897…0.008289 | 134/134 | 27/134 |
| actual_teb | 20.0000 | 75.0000 | -0.048443…0.041779 | 129/129 | 0/129 |
| dwa | 20.0000 | 55.4279 | -0.050282…0.051206 | 144/144 | 1/144 |

这些回合的发布转向与请求 yaw 均为零；转向幅度/变化率检查在直走数据上成立，**转弯动态没有受到激励，不能据此资格认定**。

jerk 是相邻源时间加速度区间均值对区间中点时间的差分，未设置事后阈值。旧固定 0.05 s 统计出现的 1.08 m/s² 不代表源时间内部状态违反 0.90 m/s² 制动限值。

bag 首尾少数 scan 没有完整 TF，策略评价窗口内全部可用；最早期的 machine_qc 失败帧继续保留。无等待的 recorder 次序检查刻画异步到达，不能与节点 0.03 s 等待后的结果混为一谈。

完整结果、SHA、频率/单调性/TF 逐总体计数、四轮高度极值、compound 几何和全部初次工具错误见 JSON。

解释边界：/odom 与 joint_states 是 SetWorldPose 运动学插件发布的内部状态；其源时间差复算比历史固定 0.05 s 导数可靠，但不等于接触/摩擦决定的真实物理加速度。无 header 的 model/link/Twist 使用 bag 接收时钟，不能假装精确源时间。全 bag TF 覆盖是事后结果；recorder 到达次序失败不能等同 evaluator 的等待队列失败。contacts 空消息只证明发布通道存在，零值不是无碰撞证明。0.33 m 是本诊断明确附加的几何裕量，不是替换旧保守圆评价器。

没有写入任何 run 目录，没有修改 H01–H03，没有启动或连接 ROS/Gazebo。
