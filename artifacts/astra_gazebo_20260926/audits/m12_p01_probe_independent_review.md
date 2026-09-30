# M12-P01 探针独立离线审查

当前冻结探针通过39/39合成评估与故障注入样例，另有roll/pitch/yaw90°三个独立逆旋转闭式样例通过。所有数据是明确标注的解析toy，没有启动ROS，也没有把历史Q02轨迹改作新物理证据。历史raw只读取了有效参数元数据；旧回合和源码均未修改。

目标参考仅来自 `/m2_driver/actuator_target` 的四个具名字段。目标的绝对速度、角度、20ms secant和已发布的最近2ms导数各自校验；实际odom、joint_states、link pose独立对照dense真值，实际加速度不被当作目标加速度。保持原36s动作序列、原0.6/0.9/0.75目标边界及实际0.85rad/s门不变。

测试覆盖：dense丢步/重stamp/跨topic错stamp/空包体/错shape/同stamp不同数据/秒纳秒不一致；目标字段缺失、重复、Ackermann几何错误和超限；合法非零vy的坐标旋转与假odom速度拒绝；仅四轮碰地例外及底盘/转向架/障碍/自碰撞拒绝；非有限数据、末段backend fault、仅startup的陈旧fault数据、错误metadata、commanded torque超限或缺包；单位四元数且数据源一致的真实actual-slew失败仍拒绝。

保留前三轮离线历史：第一轮toy未适配新增target names/count/velocity字段而baseline拒绝，不能当真实bug；第二轮复现秒/ns不一致、target全程+10速度、假actual odom三个false-pass；第三轮只剩startup单false误充全程无fault。最终四项均关闭，全部源码执行快照与SHA在JSON，测试期间未变。

独立展开新URDF并调用冻结probe的有效参数/world校验通过；driver接口源审确认actual值在WorldUpdateEnd读取、参考另发、body vy保留，只施加有限joint force。新dense插件每个物理步记录body状态并保留旧50Hz相位；runner记录全部新增topic且assessment_exception按基础设施失败记账。更完整的driver/controller/摩擦资格由对应独立组件与真实回合验证，不能由本探针toy pass代替。

限制：backend_fault Bool没有源header，用receiver ROS时钟验证95%计数、0.04s首尾与0.06s接收间隙，时基弱于其他source header；commanded effort不是反作用力测量；0.85门仍为原50Hz/20ms评价，不能等价成连续时间物理限值。未来真实失败必须原样保留。

探针SHA：`2ded4d98e2c6191c50bfa3a55792b211f4be93500fbb76f0a473f2b848dc6f4f`；测试SHA：`894c402d6a68286feffeae86b887aa716cb0faa7a101c334b4fd2644d3710125`。完整fixture见 `m12_p01_probe_independent_review.json`；展开URDF、有效参数和runner/driver源码SHA见 `m12_p01_integration_static_review.json`。
