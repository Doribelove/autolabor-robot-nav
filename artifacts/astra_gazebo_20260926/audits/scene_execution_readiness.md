# development_a01 场景独立执行准备审查

状态：**offline audit complete / execution_ready=false**。未启动 ROS/Gazebo，未访问 final seeds。原 v1 生成器、18个布局与36个episode manifest/world保持原字节。

主要证据：18/18布局找到静态前进 Ackermann 可行路径，SDF actor参数0处不一致；3个遮挡布局的6个actor回合存在完整swept square与静态screen相交，且gate初始中心遮挡为0/6。它们不具备遮挡实验资格，必须保留失败诊断并用新版本新身份修复。

|布局|静态路径|路径长度m|搜索节点|最窄未阻塞中心带m|
|---|---|---:|---:|---:|
|static_avoidance_000|static_forward_ackermann_witness|9.2|582|5.208|
|static_avoidance_001|static_forward_ackermann_witness|9.6|568|3.869|
|static_avoidance_002|static_forward_ackermann_witness|9.6|800|4.623|
|single_crossing_000|static_forward_ackermann_witness|9.0|0|16.0|
|single_crossing_001|static_forward_ackermann_witness|9.0|0|16.0|
|single_crossing_002|static_forward_ackermann_witness|9.0|0|10.175|
|head_on_yield_000|static_forward_ackermann_witness|9.0|0|4.002|
|head_on_yield_001|static_forward_ackermann_witness|9.0|0|4.404|
|head_on_yield_002|static_forward_ackermann_witness|9.0|0|4.41|
|sequential_interaction_000|static_forward_ackermann_witness|9.0|0|16.0|
|sequential_interaction_001|static_forward_ackermann_witness|9.0|0|16.0|
|sequential_interaction_002|static_forward_ackermann_witness|9.0|0|16.0|
|occluded_emergence_000|static_forward_ackermann_witness|9.0|0|9.243|
|occluded_emergence_001|static_forward_ackermann_witness|9.0|0|9.452|
|occluded_emergence_002|static_forward_ackermann_witness|9.0|0|9.488|
|width_change_corridor_000|static_forward_ackermann_witness|9.0|0|2.565|
|width_change_corridor_001|static_forward_ackermann_witness|9.0|0|2.299|
|width_change_corridor_002|static_forward_ackermann_witness|9.0|0|2.653|

静态witness采用轴距0.65m、速度0.4m/s、转角≤0.40rad、转向率≤0.75rad/s、转向一阶常数0.18s。碰撞足迹为车身与四轮投影的凸包，前轮使用内外轮Ackermann角；障碍膨胀0.36m含0.33m安全裕量和0.03m采样间隙界。该路径仅证明平面静态存在性，不证明动态actor安全、旧8候选覆盖或22s内完成。最窄中心带不是全局通路证明；障碍可需要绕行。

actor审查采用完整直线路径上定向0.72m方盒的精确凸包sweep，并逐静态box相交检查。旧SetWorldPose actor会强制穿过遮挡screen，这属于环境构造缺陷，不能算算法正常碰撞，也不能靠删frame掩盖。初始5条中心/角点射线只用于排除明显不遮挡，v2将升级真实角分辨率多射线测试。

随机与时序：actor base_speed/response_distance/response_time_margin已实际写入SDF并被plugin读取。response_seed虽然传入但jitter×0，不能作为随机响应；sensor/actuator分别登记的随机流尚未接入live，actuator jitter=0。gate_delay元数据尚未生效，SDF仍是绝对start_delay=5加世界x门。

新增纯组件 `route_world_adapter.py` 提供SE(2) point/pose/box/path双向变换、保持body Twist命令不变，以及首次route gate后延时激活/时钟回退拒绝；3项测试通过。它还没有接入live episode节点或actor plugin，不声明迁移完成。

依赖与复现：JSON逐项绑定场景表、生成器、URDF/xacro、节点、plugin/header、合同和审计源码SHA。复算入口为 `python3 artifacts/astra_gazebo_20260926/audits/scene_execution_readiness_source.py`。首次body-only参考计算与路径已另存draft；最终使用steered wheel compound hull重算。

后续步骤：新增v2生成器，scene input级选择不穿过screen且真实激光多射线初始低可见、接近后可见的遮挡布局；承接已登记prior geometry，使用新parent/layout/episode identities，不覆盖A01。所有方法共用同环境版本，并在live预检测量TF/scan/odom源时刻、车辆落地、所有碰撞、gate时序与随机流。
