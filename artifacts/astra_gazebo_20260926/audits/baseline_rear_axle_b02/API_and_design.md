# B02 后轴规划参考点组件与集成提案

新 `baseline_rear_axle_adapter_b02.py` 是纯组件，未接 ROS、launch、参数、种子或当前规划器。复用 `axle_reference_adapter.py` 与 `geometry.Pose3`，没有复制另一套 Ackermann 积分器。源码诊断来源为 historical 的 `baseline_tuning_b01/reference_frame_addendum/report.json`（作者补充诊断，不冒充独立资格）。

参考点关系是 `r_BR=(-0.325,0,0)`、姿态方向相同。位置 `p_R=p_B+R_B·r_BR`，速度 `v_R=v_B+ω×r_BR`。平面情况下 `vx_R=vx_B`、`vy_R=vy_B−0.325·ω`。在理想无侧滑转弯时，中点 vy 不为零；转换后的真实 rear vy 仍必须保留，不能用“非完整”假设清零。

组件提供以下实际接口：

| 接口 | 支持范围 |
|---|---|
| `planar_pose/twist_to_rear/base` | 复用旧 helper 的纯 SE2/三分量可逆转换 |
| `rigid_pose/twist_to_rear/base` | 完整 xyz/quaternion 与六分量 body twist；保留真实 tilt、vz、wx/wy |
| `planar_*_covariance_to_rear` | 3×3 均保留相关项；pose 为一阶 Jacobian，twist 为线性精确传播 |
| `rigid_*_covariance_to_rear` | 完整 6×6；pose 只接受 parent fixed-axis 小旋转误差约定，twist 只接受 child/body axes；不支持任意 Euler 参数 covariance，显式拒绝 |
| `footprint_to_rear` / `local_points3_to_rear` | 所有顶点平移 x+0.325，保留顺序/个数/z；不重新 hull、缩形、padding 或删轮 |
| `StampedOdometry` / `adapt_odometry` | source/receipt/available 纳秒与 parent frame 原样保留；child 改为 `rear_axle_plan_b02`；不可早于实际可用时刻消费；不重打时间、不代替调用方原 freshness 门 |
| `convert_fixed_route` / `get_plan_request_to_base` | 每个有朝向的 route pose 精确转参考点，保留终点指定 yaw；GetPlan 请求逆回原 base 语义 |
| `base_goal_reached_from_rear` | 重建 base 的几何谓词用于测试；共享 coordinator 仍使用原 base odom 作为结果权威 |

协方差约定必须显式传入。完整 pose 的 Jacobian 为 `[[I, −skew(Rr)],[0,I]]`（parent-axis 位置/小旋转误差）；twist 为 `[[I,−skew(r)],[0,I]]`（body 线/角速度）。没有忽略 xy/yaw 或线/角 cross covariance。pose 传播是局部一阶近似，不声称对有限非线性随机旋转分布精确。P05 odom 源未填写 covariance 而为全零；组件保留零，不据此声称无不确定性。未知/不同 convention、非有限/明显非 PSD covariance 会拒绝。

拟议一贯 wiring（没有创建 node/launch/config）：

1. 保留物理 P05 `/odom`、`odom→base_link`、map→odom、机器人 URDF/sensor TF 与 coordinator 原样。一个唯一新 owner 发布静态 `base_link→rear_axle_plan_b02`，平移(-.325,0,0)、无相对旋转。不要同时发布第二条 `odom→rear`，否则 TF 子节点有重复父链。
2. 唯一新 odom adapter 发布 `/astra/baseline_b02/odom_rear`，header frame仍 odom、child rear，精确源时戳与完整 twist/covariance。TEB 与 DWA 的 **实际 `odom_topic` 参数** 都显式指向它；只改 remap 或只改 TF 都不够。原 coordinator、evaluator 继续读 `/odom`。
3. 两个 costmap 的 `robot_base_frame` 与 TEB 读取的 costmap robot reference 一起改 rear；map/global frame不改。实际现有 TEB 16 个 raw footprint 顶点、两个 costmap 64 个已含 Euclidean .33 的顶点都平移 x+.325。`footprint_padding=0`、TEB `.33` 障碍距离及全部物理/速度/角限保持原定义，不双加 margin。Sensor frame仍是原 laser，经同一 TF 树转换，不伪造新 scan。
4. 新 GetPlan shim 用 rear start/goal 的**各自完整 yaw**逆回 base 后调用原 `/astra/common_route/get_plan`，对原返回的每个 oriented pose 做同一转换；共享静态 map bytes 和 canonical base guide不变。新服务例如 `/astra/baseline_b02/get_plan`，原 `FixedRouteGlobalPlanner` 可按新 `route_service` 参数使用。它检查返回 path 最后一项与请求 goal 的 XY≤1e−6，因此请求 goal和path endpoint必须同时改 rear。原 provider还比较目标 yaw和XY；直接给它rear goal会明确拒绝。不得改 XY 后重新按 rear chord 算 yaw，最后一项必须保留原 explicit goal yaw。不得给不同方法重新 A*、平滑或换路线。
5. 各基线 planner 的内部 path/goal/velocity 全部声明 rear reference，输出仍只有 vx/w；r沿纵轴使两点 vx、w 相同，所以保持现有 relay/driver/warmup/gate 不变。真实 rear vy用于状态估计/诊断，不成为允许发横向命令的理由。原 coordinator 对横向 method command 的检查仍保留。
6. 记录 base/rear source-stamped状态、full covariance convention、base/rear oriented-path SHA、实际 served/provided/local plan、有效 TF owner与足迹哈希。独立新方法身份，例如 `actual_teb_rear_reference_b02`/`dwa_rear_reference_b02`，不能覆盖旧 baseline 结果。参考点适配前后路径打分/局部窗口可能变化，需新鲜成对工程资格。

有三项不能用组件 pass 掩盖的边界：

- **目标容差不等价。** 精确 goal centre 可以转换，但两个参考点相同 .25 m 圆并不等价。若 rear 位置误差 .24 m、yaw误差 .20 rad，选择同向误差可使 base 距离达 `.24+.65*sin(.10)=.304892 m`。测试有真实数值反例。共享 coordinator 的 base `.25/.25` 判定保持原样；局部 planner 是否过早停止仍需专门资格。理论充分半径为 `base_tol−.65*sin(rear_yaw_tol/2)`，这里只报告公式，未采用新半径/配置。后继必须明确选择等价 base predicate wrapper 或经预注册的内部停止策略，不能把 rear reached当base成功。
- **完整 tilt 与二维 costmap。** 完整 rigid state/三维点变换精确；SE2下全部 16/64 世界多边形顶点精确不变。若实际 base pitch非零，静态rear TF后只取yaw的二维 costmap投影与原base投影有 `0.325*(1−cos(pitch))` 的平移差（pitch=.05时约0.000406m）；测试保留该非零反例。不得宣称在tilt下平移静态2D footprint也严格等价。现有 tilt-envelope能否覆盖这一参考变更，需要新集成几何审查；本组件不擅自加padding或改.33。
- **仍不等于 Ackermann 可执行规划。** 共享guide的heading折点可能在rear XY产生跳变/短段，必须保留对应关系、检查实际消费，不能用此转换冒称平滑可达。DWA仍无显式转角/转角速率约束；TEB carlike只是软项。DWA reversed vy sampling window也不能一概宣称非法：现实现含0端点，是否输出非零vy依赖实际评分。组件未增加 `acc_lim_y`，不拿它作普遍修复。

32项小型离线检查通过，包括实际配置的 16/64 顶点世界集合不变、左右转与真实侧滑、完整倾斜pose/三维点、六分量twist、协方差有限差分、源可用时边界、route末项显式yaw和goal反例。它们不提供 live/闭环/摩擦或收益资格。后继需独立源码review与实际 TF/odom/costmap/goal 接线检查，然后新鲜有界工程回合，不能复用旧seed或改旧结果。
