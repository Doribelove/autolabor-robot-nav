# B05 静态后轴资格：零权限设计与纯组件

B05 是后继候选设计，保留 B04 所有字节。没有执行 release、seed 登记、ROS/Gazebo、训练或新仿真证据。

拟定 clear + 冻结 D04 四个静态 parent，共 5 个几何，各 2 个新模拟种子，TEB/DWA 成对交叉顺序，共 10 块 / 20 回合。263004001–263004010 仅提案，与当前 registry 快照及 N06 提案无冲突；不构成预约，后续必须重新检查。原图/地图/路径绑定保留，重复种子和 N06 共用 parent 不冒充新独立几何。历史任何状态的种子/身份/预算都不能转移；attempt=1，无补跑、替换或续跑。

唯一新代码是无 ROS 的纯校验/记账组件。`compare_full_parameters(prepared, observed)` 对 /move_base 整棵参数树检查精确类型、有限值、列表顺序和所有键；int/bool/double 不互换，浮点不加容差。`compare_parameter_phases` 要求 paused_bootstrap、positive_clock_ready、paused_terminal 全部出现。B04 原窄 gate 能接受的 max_vel_x=17 在这里失败。

期望参数的完整性不是该函数自行证明的：真正 prepare 还须根据冻结 cfg/源码枚举插件/动态重配置/代价图默认值，按阶段生成完整期望并绑定。不能用首次 live 实际值作为期望，也不能忽略意外 auto-default。当前只有 B04 实际离线展开值的比较例子，完整 initialized runtime namespace 尚未实现，仍阻断 release。

公共任务保持原始 actual base_link gate、0.25 m / 0.25 rad goal、22 s cap；私有 rear action 的 SUCCESS 无终态权限。保留 full3D offset/角速度叉乘/真实 vy/协方差与 source/available 区分，原后轴 adapter、TEB/DWA 配置、P05/Kd6 不改。对所有真实九 collision shape、rear-yaw 原始16点/外扩64点 hull 的逐源几何包含和真实 command→relay→driver ingress 顺序/数量/字段闭包列为必须新增的实现门，不能拿源码要求代替实际通过。Ingress receipt 是司机接收/入队时刻，不是力生效；无 header Twist 不凭 bag time 重造 source stamp，重复值按事件计数，末尾未带 trace 的 zero 不能偷偷丢弃。

保留 B03 partial-batch 先输出但无回执的真实负例；B05 同样按缺证失败，不能补回执。真实回执、TF/goal 传输、倾斜 yaw-only footprint、停转/刹停/低速执行均须 fresh 物理资格。动态 actor 不在本设计内。

`proposal_schedule` 只返回 allocated_budget=0 的固定配对提案。`failure_inclusive_summary` 保留20个已闭合身份：实际 valid goal 用源时间，所有已尝试失败用22 s；未执行退休行保持无测量、另标行政22 s上界，不能当作真实失败观测或完整配对。该函数不判资格、不授予性能结论。初始4个 clear 单元未全部有效完成即闭合该资格阶段；后继静态中的算法 timeout 可继续预定顺序，但物理/安全/完整性问题中止并退休余额，不换种子。

20 项作者小 CPU 测试通过。首轮夹具调用 Python3.8 没有的 math.nextafter 失败已保留，仅测试改成明确相邻 IEEE double，生产组件未因此更改。测试包含真实 B04 全参数树和窄 gate 反例、所有类型/缺项/多项/footprint 差异、历史冲突、重复/遗漏失败、真假退休与配对计数。独立审查仍 pending；没有真实 B05 运行或图像，不能声称 rear-base 物理、动态 actor、实时回执、强 baseline 性能、一般安全或实车资格。
