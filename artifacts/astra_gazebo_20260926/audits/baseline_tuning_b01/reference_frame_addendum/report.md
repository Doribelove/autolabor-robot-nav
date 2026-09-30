B01 参考点补充诊断：存在源码可确认的模型参考点差异，需要新工程资格因素。原 B01 报告、24 份 coarse 配置及当前 N04 均保持原样；没有运行仿真或查看新增 N04 结果。

物理 URDF 前轮转向关节在 base_link 的 x=+0.325 m，后轮在 x=-0.325 m。P05 驱动的里程计发布 base_link 原点的实际速度；车轮参考速度则采用后轴无侧向速度的 Ackermann 几何。刚体变换给出 vy_rear=vy_base−0.325ω。因此，即使后轴完全无侧滑，轴距中点仍有 vy_base=0.325ω，不能把这一项解释为真实侧滑。

DWA 在 20 Hz 下使用 acc_lim_y=2.5，窗口半宽为 0.125 m/s。在理想无侧滑状态下，只有 |ω|≤0.384615 rad/s 时该窗口保证折叠为 vy=0。当前两基线允许 ω=0.7805，对应 vy_base=0.2536625 m/s；物理最大转角 0.4964 rad、请求 vx=1.2 m/s 的理想几何值则为 0.3249823426 m/s。后一个例子超出基线的 yaw 请求上限，报告中保留为物理几何支持点，不冒充基线可请求状态。

超出窗口时，DWA 的采样上下界反转，而 VelocityIterator 会把请求的单样本提升为两个端点。例如 ω=0.5 时，实际 float32 输入对应 [0.037499994, 0]。这说明可能产生非零横向候选，并不证明一定选中它或一定失败。单纯任意增大 acc_lim_y 既不能修正轨迹参考点，也不是车辆横向加速度的资格证明。

TEB、DWA 当前都使用 costmap 的 base_link 位姿、base 坐标 footprint 与未经参考点变换的 /odom。DWA 将 vy=0 的 unicycle 轨迹直接积分于该位姿；TEB carlike 非完整约束也把轨迹点视为沿车身方向运动，且该约束为加权软项。它们均没有补偿转弯时中点的 0.325ω 横向运动。现有 footprint 本身与 base 位姿是几何一致的，问题不是 footprint 数值错误。冻结 analytic/PCT 核心已经在后轴积分并转回 base 进行几何检查，因此这个差异也是强基线资格前需处理的公平性限制。

建议独立的新 rear-axle planning adapter 因素：保留权威 /odom 和原 TF，另提供同源时间的后轴规划位姿、完整 twist 与协方差；不能把实测残余 vy 清零。把 TEB 16 边、costmap 64 边的坐标同时平移为 (x+0.325,y)，保持世界占用集合和 0.33 m 裕量不变。planner 的 goal/guide 应依各点朝向变换，继续由原共享 base 几何 goal 裁决。共享 v/ω 输出、物理驱动及原始评估保持冻结。

新因素须先离线验证 SE(3)/平面变换、源时间、协方差、footprint 世界点不变，再以新身份进行有限转弯和瞬态工程资格；这不授权执行。后轴参考适配也不自动让 DWA 的所有 v/ω 组合满足转向角与转向速率约束，该限制仍需保留。不得追溯改判旧回合，或宣称此因素已经解释 N04 失败。

复算命令：

`/usr/bin/python3 artifacts/astra_gazebo_20260926/audits/baseline_tuning_b01/reference_frame_addendum/reproduce.py`

`synthetic_reference_cases.json` 是源码支持的离线数学反例，不是 Gazebo 数据。六组转弯/窗口情形及四种姿态下两组完整 footprint 的世界坐标不变性均由脚本计算；源文件精确 SHA 和行区间见 report.json。两个最初只读路径猜测错误也在该 JSON 保留。
