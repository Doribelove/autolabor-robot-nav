# M12 物理执行后端（设计，尚未 live 资格）

采用独立 `astra_physical_ackermann_plugin`：每个 ODE physics step 以有限轮轴力矩和转向 PD 力矩驱动车轮/转向关节，物理引擎决定车体运动；不调用 SetWorldPose/SetPosition/SetVelocity 或清零车体速度。保留 `/cmd_vel_navigation -> relay -> /cmd_vel`、有限速度/转角参考变化、command delay、timeout、brake/emergency 接口。新插件、新 xacro、新 spawn launch 对所有方法共用，旧驱动/Q01–Q03 文件不改。

Gazebo 官方区分直接设置瞬时关节速度、ODE 有限力矩马达以及施加力矩的 PID；本实现选择最后一种，初始增益是待资格的仿真候选，不是实车标定。[Gazebo 关节控制](https://classic.gazebosim.org/tutorials?tut=set_velocity)。当前0.002秒ODE步长保持，controller每步运行，actual pose/twist/joints在WorldUpdateEnd发布；50Hz只作为遥测率。

物理模型继承原 body/四轮/steer/sensors 几何与惯量。展开URDF实际总质量79.6kg（chassis66+4轮12+2steer1+laser.5+imu.1），不是属性名义80kg；中立姿态COM约(0.004083,0,0.252638)m。轴距.65、轮距.60、轮半径.15、轮宽.08；尺寸质量为simulation_candidates.yaml注明的datasheet/示例/占位来源，全部calibrated=false。当前轮胎mu1=mu2=1未实测。不得为修复0.4kg差额静默更改旧模型。

轮速度目标必须区分前后轮；前轮应包含绕后轴转弯时的L*yaw_rate侧向分量。初版保持原驱动对四轮施加驱动的模拟结构，但改为有限独立扭矩；真实车辆驱动拓扑尚未核实。轮 PI、转向 PD 有限增益/积分/输出上限，所有参数显式写入新xacro和metadata；URDF原500/300Nm关节限位只作更高层硬上限，不能当成已标定电机能力。速度参考的.6/.9m/s²及虚拟center steering的.75rad/s均保留；actual加速度/actual steering不直接等于参考，必须通过新的实际physics测试资格，不能把有界参考冒充实际物理边界。

重要参考点：base_link在轴间中点，真实无侧滑关系为body vy=(L/2)*yaw_rate。`cmd.linear.x`继续表示body forward vx，`angular.z`转steer公式仍用atan(L*w/vx)，零速不支持原地转向。odom直接发布actual base_link位姿及body twist，包括真实vy；不得强行抹掉vy来匹配旧中点bicycle积分。rear-axle候选/reference adapter由root另行实现并测试。

摩擦实验使用合同mu=.4/.7/1、执行delay=0/.05/.1/.2秒、brake=.9/.7/.5m/s²。常规.6/.9远低于mu=.4对应3.924m/s²牵引上限，可能没有显著mu效应，不能因此调低mu或抬高正式速度制造阳性。先在隔离机械资格场地采用模型允许的2.6m/s与center steer.40rad，横向需求约4.397m/s²，区分.4与.7；此fixture不是正式核心导航样本。Gazebo取两个接触表面的较小mu，所以必须记录ground与wheel双方实际surface参数。[Gazebo 摩擦](https://classic.gazebosim.org/tutorials?tut=friction)。接触求解、步长和摩擦方向还需yaw/step-size敏感性检查。[ODE参数](https://classic.gazebosim.org/tutorials?tut=physics_params)。

资格顺序：离线joint/inertia/geometry一致性与torque/reference tests；静止支撑与真实关节速度；前进/反向直线阶跃和常速；左右恒曲率及base/rear参考转换；实际steer slew；coast/brake/emergency/timeout/NaN；mu可辨识fixture；原合同delay/brake敏感性；新共同map/globalroute与全部方法小矩阵。每项使用fresh identity，保留失败；backend未通过实际跟踪/接触/连续geometry门之前不跑core D01，不与旧pose-set结果混池。ODE连续运动不等于形式连续安全，接触冲量/离散采样间隙仍需独立包络验证。
