# V3-SCOPE V3-02 可行干预预测器终态交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`v3_02_d2_independent_heldout_review_pass`

## 1. 最终结论

V3-02-D1/D2 已解决共同前缀模型在规划器接入前的最后一个动作域缺口：
观测时刻后的 ego 分支现在由固定生成器施加 0.60 m/s² 最大加速和
0.80 m/s² 最大减速，并把 certified stop（目标速度 0）纳入 train/cal/test
支持域。D2 在 fresh seeds 6281--6283 上一次性完成 45/45 episode，通过全部
数据完整性、运动可行性、stop、覆盖率、宽度、误差和候选敏感性门。

这证明的是 parameterized-crossing 范围内、固定直线加速度受限生成器上的
candidate-conditioned prediction。它允许 V3-03 使用同一个候选生成器设计 hard
verifier，但不构成导航安全、跨场景泛化、性能提升或实车资格。

## 2. D1 数据与校准

- train：6 blocks × 3 policies × 5 candidates = 90 episode；
- calibration：3 blocks × 3 policies × 5 candidates = 45 episode；
- candidates：certified stop、0.6、0.8、1.0、1.2 m/s；
- common prefix：`t=0--0.5 s`、0.8 m/s；
- branch：目标速度由固定 0.60/0.80 m/s² 加/减速率达到；
- 模型：3 mode × 4 horizon complete-quadratic ridge；
- conformal score：actual-mode absolute residual；
- conformal radius：0.379024806626 m。

D1 数据资格：

- 135/135 episode；
- 最大 ego 轨迹实现误差：`3.34e-10 m`；
- 最大计划加速/减速：0.60/0.80 m/s²；
- stop terminal position error：0；
- stop 在绝对时刻 1.5 s 后的位移：0；
- 共同前缀 actor coordinate/speed spread：0/0。

D1 calibration：

- coverage：1.0；
- mean full width：3.8764017 m；
- actual-mode MAE：0.1915476 m；
- minimum candidate sensitivity：0.7501869 m；
- 全部门通过，held-out 未访问。

## 3. D2 正式留出结果

- 45/45 episode，180 points，attempt=1，retry=0；
- marginal coverage：1.0（门 0.90）；
- per-horizon coverage：1.0 / 1.0 / 1.0 / 1.0（门 0.80）；
- mean full envelope width：3.8778909 m（门 4.50 m）；
- actual-mode MAE：0.1897249 m（门 0.50 m）；
- minimum candidate sensitivity：0.7379160 m（门 0.20 m）；
- physical coordinate violation：0；
- 最大共同前缀 actor coordinate/speed spread：0/0；
- 最大 ego trace error：`3.34e-10 m`；
- 最大计划加速/减速：0.60/0.80 m/s²；
- stop terminal error / post-stop motion：0/0；
- hidden policy runtime input：false。

## 4. 核心 SHA-256

| 资源 | SHA-256 |
|---|---|
| D1 frozen model | `fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7` |
| D1 calibration report | `90b9b57c7204b3391012a467f458c9951afe29f4fbd02283ee9e605ef61d447a` |
| D1 model freeze | `01f01994d0c47dad603f9c77663710a791b71feab09524b43b85ece81b3428d0` |
| D1 independent model review | `df0c56635b76fe6a22d3c3aa6c832cf00c9811073d9b08a3b8de9c90d1023255` |
| D2 release | `72b66ec63d0b570fe6a2a8cb1a2b7851bc0b6d4dbb7cb4b494aa4dfe9770c8c4` |
| D2 release review | `0bf7395e8feac3696d1bfd11c7e3b492b527f6e8afc788c1877aa7186e516de5` |
| D2 assessment | `05f6b9a17aabdbd1aa7b9da732fe1fcd3075c8f7b7c090bb5be4279b43e8770c` |
| D2 evidence freeze | `62fabc7507264e6a2d0a5e31fbae8a8df1aece57da6f58e97caf9dc979ca3343` |
| D2 independent result review | `f0fd7d3908015851820c4512f868a9a5944919612c8f826c914cba309f3fa001` |

## 5. 权限与下一入口

D1/D2 已终结；不得重跑、续跑、复用 seeds/budget、测试后更新模型或放宽门槛。
没有运行 move_base、RL、CAN/串口或实车。

下一入口是 V3-03 独立纯离线 hard verifier / analytic ranker：

1. 候选必须复用冻结的可行生成器身份和同一 scene snapshot；
2. calibrated actor interval 只作为概率占据输入，不能等同绝对安全；
3. verifier 还必须独立检查 ego footprint、tracking margin、制动可达、health、
   timestamp、candidate digest 和 timeout；
4. certified stop 必须始终存在，输入缺失/过期/非法时 fail closed；
5. ranker 只能在 verifier 接受集内选择。
