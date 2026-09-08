# 导航优化专用候选版约束（2026-09-07）

2026-09-08 22:14 当前清扫停滞只读审计：本次操作员初值已设置，LOCALIZED，不能再
沿用下方 WAITING_INITIAL_POSE。当前计划 a902200d614d4f0cbe9dd285f59208fc，第 8/8 段，
79.9652%，active=true、paused=false；SWEEPING 和短暂 WAITING_OBSTACLE 循环，指令及
双轮零速，底盘/避障就绪。22:06:06 起全局规划器反复拒绝强制清扫线，非入口/4 米门。
实时地图复算命中前方约 15.126 m 的单个致命栅格（16.75,-26.95），原静态图为空地。
整条剩余约 19.1 m 清扫线碰撞检查导致入口立即拒绝；实际清除射线范围 11 m，未确定
远端障碍记录是否残留或现场实物。已截图、保存地图/日志，未清图、改控制或暂停/取消。
运行版仍 20260908_214006，主门 true/FOD=false/1.60 上限保留。证据与运行边界见
docs/SWEEP_BLOCKED_AUDIT_20260908.md、validation/sweep_blocked_audit_latest.txt。

2026-09-08 21:45 部署启动最新：用户要求“现在部署，进行实验”。已通过视觉会话的
stop.request 正常关闭 Qt/相机与其私有 BPU 服务（返回 0），主导航此前已统一停净。
本地已验证代码通过候选部署入口原生发布为 J6M `20260908_214006`，板端静态检查
通过，管理器与 coverage.yaml 两端 SHA256 一致。带原静态地图完整启动返回 0，
严格 runtime 通过；active/MainPID=1092384，管理器 PID=824590，Qt 地图 READY。
运行参数 4.00 m、入口 0.40 m/25°、规划 0.30 m/20° 已核对，新触发代码生效。
当前为本次冷启动的 WAITING_INITIAL_POSE，需要操作员按真实位置设置新初值。
覆盖 IDLE/无活动批次，急停反馈已解除、底盘/避障就绪；30 秒最终指令 1483 条、
双轮各 1507 条全部零速。启动期记录 2 次 VCU ControlTimeout 和急停状态，后续
30 秒未复现控制超时。完整归档另见 21:43:38–43 的双轮非零反馈（最高
0.458/0.433 m/s），以及零星 ±0.01237 m/s 反馈，最后一次为 21:46:15；全会话
ROS 最终指令均零，来源尚未确认，不能把上述 30 秒静止窗口推广到整个会话。
当前双轮已归零。等待初值期间 move_base/FOD 的 TF 告警仍存在，不能宣称
定位或实车恢复已验收。当前沿用操作员手动实验，ABC 自动控制 STOP 保留。
被动记录器恢复于 `validation/coverage_live_20260908/20260908_214201/`，
部署证据见 `validation/entry_recovery_deploy_latest.txt`。主门 true、FOD=false、
1.60 上限与配置/授权均保留；助手未发送初值、运动目标或非零速度。未提交或推送。

2026-09-08 入口恢复触发修复（本地未发布）：用户要求恢复 4 米上限，只收紧触发，
且不得影响入口合格后进入清扫。配置和默认值现为 4.00 m；不再因普通转场失败距入口
2 m 内或单独清扫前入口超差就启用特殊恢复。必须有最终接近段可信连续记录、有向进度
达到 85%、沿实际行进方向越过原入口平面至少 0.02 m、当前在原 0.40 m/25° 容差之外
且距入口不超过 2 m，才应用原恢复代价及 4 m 门。每次规划前重判并清除过期标记；
入口合格仍直接进入原停车/清扫交接，不新增 85% 或越界门槛。原代价数值与运动保护不变。
2 项旧代码误触发断言复现，修复后 149 项状态机、193 项覆盖包 Python 测试通过；
本机 Release 构建和静态检查通过。J6M current 只读确认仍为
`20260908_192331`（10 m 旧触发版），未部署、未启动导航、未发运动目标。
导航 inactive/MainPID=0，独立视觉会话保留。未提交或推送；详见
`docs/ENTRY_RECOVERY_TRIGGER_FIX_20260908.md` 和 `validation/entry_recovery_trigger_latest.txt`。

2026-09-08 21:08 最新：用户要求只打开本版 Qt 视觉识别。已通过统一入口完整停止导航
（退出 0），被动覆盖记录器随原服务退出；随后在本机独立 ROS master
`http://127.0.0.1:11571` 启动 ZED QUALITY RGB-D、FCOS 几何筛选和 Qt BPU 预览。
导航、定位、雷达、覆盖、底盘及 CenterPoint ROS 节点均未启动；ZED 跟踪及 RViz 关闭。
25 秒观察收到 373 帧有效且已标定结果，约 14.91 Hz，未收到速度、目标或初值消息。
当前场景保留不确定框，不能据此宣称垃圾精度验收通过。窗口持续运行，无演示截止；
关闭 Qt 会结束本次相机、识别及私有 BPU 会话。主门 true、FOD=false、1.60 m/s 和
授权标记保留，J6M current 仍 `20260908_192331`。未构建、部署或提交；原有未提交
导航修改保留。完整导航会话已结束，后续重启导航需新会话真实初值，不能沿用旧定位。
运行记录和停止方式见 `docs/VISION_ONLY_SESSION_20260908.md`。

2026-09-08 后续只读转场审计：用户手动任务 `8c3ae04f72e84a73b6277e6b0e0aea3b`
在 19:50–19:51 的第 2→3 条线转场失败，停于第 5/8 段、56.6495%。11 次 Hybrid
请求均在 6×6 m 窗口成功，最长约 0.757 s；包含获准执行的 6.90 m 全倒车大圆弧，
取消前定位反馈累计约 1.779 m，横偏 0.357 m 后重规划，再因低进展耗尽重试暂停。
两搜索线程只区分启发权重，并无前/倒车起步分工；日志未保存选中 lane。
发现短 cusp 的 SUCCEEDED 终态直接返回、未复核有向进度的代码缺口，与 0.30 m
小段仅挪约 0.07–0.08 m 就报告到达、随后无法接回后缀的现象吻合，尚未修复。
当前恢复代价按 F/R 0.20/0.80 m/s 计算、实际倒车上限 0.30；候选选择优先换向次数。
详细时间线、证据及边界见 `docs/TRANSITION_REVERSE_PATH_AUDIT_20260908.md`。
本次仅只读查询和保存分析，未改控制/参数或恢复任务，自动实验 STOP 仍保留。

2026-09-08 19:40 最新：用户明确要求“解除暂停，恢复手动实验”。操作员已提交本次
真实初值，实时 LOCALIZED（overlap=0.9992、RMSE=0.1645、inliers=4854），无需再次
设置本会话初值。底盘/避障就绪，无活动任务、无保留目标；助手已调用导航暂停服务
false，返回 `navigation resumed without a retained target`，导航与覆盖 paused 均
确认 false。恢复瞬间覆盖 IDLE、无批次，指令/双轮零速；此后由操作员手动实验。
入口恢复总长运行值仍 10.0 m，release `20260908_192331` 不变。后台被动记录器继续
运行，自动多轮实验 STOP 标记保留；不据此恢复助手自动巡扫或重发旧目标。
证据 `validation/manual_experiment_resume_20260908_194029.json`。本次仅解除暂停并记录，
未重启、构建、部署或发送初值/运动目标，保留既有运动授权和速度上限。

2026-09-08 19:30 最新：按用户要求，将入口恢复路径总长从 4.00 m 调整为 10.00 m，
配置及两处默认值已同步；入口容差、恢复分类、速度和其他保护不变。140 项状态机
测试（含 10 m 接受/超限拒绝）通过，本机 Release 构建、静态检查、J6M 原生发布与
板端静态检查通过。current 已更新为 `20260908_192331`，配置和管理器两端 SHA256
一致。统一停机、带原静态地图冷启动均退出 0，runtime 检查通过；候选 active，
MainPID=523119，覆盖管理器 J6M PID=781319，Qt 地图 READY。
19:29:36–19:30:36 静止观察中，实际参数前后均 10.0；3022 条最终指令、双轮各
3067 条反馈全零，覆盖 IDLE/无活动批次、导航暂停 true，传感器持续更新。
当前新会话为 WAITING_INITIAL_POSE；须由操作员在下次实验前重新设置真实初值，
不能沿用 17:33:51 的旧会话初值。初值前 move_base 尚无 status 发布、记录 12 条
等待 map→base_link TF 的告警属该阶段预期，不能宣称已完成定位/导航移动稳定性验收。
用户要求读取和运行稳定后告知再开始实验，本轮仍保持暂停，ABC STOP 标记保留，
没有发送初值、目标或非零速度。主门 true、FOD=false、1.60 m/s 和授权标记保留。
被动记录器已恢复至 `validation/coverage_live_20260908/20260908_192709/`。
变更、验证及当前生效状态见 `docs/ENTRY_RECOVERY_LIMIT_20260908.md`，证据路径见
`validation/entry_recovery_10m_latest.txt`。未提交或推送。

2026-09-08 18:13 最新：用户因下课高峰期明确暂停实验，并要求在切换网络前保存
实验进度与终端数据。18:09 已设导航暂停 true，A 批次此前已按精确 ID 取消；
18:13 连续 15 秒确认无活动目标，730 条最终指令、双轮各 755 条反馈全零。
保留主门 true、FOD=false、1.60 m/s 和授权标记，完整栈及被动记录器留存运行。
`validation/abc_trials_20260908_1752/STOP` 保留；未经用户明确继续，不得自动恢复
实验、发送目标或位置调整。切换网络后的运行/连接状态须重新实测。
第一轮 A 在 27.5172% 的第 3/8 段转场复现恢复路径 4.20 m 超过 4.00 m 门限，
已取消，B/C 与提速对比尚未执行，辅助 reposition.py 未运行，尚无最佳参数结论。
本次 17:27 启动会话已有操作员真实初值，不重复索取；如后来完整冷启动则须新初值。
交接见 `docs/EXPERIMENT_HANDOFF_20260908_PAUSED.md`，固定归档位置见
`validation/network_switch_20260908_latest.txt`。本轮未构建、部署或提交。

2026-09-08 17:51 多区域实车实验授权：用户明确将底盘控制权限交给助手，授权对 Qt
新保存的 A、B、C 反复进行多轮实验，包括轮流清扫、约半程取消后转另一区域；突发
行人情况由现场操作员告知。随后追加重点关注转场停滞及覆盖管理器拒绝运动，并允许
卡住时经受保护控制入口调整至下一条轨迹附近，研究入口恢复路径总长上限。
沿用已确认的封闭测试区/实体急停/1.60 m/s 最终上限，主门 true、FOD=false 保留。
助手可在本次授权范围内发起任务，不得再把下方历史“助手未发目标”当作禁止本轮执行。
本轮先以清扫/转场前后向不超过 0.30 m/s 验证；保留定位、避障、运动门、唯一最终
速度发布者和完整任务所有权，不允许绕过保护直接发布第二路最终 cmd_vel。
每次人工式位置调整、任务取消和暂停必须记录，不能当作无人干预成功。实验定义及
控制证据位于 `validation/abc_trials_20260908_1752/`，原被动记录器继续运行。

2026-09-08 16:56 主运动授权最新：用户明确要求“开启主运动授权”。沿用已记录的封闭净空
测试区、人员远离、实体急停可用与 1.60 m/s 上限确认；当前 CAN 固定端口已确认。
预检 LOCALIZED、底盘及避障就绪、无活动目标、指令及双轮零速。统一停机退出 0 后，
通过本候选授权入口创建 `runtime/motion_authorized.ok`，私有配置仅将 MOTION_ENABLED
改为 true；FOD=false、导航 0.80/倒车 0.30 和最终 1.60 上限不变。
带原静态地图完整冷启动退出 0，严格 runtime 通过；active/MainPID=980973、Qt PID=988649，
地图 READY。实际网关参数 motion_enabled:=true，看门狗 true/1.60/0.25 秒，J6M 配置
同步为主门 true/FOD=false；current 仍 `20260907_214928`，本次未构建或重新部署。
16:56 快照为新会话 WAITING_INITIAL_POSE、覆盖 IDLE/无计划；需操作员按真实位置重新
设置初值，不能复用上次。新监测会话累计 5446 条最终指令、双轮各 5544 条全零。
助手未发送初值、目标或非零速度，不自动恢复旧任务。不要在收尾撤销此次授权或按下方
历史记录恢复 false。后台监测已恢复于 `validation/coverage_live_20260908/20260908_165422/`。
配置备份、运行证据和范围见 `MOTION_AUTHORIZATION_20260908.md`。

2026-09-08 16:36 最新：用户要求停止正在运行的 GNSS 版、保存其进度，再启动本目录。
GNSS 版已通过其统一入口完整停机（退出 0），源码与文档保存到其原本地分支，提交
`9603e23`，工作树干净、未推送；私有配置和运行资产仍留本地。
本目录仍为 `feature/litter-perception-optimization-20260908`，统一入口带
`map_20260829_221335_gimp_cleaned` 启动退出 0，严格 runtime 检查通过；候选
active/MainPID=880803，Qt 地图 READY（1311x670、0.10 m）。J6M 与 MID360 网络正常，
FCOS 筛选状态及 CenterPoint 观察状态持续更新。本次未重新构建或部署，导航 J6M current
仍 `20260907_214928`，GNSS 独立 current 仍 `20260908_143728`。
16:35:43 起约 20 秒观测：990 条最终指令、左右轮 809/781 条反馈全零，覆盖 inactive；
定位全部 WAITING_INITIAL_POSE，需操作员在本次 Qt 按真实位置设置初值，不能复用旧值。
两运动门保持 false，未创建授权、发送初值、目标或非零速度；程序启动不等于实车移动验收。
证据 `validation/start_navigation_20260908_163602_runtime.json`。此条取代下方停止交付状态。

2026-09-08 垃圾误框优化候选：当前本地分支为
`feature/litter-perception-optimization-20260908`，从导航验收提交 `99e8ac9` 分出。
本轮用户要求本地保存、次日亲自启动验收；不启动实车、不部署 J6M、不改变运动配置。
AGX FCOS RGB-D 后处理新增类别无关的地面/前景几何及连续确认，Qt 默认显示筛选结果，
保留橙色不确定框，并可切换全部原始框。原始 `/perception/fcos/objects` 保留，新增候选与
逐框审计 topic 全部仅观察；没有迁移垃圾专用权重或接入自动清扫。
J6M 只读确认 current 仍 `20260907_214928`；原导航分支及其他工作树保留原提交。
当前目录会留在新分支，本机产物供此次验收使用；停止状态切回旧分支后必须重新构建，
不能让旧源码与新 Qt 产物混用。原因、验证与启动/对照步骤见
`docs/LITTER_PERCEPTION_OPTIMIZATION_20260908.md`。下方为此前导航交接历史。

本日收工保存：用户要求将当前全部进度保存到 Git 分支，供明日验收；当前分支为
`feature/navigation-optimization-20260907`。验收流程与证据归档说明见
`docs/ACCEPTANCE_HANDOFF_20260908.md`。此保存操作不启动程序、不改变现场运动设置。

2026-09-07 22:46 仿真验证最新：按用户要求，首入口修复已完成隔离 ROS 场景验证，15 项
集成检查通过。独立网络/PID namespace 只含 lo；运行真实 move_base、Navfn、TEB 插件、
覆盖管理器和导航所有权桥，用合成地图/TF/状态输入。旧 release 20260907_212100 的
管理器生成 307 点后约 7 ms 被空消息覆盖，晚订阅者为 0 点；修复版连续 3 次保留
307 点，绕墙场景保留 462 点/23.272 m。取消、换图、新批次替换、规划中取消和 Navfn
封闭无路检查通过，0 个 move_base/Hybrid goal、没有 cmd_vel 消息。仿真进程已全部结束。
本轮新增仿真工具和原因报告，未改业务代码、未重新部署或启动实车。候选 current 仍为
20260907_214928；用户主动断开底盘及雷达供电，实车服务继续停止。此结果不等于 Qt/Ogre
实际渲染、TEB 行驶、J6M Hybrid CPU 或实车验收。详见 `FIRST_ENTRY_SIMULATION_20260907.md`。

本轮随后现场确认：用户已主动断开小车底盘供电，雷达因此无电，要求先汇报工作进度。
这解释了下述 MID360 无载波，不能继续误报为待查网线故障。停止恢复启动和待连接的监控器；
当前程序保持停止，修复已部署但实机回归未完成。待用户恢复供电并继续实验后再统一启动。

2026-09-07 22:08 最新：用户覆盖实验复现首入口预览仍为空。批次因主运动门 false
失败时，`_record_batch_region_result` 先清掉路径和 plan_map_digest，导致后续 finalizer
保留逻辑失效，上一轮修复未闭环。本轮已在两级清理中统一保留判定，预览独立记录地图摘要
和批次令牌；取消、跳过、执行过、地图变化和旧批次均不保留。先复现再修复的回归通过；
206 项针对性测试、AGX Release 构建、静态检查、J6M 原生部署及板端静态检查通过。
候选 current=`20260907_214928`，板端管理器与本地 SHA256 一致。
完整双机栈已统一停净，但带原静态地图恢复启动退出 1：MID360 网卡
`50:54:7B:E3:C9:10`（本次 eth0）无物理载波，192.168.1.112 不通；J6M 双向通信正常。
当前原版/候选均 inactive、MainPID=0，J6M 11311 无监听、候选 PID 记录空。已请求现场
检查雷达供电与网线；不要称程序已恢复。链路恢复后完整冷启动，再由操作员给真实新初值。
两个运动门仍 false，助手未发送初值、目标或非零速度。21:39:44–21:44:37 实验快照最终
命令 14149 条、左右轮 14625/14622 条全零；没有 Hybrid action，CPU 仅为空闲基线。
另捕获 7 条 VCU ControlTimeout 日志，后续 60 秒未复现，根因未定，不能称无故障。
详见 `COVERAGE_CPU_EXPERIMENT_20260907.md` 与 `validation/coverage_live_20260907/`。

2026-09-07 21:27 首线入场预览修复最新：用户要求修复“当前位置到第一条清扫线入口没有
规划路径”。根因是旧实现只在运动安全预检通过后提交首个 move_base 目标，主运动门为 false
时预检提前拒绝，因此 Navfn 从未运行，Qt 也没有独立的真实入场路径 topic。候选现增加
`/coverage/first_entry_path`：覆盖几何和实时位姿确定后，通过插件只读
`/move_base/CoverageGlobalPlanner/make_navfn_plan` 查询 Navfn，校验 map 坐标及首尾位置，
Qt 以深蓝色常显；执行前失败时保留供检查，新任务、地图变化或取消会清空。该服务不发送
action 目标。插件内所有 Navfn 调用现由同一互斥锁串行，避免预览、move_base 规划和 Hybrid
拓扑查询并发访问同一 NavfnROS 实例。AGX Release 构建、204 项针对性测试、283 项汇总检查、
J6M ARM64 原生部署和板端静态健康均通过；J6M 候选 release 为 `20260907_212100`。统一入口
带原静态地图冷启动返回 0，服务 active/MainPID=2724377，Qt PID=2731384，地图 READY。
当前新会话为 WAITING_INITIAL_POSE；move_base 插件及其预览服务会在操作员按真实位置设置
初值后初始化，尚缺同一区域的真实首入口路径回归。不要复用旧初值。`MOTION_ENABLED=false`、
`FOD_MOTION_ENABLED=false`、候选授权标记不存在，10 条最终速度全零；助手未发送初值、目标
或非零速度。详见 `COVERAGE_CPU_EXPERIMENT_20260907.md`。

2026-09-07 Qt 闪退修复最新：用户已授权修复，已在本候选完成实现、构建与完整冷启动。
实际 GUI 回归先复现全局 AsyncSpinner 抢走 RViz GUI 队列回调；已改私有业务队列。
真实 Ogre 压测又复现删除后第二次建窗的纹理错误/SIGSEGV，现 ROS master 恢复仅重置
显示缓存并重新注册本进程 ROS 端点，保留同一原生窗口与 manager。ZED 项目启动入口
同时关闭位置跟踪和会隐式开启跟踪的深度稳定，QUALITY RGB-D 保留。283 项汇总测试、
1000 次实际页签切换与 5 次真实渲染重连均通过，未改故障级联停机或运动安全策略。
共享 robot_bringup 已随 J6M 原生 release `20260907_191248` 发布，板端静态健康通过。
统一入口带原静态地图启动与严格 runtime 均返回 0；候选 active/MainPID=2161491，
Qt PID=2169173、地图 READY（1311x670、0.10 m），ZED PID=2167341、诊断跟踪 INACTIVE。
19:16:13 起 60 秒观测 faults=[]，2907 条最终速度及左右轮反馈全零。当前新会话仍
WAITING_INITIAL_POSE，需要操作员按真实位置设置初值；不要复用旧初值。两个运动门
保持 false，候选授权标记仍不存在，助手未发真实初值、目标或非零速度。麦克风仍未枚举。
原工作区未改，CenterPoint 仍只观察。详见 `QT_CRASH_FIX_20260907.md`，短时通过不等于
小时级老化、ToDesk 热重启/显示热插拔或实车移动验收。

2026-09-07 17:48 最新：用户准备继续实验时 Qt 消失。只读日志确认不是 Qt/Ogre 自身断言：
ZED 先报告位置跟踪发散并重置里程计，随后必需节点 `/zed2/zed_node` 以 `-11`（SIGSEGV）退出；
其 roslaunch 关闭了视觉链，UI 伴随进程也随生命周期退出。已按统一入口带原静态地图做完整冷重启，
不是单独拉起 Qt；命令退出码 0，服务 active/MainPID=1667551，195 项检查和严格 runtime 检查
均通过，Qt 地图 `READY`（1311x670、0.10 m），ZED、正式检测、双 BPU 感知、导航与覆盖节点
均在线。新会话当前 `WAITING_INITIAL_POSE`，须由操作员按真实位置重新设置初值；不要复用旧会话
初值。`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`，watchdog inactive，10 条最终
`/cmd_vel` 样本全零。助手未发送初值、目标或非零速度。J6M `current` 仍为
`20260907_170157`，本轮没有重新构建或部署。

2026-09-07 17:10 最新：用户要求打开本目录的静态地图完整功能 Qt；MID360 物理链路恢复后，
已修复独立候选遗漏的 BPU 私有路径迁移和 `live_protocol.py` 运行依赖。旧候选遗留 BPU worker
仅按其 `service_manager.py` 的 PID/启动身份记录精确停止；旧工作区源码未修改。AGX 全量 Release
构建、BPU 包 21 项测试、195 项静态检查、J6M 原生部署及板端静态健康检查通过；J6M `current`
现为 release `20260907_170157`。候选统一入口带
`map_20260829_221335_gimp_cleaned` 启动返回 0，服务 active/MainPID=1443773，Qt 地图
`READY`（1311x670、0.10 m），严格 runtime 健康检查通过；FCOS BPU 结果约 7--10 Hz、
CenterPoint 约 5 Hz。当前是本次冷启动的 `WAITING_INITIAL_POSE`，必须由操作员按车辆真实位置
设置初值；不要复用历史初值。`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`、候选运动
授权标记仍不存在，10 条最终 `/cmd_vel` 样本全零；助手未发送初值、目标或非零速度。当前未枚举
实体麦克风，Qt/AI 节点在线但语音采集不可用。CenterPoint 仍仅候选观察，不接入导航控制。

2026-09-07 最新：本目录是独立工作树
`/home/slam/robot_j6m_ws_navigation_20260907`，分支
`feature/navigation-optimization-20260907`。开工时完整保留了
`robot_j6m_ws_optimized_20260905` 的 tracked/untracked 源码修改，并补齐当前地图、私有配置、
ASR 与 CLIP 运行资产；原工作区未清理、重置或修改。覆盖转场 Hybrid A* 已改为可取消 action，
每层两路并行搜索，按 6x6 m、10x10 m、全图逐级回退，滚动/恢复总预算 5 s。AGX 全量 Release
编译通过；覆盖包 187 项测试、全局汇总 195 项和隔离生命周期 10 项均零失败。J6M 已在独立
ARM64 chroot 原生构建并部署 release `20260907_141331`，其独立 `current` 已通过静态健康检查。
部署后原/新导航均停止，11311 无监听，`MOTION_ENABLED=false`、`FOD_MOTION_ENABLED=false`，
没有复制运动授权标记、发送目标、发送非零速度或做实车移动测试。详见
`NAVIGATION_OPTIMIZATION_20260907.md`；一键启停以 README 顶部命令为准。

2026-09-06 19:36 最新：已修复已知地图 localizer 在人工初值附近被 5 m 粗 ICP 拉入
重复走廊错误极值并持续 LOST 的问题；改为 0.50 m 局部精 ICP 优先，原粗到精路径仅作
质量门失败后的回退。J6M current 已部署为 20260906_190656，实机进程和运行参数归属已
核对。操作员在冷启动后重新设置真实初值，当前 LOCALIZED、Qt 地图 READY，无活动目标，
最终指令与双轮反馈为零。固定同一人工初值完成 5 个完整组、每组 12 次重定位，5/5 组
均满足两轴绝对偏差 P75 <= 0.05 m。按追加试验前固定的最差轴最小规则，第4组最佳：
X/Y P75=0.01442/0.02330 m；全部60次合并 P75=0.01289/0.03592 m。该结果只证明
重复定位一致性，没有外部真值，不能写成绝对地图坐标精度。操作员说明此前压力试验期间
有人走动，那两组数据不纳入完整组；另有一次旧24秒窗口内中止尝试，也未拼接进完整组。
详见 `FAST_LIO_REPEATABILITY_20260906.md`。本轮未发送导航目标或非零速度，主运动门
true、FOD=false 和既有授权标记保持不变。

2026-09-06 17:12 Qt崩溃后重启最新：用户要求再次实验，已通过候选统一入口带原静态地图
完整冷重启，退出码0、997项检查及runtime健康检查通过。候选服务active/MainPID=1261687，
Qt PID=1266732且未再出现Ogre断言，原版inactive；J6M进程仍来自release
20260906_152402。地图READY，操作员已在新会话设置真实初值，最新LOCALIZED，overlap=0.9846、
RMSE=0.1786、inliers=4339；不要再要求重复本次初值。主运动true、FOD=false、1.60上限和
已有授权保留。本轮用户在Qt先发起28.00 m²批次任务；因其与初值提交时序交叠，助手只调用
`/coverage/set_paused=true`作短暂停车核对，确认指令/轮速归零。随后操作员从Qt新建了不同的
61.49 m²计划9f6498e3，非旧任务倒退；17:12实测其TRANSITING、5/6段、进度66.21%，车辆
正在执行，不能沿用此前零速状态。助手未发送初值、目标或非零速度。本轮没有改业务代码、
构建或部署；第二条CenterPoint仍为候选观察，未接入正式导航控制。详见
FULL_PROJECT_START_20260906.md 的17:12追加记录。

2026-09-06 16:34 完整重启最新：用户暂停第二条链路开发后，单独要求重启整个程序。
已通过候选统一 restart 入口带原静态地图完整冷重启，退出码0、运行态检查通过。
候选 active/MainPID=1131832，Qt PID=1136854，原版 inactive；原生双模型 BPU 新PID=46646，
J6M current 仍20260906_152402。现场已在新会话设置初值，最新 LOCALIZED，overlap=0.9998、
RMSE=0.1704、inliers=4462；不要再次要求本次已完成的初值。地图READY，20秒最终指令与两轮
反馈全零、faults=[]、末次无导航目标。主运动true、FOD=false、1.60上限和已有授权保留。
本次场景FCOS RGB-D约13.10 Hz、CenterPoint约4.99 Hz；这不是此前多框瓶颈的修复或性能保证。
第二条导航接入及Qt三维框开发仍暂停，CenterPoint仍候选观察、检测为0框。本轮未改业务代码、
构建或部署，未由助手发送初值/目标/非零速度。详见 FULL_PROJECT_START_20260906.md 的重启追加记录。

2026-09-06 15:53 完整启动最新：用户明确要求启动完整项目，并确认MID360已接线供电；
链路已恢复，经候选统一入口带原静态地图启动成功。候选服务active，MainPID=919104，
Qt PID=927465，原版inactive，J6M current仍20260906_152402。地图READY，当前本次冷启动
仍WAITING_INITIAL_POSE，未由助手发送初值/目标/非零速度。20秒最终指令/轮速全零，faults=[]。
主运动运行态true、上限1.60、FOD=false，已有授权保留。FCOS RGB-D、CenterPoint和原正式后端
均运行，J6M原生BPU服务PID=10033；真实MID360去畸变点云约10 Hz、CenterPoint约5 Hz。
全栈多框场景下FCOS有效深度输出只有约1.7–2.4 Hz，源帧龄接近0.35 s导致大量过期丢弃；
不能用下方独立测试10–14 Hz代表当前完整项目性能。CenterPoint仍仅候选观察，精度未验收。
本次只启动与被动检查，没有改业务代码/配置、构建或部署。项目保持运行，详见
FULL_PROJECT_START_20260906.md。本条取代下方“网口未接/全部停止”的历史交付状态。

2026-09-06 双感知接入最新：用户已确认 ZED 左目相对 MID360 前0.42 m、右0.14 m、等高，
光轴水平向下41°，无偏航/侧倾。先完成 GitHub 修改前备份分支和标签（e0cbf76），
本地 runtime/backups/before_dual_perception_20260906 保留独立解压副本、归档和 bundle；
2531个文件校验一致。已部署新 J6M ROS current=20260906_152402，新增 FCOS RGB-D 与
CenterPoint ROS 包、独立双模型常驻服务。候选 BPU_PERCEPTION_ENABLED=true，随下次正常
UI 启动伴随链路。主运动 true、FOD=false、1.60上限和已有授权标记保留。
真实 QUALITY 深度+Qt+合成 CenterPoint 并行实测 FCOS约10–14 Hz；997项测试通过。
MID360现场网口无载波、不可达，真实点云验收未完成，不能把263帧合成ROS数据写成MID360实测。
CenterPoint仅候选观察；发现旧TEB自定义障碍物保留旧帧及TF失败单位变换问题，本版明确拒绝
开启学习型障碍物控制入口。COCO/nuScenes不是五材质垃圾模型，正式后端未替换，未验收GPU节省。
本轮没有启动完整导航、发送初值/目标/速度；最后相机、Qt、远端测试节点和私有BPU服务均退出，
原版/候选导航服务均inactive，无当前运行态LOCALIZED/watchdog结论。详见 DUAL_PERCEPTION_20260906.md。
此记录优先于下方历史存活状态，不自动重启导航，也不删除或重新索取已完成的历史主运动授权。

2026-09-06 12:39 最新：本轮 FCOS 性能优化已部署到 J6M 私有实验，候选 Qt 已构建。
旧桥 0.5 Hz 改为默认上限 30 Hz、模型常驻；独立 15 Hz 相机实测 14.90 Hz，
30 Hz RGB 相机实测 28.22 Hz、结果源帧年龄均值 84.57 ms。正常导航相机仍为 15 Hz，
不能把独立 depth=NONE 测试写成完整导航性能验收。详见 `FCOS_PERFORMANCE_20260906.md`。
本轮进入时两导航服务已 inactive、J6M 刚重启，旧 BPU 已停止；仅启动隔离相机/Qt 测试，
现测试全部正常退出，两导航服务仍 inactive，BPU ratio=0。没有启动导航、发送初值/目标/
非零速度或恢复旧目标。保留主运动配置 true、FOD=false、1.60 上限和既有授权标记；
没有当前运行态 watchdog/LOCALIZED 结论。候选 J6M current 仍为 20260905_160700。
J6M 墙钟重启后回到 2025-06-26，后续完整启动使用现有同步流程；本轮测量不依赖板端墙钟。
下方“主链/BPU 存活”是此前会话历史，不能据其自动恢复导航或要求重复本次未启动会话的初值。

本轮后续最新：已收到现场初始位姿，实测 LOCALIZED（overlap=0.9994、RMSE=0.1732、
inliers=5156）。主运动门仍 true、BPU tmux 存活；不要再要求重复设置本次已经完成的初值。
助手未发送任何目标或非零速度；实际移动导航/高速制动仍未由助手验收。

2026-09-06 00:13 最新：主运动授权已完成统一冷重启并实际生效，watchdog motion_enabled=true、
最终线速上限 1.60、超时 0.25 秒，FOD=false。完整 runtime 返回 0，原版 inactive。
候选服务 MainPID=577140，Qt PID=584969，BPU 持续会话 qt_20260906_001122_676676。
10 秒检查最终指令/轮速全零、无保留目标；现为 WAITING_INITIAL_POSE，必须由操作员在
新 Qt 重新标真实位姿。没有自动发目标/恢复旧目标/助手实车移动测试。用户要求删除安全逻辑，
未删除任何避障、定位门、急停或失联保护；已落实其主运动授权诉求。
现场封闭净空/人员清场/实体急停可用已由用户明确确认，不再重复此前不等价的遥控状态位要求。
当前交付状态不再是 false，后续不得把下方历史静止阶段误当现状。详见
`MOTION_AUTHORIZATION_20260906.md`，该记录包含实测证据、诊断探针误报修正及未验证场景。

2026-09-06 00:04：用户再次明确要求开启主运动授权，并已回复确认测试区封闭净空、
人员远离车辆且实体急停有效；这是 README“安全启用运动”中的封闭测试区路径，
没有声称车辆架空。此前仅凭 gamepad_emergency=false 判断遥控停车不可用不成立，
不要再让用户重复该不等价按键检查。只开启主运动门，不开启 FOD、不恢复旧目标、不发非零速度。
00:02:33 起 12 秒预检定位全部 LOCALIZED，最终指令和两轮全零、归属唯一、faults=[]。
已通过现有授权入口生成候选专属 motion_authorized.ok，config 主门改 true、最终线速
上限由 1.70 收紧至用户上限 1.60；NAV 默认 0.80 和 FOD=false 未改。正在统一带原地图
冷重启使参数生效；本条只记录配置和授权，不能据此声称运行态已经 true 或实车导航已验收。
原始配置备份 validation/motion_enable_config_before_20260906.env。保留全部安全保护。

23:45 最新：用户已明确确认在测试区、遥控器停车，要求最高 1.6 m/s；不要再将
测试区域、速度要求和停车方式写成未提供。尚未取得遥控停车键触发的实测反馈，
也没有完成当前现场的制动/失联停车验证或架空条件确认，不能虚构授权标记。
本轮只在验证 candidate master 归属后取消活动 move_base 目标并设置 navigation_pause=true，
服务返回 `move_base goal canceled and retained`（保留的目标不可未经确认恢复执行）。
没有创建 motion_authorized.ok、没有修改速度配置、没有删除看门狗、没有改动两个 false
运动门、没有发送非零速度或目标。23:43:55 起 25 秒只读检查：500 次定位均 LOCALIZED，
MID360/IMU/LD19/融合 scan/ZED 持续更新，1232 条最终指令和两轮反馈全零，faults=[]；
证据 `validation/motion_preflight_20260905_2344.json`。另 25 秒急停断言订阅未收到任一
急停字段 true（timeout 124），这不证明急停故障，也不证明其有效，需现场按键配合。
ControllerMonitor 未更新不单独判故障：源码明确其为固件可选广播，四项必需 CAN 状态
仍新鲜。候选导航/Qt 保持运行、原版 inactive；BPU 独立 tmux 同一会话已达 639 秒/312 帧。

23:35 最新：用户要求启动车辆测试，并提出删除看门狗/设置 true。尚缺测试区域、
最高速度和实体急停确认；未删除看门狗、未改变两个运动门。实时定位 LOCALIZED，
已有活动导航目标，上游曾请求约 0.70 m/s，最终 watchdog 因 motion=false 输出零。
不得未经现场条件确认直接放行旧目标。BPU 旧桥 361402 退出码 137，无正常退出记录，
不能确定强制终止来源；非十分钟到期。已在隔离环境中启动独立 tmux 后台会话恢复，
私有 socket=`runtime/run/bpu_preview.tmux.sock`，session=`j6m-bpu-preview`；
当前 BPU 会话 `qt_20260905_233426_560948`，不依赖本轮临时终端。没有修改系统服务。
停止仍用实验 `start_qt_bridge.sh --stop`，不要删除私有 socket 或杀未知进程。

23:21 最新：用户再次要求关闭当前 Qt 后带静态地图重启，已统一完整冷重启并恢复持续 BPU。
地图仍为 `map_20260829_221335_gimp_cleaned`，Qt 地图 READY、实际框图更新；
20 秒静止检查命令/轮速全零，等待新的初始位姿，两个运动门 false，原版 inactive。
当前运行会话和证据见 `STATIC_QT_RESTART_20260905.md`；本轮没有代码或导航 release 变更。

22:42 最新：用户明确要求 BPU 一直运行，不要十分钟演示。已查明旧桥 `time_limit`
正常退出，改为导航伴随无参数默认持续模式，同时处理板端 660 秒截止；保留超时、
帧龄、限频、资源余量和独占保护。当前持续桥运行，同一进程已实测超过 696 秒/340 帧，
跨过两处截止，55 秒静止检查零速/无故障，详见
`CONTINUOUS_BPU_20260905.md`。下方 600 秒描述只适用于显式限时演示。
没有重启完整导航或切换正式后端；原版仍停止、两个运动门 false。
只写候选/私有实验，无系统服务变更。相机朝向/局部位姿较 22:06 变化，已询问现场
是否人工挪动；最近两段检查输出/轮速均零，不据此宣称全程无移动。
用户随后回复“是有变化”，记录现场变化，不解释为允许发送运动目标。

22:08 最新接续：用户后续明确要求“你帮我启动”，已授权保持静止的完整导航启动，
不是运动授权。21:44 的完整栈和 21:50 的 BPU 伴随运行均完成零速短测；
最新又要求综合页直接显示 BPU，因此仅修改候选 Qt 的综合页显示源，构建和 Qt 测试通过，
已统一停止旧实例、冷启动新 Qt 并确认综合页实际 BPU 框图。22:06 起 20 秒实测速度命令/
轮速全零，地图 READY、等待初始位姿；候选导航运行，原版 inactive。实验桥约 22:16 到期。
实际最新状态、证据和伴随桥接入口以
`FULL_NAV_BPU_OVERVIEW_20260905.md` 及 `validation/overview_bpu_*` 为准。
这条更新取代下方“全部停止/不得自动重启”的历史交付状态；仍不授权运动、猜测初始位姿
或放开两个 false 运动门。18:22 的轮速脉冲根因、全局定位稳定性仍未查明。
FCOS 仅显示，未替代正式 YOLO/五材质后端，未验证 GPU 节省量；BPU 伴随预览最长 600 秒。

21:16 最新补充：用户要求 BPU 与导航 Qt 一起显示。候选 Qt 新增独立预览页，
通过私有相机 master 的真实显示/隐藏停算/恢复/桥接停止留图验证，135 帧 BPU 结果。
当前相机、Qt、桥接和 BPU worker 均停止；两个完整导航栈仍停止。源码/Qt 本机构建已更新，
正式后端和 J6M 导航 release 未改。详见 `experiments/j6m_bpu_20260905/QT_PREVIEW_RESULTS.md`。
正常导航并行运行、原模型 BPU 量化与 GPU 节省量未验收，不自动恢复运动或完整栈。
开工时主机资源未饱和；用户反馈卡顿来源尚未确认，不得擅自重启 ToDesk/修改系统服务。

本副本为 `/home/slam/robot_j6m_ws_optimized_20260905`，当前交付状态见
`OPTIMIZATION_REPORT.md`。下方从原项目继承的历史描述不授权改动原目录。
所有修改、日志和构建仅在副本及 J6M `/map/robot_j6m_optimized_20260905` 中进行；
本机运行统一经 `scripts/optimized.sh` 隔离入口。不得为消除权限错误解除只读隔离。
原版与候选版不能并行控制底盘。运动区域、速度和急停尚待用户确认，不能复制旧运动授权标记。
不得把回放、单元测试或相机静态测试写成实车导航验收。
用户最新明确要求“不要移动，可以启动导航，MID360 已启动”。16:59 已通过隔离入口启动
候选完整栈，双运动放行 false、地图 READY；原版 inactive。17:08:57 起 Qt 操作员
已提交初始位姿，costmap 开始发布。17:11 能进入 LOCALIZED，但 30 秒窗口仍有
DEGRADED/ALIGNING；RMSE 接近 0.35 m 门限，不能宣称定位稳定。详情及实测 JSON 见报告。
18:22 后状态更新：静止续测检测到 4 次左轮 ±0.01237 m/s 反馈，命令均为零，
已通过统一入口停止双机栈；原版也停止。相机画面有近距离白色遮挡、深度有效比例低。
在现场确认车辆静止、轮速脉冲来源与相机遮挡之前，不得自动重启。
新增相机内参别名修复已本机构建并通过 949 项测试，但未做修复后实机启动，J6M 发布未变。
最新状态和证据见 `STATIONARY_FOLLOWUP_20260905.md`。不得猜测初始位姿、发送目标或恢复运动放行。
用户随后确认“完全静止”，但轮速反馈来源和相机遮挡未查清，不自动重启完整栈。
用户已授权把神经网络推理迁至 J6M BPU、释放 NVIDIA GPU；ZED SDK/深度仍留 NVIDIA。
19:27 BPU 实验阶段结果见 `experiments/j6m_bpu_20260905/RESULTS.md`：
参考 FCOS 实机离线运行成功，原模型三份 ONNX 导出及 30 项浮点对照通过；
没有生产切换、实时跨机适配或 GPU A/B，缺 x86 OpenExplorer 量化编译环境。
实验只写独立实验目录，原/候选导航均停止，默认后端和两端导航 release 未变。

# 原项目继承的工作约束

本文件适用于 `/home/slam/robot_j6m_ws` 整个目录树。进入子目录工作时也必须遵守。

## 信息源与优先级

开始任何修改前，至少阅读：

1. 本文件；
2. `README.md`；
3. 与任务有关时再读 `docs/ARCHITECTURE.md`、`docs/HANDOFF.md` 和对应脚本。

`/home/slam/AGENTS.md` 要求读取的全局交接文档记录的是更早期的
`/home/slam/robot_ws` 迁移状态，其中旧网络拓扑、旧设备归属和“尚未完成”结论可能
已经过时。发生冲突时，本项目当前的 `AGENTS.md`、`README.md`、
`config/dual_host.env`、实际脚本和实时只读检查优先。

本项目是室内 J6M 双机版本；`/home/slam/robot_ws` 继续承担机场 GPS 模式。不要用
旧工作区的一体化脚本启动本项目，也不要顺手修改、同步或清理旧工作区。

## 源码和工作区保护

- 工作树可能包含用户已有的 tracked、untracked 和子模块修改。先运行
  `git status --short`，只改任务直接涉及的文件，保留所有无关变化。
- 未经明确要求，不得执行 `git reset`、`git checkout --`、`git clean`、提交、推送、
  删除地图/rosbag/日志/发布版本或重写用户配置。
- 不得刷写 J6M 的 ACORE、MCU、boot、system、整盘镜像或机器人固件。
- 不要把 `/home/slam/robot_ws` 或 J6M 上的无 Git 源码副本当作本项目权威源码；权威
  开发目录是 `/home/slam/robot_j6m_ws`。
- 不要为了抢占串口或 ROS 节点而使用宽泛的 `pkill`/`killall`。项目脚本只停止具有
  PID、运行令牌或严格工作区来源证据的进程；无法证明归属时应报告并停止操作。

## 双机职责和 ROS 拓扑

NVIDIA 本机负责：

- MID360 物理网口驱动；
- USB-CAN/M2、前后 LD19；
- ZED、CUDA、YOLO11、Qt/RViz；
- `/cmd_vel` 最终看门狗。

J6M 负责：

- ROS master；
- Livox topic relay、FAST-LIO、已知地图 ICP 定位；
- MID360/LD19 避障融合、map_server、move_base + TEB；
- FOD 安全仲裁。

两端是两个独立进程环境，但加入同一 ROS1 图：

```text
J6M:    ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.100
NVIDIA: ROS_MASTER_URI=http://192.168.10.100:11311
        ROS_IP=192.168.10.50
```

不得设置 `ROS_HOSTNAME`。ROS1 节点通信使用动态 TCP 端口，两端必须双向可达，不能
只验证 11311。不要默认跨机发送 ZED 未压缩图像、深度图或完整视觉点云；跨机 topic
应维持现有最小数据链。

节点运行位置由两端不同的启动入口决定，不是由 ROS 自动调度：

- NVIDIA：`scripts/nvidia_gateway.sh` / `nvidia_gateway.launch`；
- J6M：远程 `deploy/j6m/start.sh` 最终执行已安装的 `j6m_stack.sh` 和
  `j6m_fastlio_navigation.launch`；
- `scripts/start_dual_host.sh` 通过 SSH 启动 J6M，再启动 NVIDIA 端，并由用户级
  `autolabor-dual-host.service` 托管完整进程树。

## 本机编译与 J6M 部署是两套产物

严禁假定本机 `devel` 会自动出现在 J6M。

### NVIDIA 本机产物

`./scripts/build_workspace.sh` 在 NVIDIA 本机执行 `catkin_make`，产物位于本项目的
`build/` 和 `devel/`。`scripts/setup_env.sh` 加载本机 `devel/setup.bash`，因此本机
节点使用本机产物。

### J6M 产物

`./scripts/deploy_j6m.sh` 执行另一条独立部署链：

1. 通过 `rsync` 只发送选定源码，明确排除本机 `build`、`devel`、`install`；
2. 通过 SSH 在 J6M 的 Ubuntu 20.04 ARM64 chroot 内原生执行
   `catkin_make install`；
3. 生成 `/map/autolabor_runtime/rootfs/opt/autolabor/dual_host/releases/<时间戳>/install`；
4. 完成 `rospack`、共享库和 launch 检查后，原子切换
   `/opt/autolabor/dual_host/current` 符号链接；
5. J6M 启动时依次加载 `/opt/ros/noetic`、基础
   `/opt/autolabor/ros/install` 和项目版本 `/opt/autolabor/dual_host/current`。

不要复制 NVIDIA 的 `devel` 或本机 ELF 到 J6M，也不要让 J6M 直接运行源码目录中的
未安装脚本。J6M 使用版本化 catkin install 空间，回滚只允许在停止主链后通过
J6M 上的 `/map/autolabor_runtime/dual_host/bin/rollback.sh` 切换 `current`。

当前 `deploy_j6m.sh` 只同步并编译其 `paths` 和
`CATKIN_WHITELIST_PACKAGES` 中的包，主要包括：

```text
conventional
teb_local_planner
fast_lio
fast_lio_localization
robot_bringup
autolabor_dual_lidar
autolabor_fod_control
autolabor_dual_host
```

`livox_ros_driver2` 等当前来自 J6M 基础
`/opt/autolabor/ros/install`。若修改了不在部署白名单中的包，不得声称普通
`deploy_j6m.sh` 已让修改在 J6M 生效；应先核对依赖，再明确扩展同步路径和白名单，
或按用户授权更新基础 install。自定义消息有变化时必须保证两端消息定义和 MD5 一致。

### 修改后的选择

- 只影响 NVIDIA 节点：本机重新构建、相关测试、完整冷重启；通常无需部署 J6M。
- 影响 J6M 节点、J6M launch/config 或两端共享包：停止双机栈，本机构建和静态检查，
  执行 `deploy_j6m.sh`，同步时间，再完整冷启动。
- 同一个包在两端都有节点：本机 `devel` 和 J6M 版本化 `install` 都必须更新。
- 静态地图不是程序发布物；使用现有 `sync_static_map.sh` 或带 `--map-set` 的一键启动
  同步，不要塞进程序 release。

推荐发布流程均在 NVIDIA 执行：

```bash
./scripts/start_dual_host.sh --stop
./scripts/build_workspace.sh
./scripts/health_check.sh --static
./scripts/deploy_j6m.sh
./scripts/sync_j6m_time.sh
```

部署属于外部状态修改；若用户只要求解释、审查或诊断，不要擅自部署。

## 网络与设备身份

当前固定拓扑：

```text
MID360 192.168.1.112
  ↕
NVIDIA WCH USB Ethernet 192.168.1.50

J6M eth0 192.168.10.100
  ↕ 交换机
NVIDIA ASIX USB Ethernet 192.168.10.50

NVIDIA wlan0：互联网默认路由
```

NVIDIA USB 网卡的 `eth0/eth1/eth2` 名称会在重启后变化，绝不能把接口名当作硬件身份：

```text
6C:1F:F7:C4:96:B8  ASIX -> 交换机/J6M
50:54:7B:E3:C9:10  WCH  -> MID360
```

- 使用 `scripts/load_config.sh` 的硬件身份解析结果；永久 MAC 未命中时，只允许用
  `config/dual_host.env` 中同时配置且唯一匹配的 USB VID:PID + serial 兜底。不得仅凭
  VID:PID、驱动名或现存 `ethN` 自动认领网卡。
- 正常启动会在任何 J6M 远程停机前调用 `scripts/network_prepare.sh`，恢复
  NetworkManager 托管、修正 profile、等待载波/地址/两端可达；不要把这个顺序改回
  “先远程 stop，后修网络”。同步停机后还必须再次复检网络。
- NetworkManager 持久 profile 应绑定永久 MAC、`connection.autoconnect=yes`，并让
  `connection.interface-name` 保持为空。空接口名是预期状态，不要“修复”为某个
  易变的 `ethN`；激活时可以临时传入当前 `ifname`。
- 两个机器人有线接口必须位于不同 `/24`，且不得接管 Wi-Fi 默认路由。
- J6M 固定地址由厂商硬件持久配置 `hrut_ipfull` 保存；不要依赖易失 overlay 中的
  `/etc` 网络文件，也不要未经确认删除 J6M 默认路由。
- `scripts/network_check.sh` 同时检查 J6M 和 MID360。若它失败，要逐项区分是哪条链路，
  不得把 MID360 未供电误报成 J6M 网络仍故障。

首次网络配置只在两端设备均已接线供电且用户授权时执行：

```bash
sudo ./scripts/configure_network.sh --apply
./scripts/network_check.sh
```

常用只读诊断：

```bash
ip -br address
ip route
nmcli -t -f NAME,TYPE,DEVICE connection show
./scripts/network_check.sh
```

J6M 重启后时钟可能失效；网络恢复后运行 `scripts/sync_j6m_time.sh`。不要把错误时间
引起的启动拒绝误判为编译或 ROS 故障。

## 串口规则

不要猜测 `/dev/ttyUSBn`：

- J6M 串口控制台：
  `/dev/serial/by-path/platform-3610000.xhci-usb-0:4.1.1:1.2-port0`，`921600` baud；
- CAN：`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B400CG51-if00-port0`；
- 前后 LD19：`/dev/autolabor/lidar_front`、`/dev/autolabor/lidar_rear`，由稳定物理 USB
  路径产生；改变插口后必须重新做逐个拔插确认。

先运行 `./scripts/discover_devices.sh`，再核对 `config/dual_host.env`。出现
`IN_USE_BY_PID` 时只报告占用者或安全停止能证明属于本项目的进程，不得杀死未知进程。
使用 screen 诊断后必须退出并释放串口。

## ZED 相机就绪判定

- ZED 2 视频端 `2b03:f780` 必须以 `5000M` 或更高的 USB 3.x 速率枚举；`480M`
  是 USB 2.0 降级，不能仅凭 `2b03:f780/f781` 出现在 `lsusb` 就判定正常。
- 启动或诊断先运行 `./scripts/zed_camera_check.sh --wait 0`。`f780/f781` usbfs 必须对
  `slam` 可访问；hidraw 是可选内核接口，不得因它未生成而单独拒绝已能被 ZED SDK
  识别的相机。重启后 usbfs 若遗留为 `root:root 0600`，使用已安装的
  `autolabor-zed-coldplug.service`，首次安装入口为 `scripts/install_zed_udev.sh`。
- `/zed2/zed_node` 存活不是相机 ready。必须确认 `/fod_camera/image_raw` 和
  `/fod_camera/depth_registered` 有新鲜消息；否则检查 `log/nvidia_ui_*/vision.log`。
- 不要用可能争抢相机的 ZED Explorer/Diagnostic 作为运行栈内健康探针；确需运行时
  先通过统一入口完整停止双机栈，诊断完再完整冷启动。

## 启停、地图和运行态

正常操作只使用统一入口：

```bash
./scripts/start_dual_host.sh             # 无图 FAST-LIO 模式
./scripts/start_dual_host.sh --status
./scripts/start_dual_host.sh --restart   # 完整冷重启
./scripts/start_dual_host.sh --stop      # 同步停止并验证残留
```

- 不要只重启 `nvidia_gateway.sh`；Livox 断流而 J6M FAST-LIO 未同步重置曾导致位姿严重
  发散。修改和故障恢复优先完整冷重启。
- 手工分端启动仅用于诊断，不能替代一键脚本的地图同步、时间同步和生命周期管理。
- 静态地图模式使用 `--map-set global_maps/map_sets/latest`。切换地图模式必须冷重启。
- 静态地图冷启动不会复用历史位姿；必须由操作员根据车辆真实位置发送
  `/initialpose`，并等 `/fast_lio/localization_status` 为 `LOCALIZED`。不得猜测初值，
  也不得把 FAST-LIO 健康等同于全局定位成功。
- `map_server` 只加载二维地图；全局定位由 `fast_lio_localization` 提供
  `map -> camera_init`，本模式不使用 AMCL。
- 定位前 Qt/RViz 依靠 `operator_gui.launch` 的
  `map -> autolabor_map_display_anchor` 静态叶子识别二维地图根帧。该叶子绝不能改成
  `map -> camera_init/base_link`，也不得据此声称车辆已定位；它仅解决初始位姿前的
  地图渲染循环依赖。
- Qt 收到 `/map` 不等于内嵌 RViz 已完成渲染。静态地图启动验收还必须要求
  `/autolabor_operator_gui/map_display_status` 为 `READY`；保留 MapDisplay 自动重订阅和
  实际宽、高、分辨率校验，禁止把这项检查降级成仅检查 `/map` 有消息。
- 静态建图必须从无图模式开始，并确认 MID360、IMU 和要求的固定物理口 LD19 在线；
  普通 rosbag 录制不会自动生成地图。

日志位置：

```text
NVIDIA: /home/slam/robot_j6m_ws/log/
J6M:    /map/autolabor_runtime/logs/dual_host/
```

## 运动安全与运动门交付状态

本项目的正常交付状态为 `MOTION_ENABLED=true`。诊断、部署、停机或排障过程中可以按需
临时改为 `false`，但任务结束前必须恢复为 `true`，同步到 J6M，并核对实际运行的
`nvidia_gateway.launch` 参数也是 `motion_enabled:=true`。除非用户明确要求，禁止把
`MOTION_ENABLED=false` 作为最终状态遗留。`FOD_MOTION_ENABLED` 是独立功能门，不得因
恢复主底盘运动门而擅自联动修改。

`runtime/motion_authorized.ok` 是独立的启动安全门：用户已确认车辆架空、人员远离车轮、
实体急停可用且 CAN 端口已确认后，可以创建并保留；不得在部署、重启或任务收尾时静默
删除。若现场条件未确认或标记缺失，不得代替用户虚构确认，应先询问用户。

即使主运动门已经恢复，除非用户在当前任务中明确授权运动测试，并且现场同时满足车辆
架空、人员远离车轮、实体急停可用、CAN 端口逐个确认、速度限制不高于 `0.3 m/s`，否则：

- 不发布导航目标、非零 `/cmd_vel` 或会恢复旧目标执行的控制指令；
- 不把“启动成功”“`/cmd_vel` 非零”或“定位为 LOCALIZED”当作底盘已经安全执行；
- 诊断控制链时只允许观察、健康检查和零速验证。

只有用户明确要求撤销运动授权时，才执行：

```bash
./scripts/authorize_motion.sh --revoke
```

MID360/IMU 缺失、静态定位未完成或退化、CAN 未确认、指令过期、节点所有权异常时，
现有安全门应保持零速。修改相关代码时必须保留这些 fail-closed 行为，并为安全边界添加
或更新测试。

## J6M chroot 和持久数据

- J6M rootfs：`/map/autolabor_runtime/rootfs`，Ubuntu 20.04 ARM64 用户态，使用 J6M
  自身内核。
- `/dev`、`/proc`、`/sys`、`/run`、`/tmp` 及持久配置/地图/日志目录由脚本 bind mount。
- 不要把地图、日志、FAST-LIO PCD、ROS home 或其他运行数据写进 overlay 根目录；使用
  `/map/autolabor_runtime/{config,maps,fast_lio,ros-home,logs}` 对应持久目录。
- 不要直接修改某个历史 release。通过新部署创建新 release，失败时保持 `current`
  不变；回滚前必须先停止 J6M 主链。

## 验证和交付

根据改动风险选择验证，不能只说“应该可以”：

- Shell 修改至少运行 `bash -n` 和 `git diff --check`；
- Python/ROS 包运行对应单元测试或 rostest；
- 通用静态检查使用 `./scripts/health_check.sh --static`；
- 网络修改使用 `./scripts/health_check.sh --network` 或 `scripts/network_check.sh`，并区分
  J6M 与 MID360 的结果；
- 部署后运行 J6M 的
  `/map/autolabor_runtime/dual_host/bin/health_check.sh`；
- 主链启动后才运行 `./scripts/health_check.sh --runtime`；
- 改动两端消息、launch 或控制链时，验证节点实际运行主机、topic 发布者/订阅者、消息
  MD5、频率、时间新鲜度和双向网络；
- 未经明确要求不要为验证重启整机、部署、启动实车主链或连接运动授权。

最终交付必须说明：改了哪些文件、验证了什么、是否部署到 J6M、J6M `current` 指向
哪个 release，以及仍受哪些离线硬件或安全条件限制。不要把本机构建成功描述成远端
部署成功，也不要把静态检查描述成实机运行验收。
