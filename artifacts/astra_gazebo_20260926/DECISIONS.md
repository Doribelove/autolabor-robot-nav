# 决策与证据日志

## D001 — 以当前本地完整dirty tree为输入，保留外部来源不确定性
当前HEAD 717424c0b8190b4b2366d081529da056801e9b98，原生Ubuntu20.04.6，无活动ROS/训练。不能独立证明与Windows命名解压副本完全相同（未提供其manifest）；以用户明确指定的当前工作树作为可审计实际输入，逐文件SHA256复制，保留此来源限制。

## D002 — 单独source和构建
清除继承arena_ws/catkin_ws路径；源码快照只读，新workspace包含未提交/未跟踪文件与模型。历史build/devel不复用。稳定实车工作区不修改、不source。

## D003 — 历史发现仅用于可证伪假设
R2已完成800回合，仍为阴性。独立审计发现actor speed seed项消除、候选方向估计/停止安全可用性与评价几何存在疑点；先独立几何/接口验证，不修改历史数字或复用历史seed。

## D004 — 当前底盘物理边界必须保留
现有Gazebo ODE执行扫描/接触，但车体由SetWorldPose运动学积分驱动。摩擦/轮力不决定运动；不把调mu参数当摩擦压力证据。后续必须实证执行动力学影响并明确模型层级。

## D005 — 独立审查修复执行前计账与ROS子进程回收
预准备异常也写terminal receipt；ROS每节点setsid使父PGID不足，改为本run专属ROS_LOG_DIR+PID starttime验证后清理，确认端口释放。真实子进程fixture需独立验收；未把静态解析当runtime通过。

## D006 — smoke观测与论文图分开
原生2560×1440相机、model/link/joint、scan/odom/TF逐帧绑定。未知轮地高度/时间对齐必须实测，machine/visual/实际版心均过才可论文选图。bootstrap仅诊断，不进入性能统计。

## D007 — A01封存与新A02物理/采集修复
A01 consumed1/unstarted-retired3，goal9.15s但17帧machineQC0，轮底17.8–20.0mm悬空。保留301资源原始代码blob、所有日志、原图；不能以到达goal掩盖截图/场景失败。新A02 seed260926002只修spawn落地z和观测时间关联及相机；原R2策略未修改，非性能证据。

## A02 pipeline checkpoint

Four method identities completed once, all goals reached, full bags present. Independent raw replay passed 76 checks; source-stamp acceleration lies within configured -0.9/+0.6m/s²; historical fixed-dt acceleration was misleading and is retained only as legacy description. All turns were zero: no turning qualification claimed. Four raw selected frames and 170mm panel Q01 visually passed; first frame of every episode retained as unsettled, ineligible. No performance inference.

Scene v1 diagnostic input review found all 18 static Ackermann witnesses but all six occluded blocks had actor/screen swept intersections and failed intended initial occlusion. Keep v1 inputs immutable and non-executable for the research matrix; produce v2 fresh identities with full swept/visibility checks. New exact-timestamp physics evidence plugin built; a positive-contact fixture is required before trusting its new measurements.

## Source-stamped measurement Q01

New WorldUpdateEnd evidence plugin passed the preregistered falling-cube positive/negative fixture at seed260926003 attempt1, 17/17 checks; 201 model and link packets, 2009 contact packets and 1807 detected contact pairs. Source contact offset is zero within floating-point precision. Episode exit0 and required bag topics present. This qualifies measurement only; kinematic M2 teleport dynamics and turning are unqualified.

The old M2 backend makes discrete SetWorldPose updates, so a continuous-time Lipschitz swept-clearance bound cannot be claimed from command caps alone. The independent evaluator will retain a model-based interpolation diagnostic and mark continuous physical sweep qualification pending. M12 needs separate actual physics dynamics evidence before stronger safety/dynamics claims.

## Q02 terminal negative and Q03 mechanism

Q02 completed all 36s and retained 79/80 passes. 20ms source spacing and fixed2ms published/truth phase rule out irregular sampling; independent wheel axes reproduce the actual center slew failure. Q03 preregisters one steering kinematic hold mechanism (both SetPosition and zero joint velocity each2ms), unchanged50Hz internal state/root kinematics and all original gates. It only tests observed mismatch, not full physical actuation. D01 map/route information is asymmetric under legacy configurations and cannot support a fair comparative performance claim; new shared interface required before main study.

## Q03 not adopted

Fresh seed260926006, one36s attempt, completed with79/80 unchanged gates. Actual center-steering slew maxabs1.0073952057044138rad/s still exceeds0.85; internal maximum0.75. Zero whole non-ground robot contacts and no infrastructure exception. This candidate failed its prediction and is not adopted. Keep all Q02/Q03 sources and raw records. Next actual factor is finite-force physical wheel/steer actuation, including base-centre versus rear-axle reference semantics.

## M12-P01 full36s terminal negative

All50/500Hz source physical states, actual/target/effort metadata passed recorded prechecks, but unstamped Bool receipt gap reached.12s vs.06 and stopped original assessor before motion gates. Independent preserved-failure continuation later also found actual center20ms slew.901224rad/s above.85, six command gaps above.075(max.16) and floating elapsed phase32 boundary published one old-phase command. Preserve original report/negative. P02 changes measurement transport and command scheduler isolation/integer timing only; physical controller/gates remain unchanged. Any later gain tuning is a separate factor and identity.

## P02 measurement closure before physical gain trial

P01 independent diagnosis retained actual20ms center slew .901224rad/s versus .85, plus measurement/scheduling failures. P02 changes only source-stamped status and separate integer-time command scheduling; recorder TCPROS readiness and1s stationary warmup are explicit. Existing dynamics,36s schedule and numeric gates are unchanged. Independent negative fixtures exposed incomplete source-status/recorder/schema checks; all are repaired before any new identity is consumed. A separate P03 Kd4→6 paired three-seed design will test the position-error catch-up mechanism only after P02 measurement integrity.

## P02 terminal result

Fresh seed260926008 completed36s with119/120checks true; sole false actual center slew0.8863637149>0.85rad/s. Old Bool receipt max gap0.14s remains reported, while the new header source stream passes exact target/odom joining, coverage and monotonicity. Integer command loss and deadlines pass. All26 bag topics present and no assessment/runtime exception; independent formal-interval raw/bag replay remains required before P03. P02 is immutable and will not be rerun.

## P03 terminal recorder-tail failure

Exact release085224c0 consumed2 and retired4 units. Control valid physicalnegative0.881134rad/s; treatment report is incomplete after3 cross-source checks false. Preliminary root ns audit shows dense_links ended37.124s while formal end37.110s and other raw sources continued37.140–37.144s, without interior dense gaps. The original collector required only formal-end drain while the assessor checked allraw; independent confirmation pending. No result is rewritten, no seed/identity reused, no Kd6 qualification claimed. A new authoritative postpause source-watermark collection protocol is the next repair candidate.

## M12-P03终止归因与P04采集修复
P03两回合正式窗口均完成源协议24/24，治疗回合的三项join失败仅出现在37.110s正式结束之后：raw link37.142s、joint/odom37.140s，而dense link提前止于37.124s；bag的同源尾包完整。原结果仍为infrastructure_failure，预算2 consumed+4 retired，seeds260926010/011不复用。Kd6的单seed离线诊断peak0.8280642仅支持机制检查，不构成资格。
新P04计划在新seeds260926012–014作六回合Kd4/6配对；先修复collector，pause后在原5s墙钟预算内读取真实Gazebo最终时间，完整接收每流最后slot及所有中间slot，同锁核对源join后关闭。不得剪尾、放宽数值门或重写P03。34项作者故障测试通过，独立复核尚待完成，尚无P04 release或执行。
TIM01 r02经root实际170mm六面板查看及21资源复核接受，累计六个非重复图组；PHY01三pair预选条件不满足，保留缺失。

P04独立32/32控制流故障测试、作者34/34、manager55/55和root六份真实launch参数解析通过；原120门/metrics保持精确相同，新增完整性门。新release cb256ee6105b81d2935ae37d6794df3621b282947c6a28214be53aa05a55f5f5绑定450资源，seeds12–14六单位已启动。首个seal脚本因repository-relative审计source路径解释失败，在release写入/预算消费之前保留失败并修正路径解析；未修改审计或仿真输入。

P04在首回合m12_physics_p04_s012_kd4后终止，1consumed+5retired。全部十流采集最终水位37.144s完整、四join通过，drain0.3441s；原121门119true，物理中心角变化率.8795876超过.85，另relay失联段门失败。原headerless callback把终止zero记37.106s，而source Header37.108s、bag cmd_vel37.110s；这次不是浮点边界，整数转换也仍会误归属。原始报告不改。
新P05修复方向是被动记录驱动实际接收命令时的world_->SimTime源时间，替代以本地/clock回调时刻判定失联窗口；保持原120ms loss grace（75ms为另一命令间隔门，二者不可混淆），保留控制/力学实现和全部数值阈值。新seeds17–19待设计/审查，不能复用P0412–14；观察器Q02单独预留15/16。

### P05 final preexecution closure and launch
Three independently exposed ingress weaknesses were fixed before release: source stamps later than paused world time/off-grid now fail; callback parsing/error paths share the close lock; obsolete headerless receipt-clock match is diagnostic only, replaced by strict complete ordered source-to-driver matching. All original physical thresholds remain unchanged. Initial drafts and counterexamples are preserved. Final roster 122; 53 author probe +55 manager +68 independent tests pass. Six actual launch expansions pass. New release 523994debda2471ed1f0b0eca39ded95867f143823f6ea6d390c115d0a68dfea binds 527 resources and exact seeds260926017–19, 6 units. PHY03 selected by frozen three-control median rule before simulation. No old-stage retry or qualification reinterpretation.

### P05 terminal startup identity failure, P06 repair
P05 sequence1/seed260926017/Kd4 consumed one unit and stopped before schedule/commands: command-source validate_runtime_environment retained the P02 filename check while actual expanded URDF has P05. Required command node exited; model spawn was interrupted and probe readiness timeout was downstream. No motion data or bag was produced. Remaining five units retired. The root stage-only source equivalence check and launch parameter preview missed actual startup-validator interoperability; the original 176-test review is preserved as an offline snapshot with this blind spot. P06 changes only stage identity and source expected plugin filename, keeps exact P05 physical implementation, and adds actual source+probe startup validation on each expanded six-row URDF. P05 files/seeds17–19 are not retried. Fresh20–22 assigned only to new P06 design, pending independent review and release.

### P06 release and fresh execution
Actual six launch expansions invoke both source and probe startup validators. Independent126focused checks pass, including exact old source rejection and paused-service/identity negatives. Release b1ee6e965074b712c66d257cf652c5caae59e074abd5ab730fbec5ed88b51adc binds573resources; seeds260926020–22, sixpaired units, no retry. An initial root sealing-helper test-path typo failed before release/allocation and is preserved, then corrected to the existing inherited P05 test path; runtime bytes unchanged. PHY04 preregistered before release.

### P06 terminal callback lexical collision; P07 exact repair
P06 seed260926020 control completed36s of physical motion and all27 bag topics exist, but raw callback_errors records GetWorldPropertiesResponse object not callable: the terminal query local variable world overwrote the same-scope nested WorldState callback world(). A delayed /astra/world_links message then failed. Stable world watermark37.146s and actual accepted terminal command37.114s are preserved; original terminal drain failure is not reclassified as qualification. P06 closes1consumed5retired. P07 fixes only query variable world_response, retains all physical formulas/thresholds and source scheduling, and uses fresh23–25. Full-runtime lexical shadow audit and late-callback regression added; P04/P05 earlier timing-dependent passes do not establish absence of this bug.

## 2026-09-26T11:07:12.520348+00:00 P07 result and Q02 terminal startup
P07 complete6/6, all3 Kd6 pass exact122 gates at20ms sampling, control1/3passes; independent review8ce74b527fd15e69ebd9587c10233bce715949f21eae8ace363587dfe59f3380. Kd6 selected for the frozen simulation actuation component only. Dense2ms seed023peak.855810774468 exceeds.85; no continuous rate-bound claim.
Q02 unique release dcde24d6531e69aef942063866f8b0db2c0fb4a7136cce43dfe650f736453553 consumed seed015 then terminated before unpause/command window: deleted-map classifier rejects any deleted absolute mapping before ELF role check. Exact path absent because original function threw before saving maps. Seed016retired. Q02 unrestartable. New Q03 fresh027/028 classification/diagnostic repair authorized by long-run contract; physical/geometry gates frozen.

## 2026-09-26T11:39:20.082253+00:00 Q03/Q04终止与模型表示修复
Q03 seed027在unpause前被已保存的NVIDIA .gl可执行memfd映射严格拒绝，seed028退役；新Q04仅加入经真实GL上下文实验与独立反例审核的精确8KB映射例外并要求三渲染驱动精确hash。Q04 seed029完成10s采样，13门11通过；唯一根因是评估器以未量化URDF固定关节合并质心核对实际SDF量化质心。源时间、命令、接触和图像采集均完整。原assessment失败保留，seed030退役。修复将比较实际转换模型，不扩大原1e-7公差；不能把离线诊断重判为Q04资格。

## 2026-09-26T12:16:23.199604+00:00 Q05 fresh component result
New P04 binds actual URDF-to-SDF conversion, runtime mass/inertia/collision geometry, and actual libsdformat mapping without widening the original numerical tolerance. Exact fresh031/032 both completed attempt1 with all13 gates passing; no retry or retirement. Both captured5000 two-ms intervals and zero recorded illegal robot/actor contact. Root individually viewed allfour native frames; independent replay still pending before navigation-evidence qualification. This is component evidence only, not navigation performance or formal/general safety. New R2 sensor lifecycle r1 independently fixes swallowed second TF lookup failure while preserving old algorithm/weights; prior failure report is retained.

## N01 fresh four-method integration release
Q05 independent result f6fd40d24cc28d3a9887a09fafd646cf54a8b0f80fc61b5a6fc3cb69b80cde57 qualifies only the declared native geometry engineering component. New exact N01 release ae155b33ea2ad6c732163265c266764817ff9268587e136a0c9f9ff93aed9ea5 binds601resources and four attempt1 identities with freshseed026. Algorithm failures remain evidence; capture/infrastructure failure retires remainder; no retry/resume. A root qualifier relative __file__ binding failed before release/allocation, preserved in the first sealer log and qualification JSON; r1 canonicalized that path without runtime/evidence changes. The final qualification binding is 2e1ef51c8073ae91c9a0542fbfb2d11989b9f8e0d27dfdea92e4ee5ef47be1f7. Training/formal/hardware remain untouched.

## N01 terminal mixed integration result
Allfour freshseed026 identities consumed once. Analytic/actualTEB/frozenR2 completed with physical capture and declared model safety gates; descriptive goal times8.562/7.814/8.954s are not paired performance evidence. DWA emitted finite nonzero lateral vy, rejected by the unchanged Ackermann gateway after0.26s. Its single early native frame then failed the collector two-frame requirement, preventing final map closure; original failure stays unchanged. New N02 separates supplemental visual availability from physical evidence and retains the DWA diagnosis for a separately fresh successor. Seven native frames individually viewed; no stable/hardware operations.

N02 terminal integration pass: fresh260926033 attempt1, actual goal8.980s, 4490 physical2ms intervals, engineering minimum1.4472005883m, zero illegal contact, 9 integrity/4 engineering-safety checks true. Sensor-only ROS registration passed, immutable old R2 algorithm/weights. Manager and outer shell exit0 both retained; original N01 outer143 unexplained. Two native frames individually viewed. Single clear fixture; no route adaptation/performance/primary-matrix qualification. Independent terminal review pending.

N03 terminal integration pass: fresh260926034 attempt1, actual geometry goal9.568s (coordinator9.580s), 4790 physical2ms intervals, minimum engineering bound1.4525852493m, zero illegal contact, 8 integrity+4 engineering-safety gates. Actual DWA/new acc_lim_y2.5 integration reached goal; strict nonzero lateral command guard retained. Manager/outerexit0. Two native frames root viewed. This is one clear fixture, not fair tuned baseline performance evidence; N01 negative unchanged. Independent terminal review pending.

N02/N03 independent terminal closures passed (1d6df0f1… and0f9f8928…). N02 includes18 retained history-age diagnostics:5decision+13control traces, publication-tail up to250ms; loop start differs from exact feature read, no new retrospective gate. N03 actual85 private commands all finite with zero lateral components, maximum actualodom |vy|0.0245409m/s, within the finite native-library diagnosis domain. M08v2 independent geometry/availability passes; measured maximum20.023795ms is under50 but notunder20, and fullplanner qualification remains pending. New N04 planned48 development episodes over6parent layouts×2×4, fresh263001001–012, exact50ms actor source-to-Begin limit declared before outcomes; not yet allocated/released.
