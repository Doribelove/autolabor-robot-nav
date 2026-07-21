# T12 Residual SAC 学习性诊断、配对评估与单因素修复交接

更新时间：2026-07-13 21:29 CST

本文承接 `CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md`。所有结果均为 Gazebo 仿真，
`formal_result: false`；不授权实车运动、实车 TEB 写入或训练预算扩展。

## 1. 当前冻结决定

- 原两 seed 训练、离线诊断、24-episode 三方法配对、curriculum 单因素修复 pilot 均已完成。
- 第二单因素 episode-anchor amendment 已冻结并启动，但 seed101 两次 bounded attempt 均发生
  `move_base` SIGSEGV；完整性门槛失败后已停止，seed102 未启动。
- 后续 episode-boundary atomicity 单因素通过 500-step 完整性 pilot，但首个 2000-step
  seed101 在在线参数激活阶段 `move_base` -11；该失败已由后续 activation-timeout barrier 与
  静态 footprint 生命周期单因素继续审查。
- 最新静态 footprint 修复先通过 1300-step stress gate，再完成 seed101/102 各 2000-step；
  两 run 均无 crash/SIGSEGV，测试合计 14/14 goal，但仅 seed101 validation 上升，seed102 下降，
  因此学习门失败，新三方法配对未授权。
- 当前没有 ROS、Gazebo、move_base 或训练残留进程。
- curriculum pilot 的完整性/仿真安全门通过、学习门失败；episode-anchor pilot 的完整性门失败，
  学习性不可判定。
- 禁止扩大 seed/step 预算；不得把 14/14 test goal 写成 Residual SAC 的学习增益。
- 禁止重启当前历史 pilot。下一步只允许先离线审查 Residual 动作与 projection 边界匹配；
  如需再训练，只能登记一个新的学习因素 amendment，不能同时修改 radius、reward 与 EMA。

## 2. 原训练离线诊断

报告：`artifacts/t12/residual_training/t12_residual_learning_diagnosis.yaml`。

- seed101/102 validation change：-0.2397、-0.1763。
- 两 seed 实际都只训练了 `t11-train-clear-straight`；原因是每次 episode 结束重新设置
  未变化的 curriculum 时，`set_scenarios` 把场景索引重置为 -1。
- 全部 step 平均 projection intervention rate：85.87%。
- 每个 episode 首步的启动 theta 到 TEB-Tuned anchor normalized L1 固定为 9.0。
- 平均训练 episode 约 4.02 steps，97.79% 不超过一个 4-step hold 窗口。
- 原始动作并未严重饱和，EMA 后 L1 约为原始动作的 34.5%；主要瓶颈不是动作饱和。

## 3. 冻结三方法配对评估

预注册：`experiments/manifests/t12/residual_paired_eval_preregistration.yaml`。
报告：`artifacts/t12/residual_paired_eval/t12_residual_paired_eval_report.yaml`。

- 3 方法 × 2 training seed × 4 frozen scene = 24 episode；6/6 run valid。
- selected SAC、zero-residual、TEB-Tuned 均为 4/8 goal，0 collision/emergency/interface fault。
- selected - zero-residual paired mean return：+2.0381；seed101 +3.7356，seed102 +0.3406。
- selected 与 zero-residual 在双方均成功的 4 对 episode 中没有导航时间或路径长度优势。
- 配对 return 的同号门槛通过，但成功率没有改善，且原训练 validation 与数据覆盖诊断未通过；
  因此联合结论仍是“不支持稳定学习增益，不扩预算”。

## 4. 已执行的单因素 amendment

合同：`experiments/manifests/t12/residual_curriculum_single_factor_amendment.yaml`。

唯一执行因素：未变化的 curriculum 列表不再重置 scenario index。保持不变：

- seed 101/102；每 seed 2000 steps；checkpoint 1000/2000；
- reward、Residual radius、risk scaling、EMA 0.35、hold 4；
- T12Safety、场景合同、validation-only 模型选择规则。

修复 pilot 报告：
`artifacts/t12/residual_curriculum_fix_pilot/t12_residual_curriculum_fix_pilot_report.yaml`。

- 两 seed 均实际覆盖全部 5 个训练场景；每 seed 分布相同：clear-straight 64、clear-left 63、
  clear-right 63、obstacle 53、corridor 45。
- seed101 validation：23.5874 -> 23.3387，change -0.2487，选择 1000-step。
- seed102 validation：23.2320 -> 23.2773，change +0.0452，选择 2000-step。
- cross-seed mean validation change：-0.1018；“每 seed 均改善”门槛失败。
- 两 seed test 合计 14/14 goal，0 collision/emergency/interface fault/crash。

## 5. 修复后诊断与第二单因素

报告：`artifacts/t12/residual_curriculum_fix_pilot/t12_residual_curriculum_fix_diagnosis.yaml`。

- curriculum 数据覆盖已修复；训练 episode 平均约 6.94 steps，只有 23.09% 不超过一个
  hold 窗口，明显优于原训练。
- risk scaling 已实际激活，平均约 0.729；安全干预率提高到 46.75%，符合障碍/走廊进入训练。
- projection intervention rate 仍为 87.06%，没有随 curriculum 修复下降。
- 每 episode 首步 theta 到 anchor normalized L1 仍固定为 9.0；平均 applied-to-anchor L1
  约 3.04。这是下一优先审查因素。

第二单因素合同已经冻结：
`experiments/manifests/t12/residual_anchor_initialization_single_factor_amendment.yaml`。
唯一变化为 `initialize_residual_anchor:=true`；reward、radius、EMA、hold、safety、curriculum、
seed 和 2000-step 预算保持不变。该轮运行前冻结 checksum 全部通过；当前完整 pytest 为 134 passed。

## 6. Episode-anchor pilot 结果

失败报告：
`artifacts/t12/residual_anchor_init_pilot/t12_residual_anchor_init_failure_report.yaml`。

- seed101 attempt1 已生成完整 1000-step checkpoint，随后在训练 episode reset 的参数快照恢复
  调用链上 `move_base` 以 -11 退出；dynamic-reconfigure 服务随后不可用。
- 按冻结脚本执行唯一一次 bounded retry。attempt2 在 1000-step 前先出现 TEB invalid timediff、
  oscillation abort、`config_seq 5` 激活超时，随后同样发生 `move_base` -11。
- 两次失败均位于 `TrainingEnvironment.reset -> _restore_snapshot ->
  GazeboTrainingAdapter.restore_parameter_snapshot -> TebParameterClient.apply` 的 episode 边界。
- seed101 两次均未完成，`zero_process_crash` 和完整性门失败；seed102 因门禁未启动。
- 没有完整 run manifest/episodes/steps/model-selection bundle，因此 validation 改善、首步 anchor
  距离及学习性门槛均不可判定。不能把失败 checkpoint 当成候选模型。
- 日志能证明当前 episode-boundary anchor restore 协议不具备继续训练所需的运行稳定性；但仅凭
  SIGSEGV 日志不能断言 anchor 数值本身是 C++ planner 崩溃的唯一原因。

## 7. 当前允许的下一步

本节原边界审查要求已由第 8--9 节完成并取代。当前下一入口以第 10 节为准：不扩预算、
不启动新配对，先离线审查 Residual action 与 projection 边界匹配；若继续训练，必须先冻结
一个且仅一个学习因素 amendment。

## 8. Episode-boundary atomicity 单因素结果

合同：`experiments/manifests/t12/residual_episode_boundary_atomicity_amendment.yaml`。

唯一系统变化是把 episode 边界串成原子顺序：确认 action server 存活、取消旧 goal、等待 action
终态和 local plan 静默 0.25s、恢复 anchor、reset 场景、最后下发新 goal；未静默或 server
不可用时在参数写入前 fail-closed。reward、Residual radius、EMA、hold、safety、SAC 与预算未改。
完整 pytest 为 134 passed，workspace 71 packages 构建成功。

500-step 完整性报告：
`artifacts/t12/residual_boundary_integrity_pilot/t12_residual_boundary_integrity_report.yaml`。

- 89 次边界确认、0 quiesce failure；86 个 episode 首步到 anchor 的最大/平均 normalized L1
  均为 0；五个训练场景全部覆盖。
- 0 crash、activation timeout、interface fault、dynamic-reconfigure failure、collision、emergency；
  test 7/7 goal。
- projection intervention rate 从参考 87.06% 降到 59.8%，下降 27.26 percentage points，
  通过预注册的 ≤77% mechanism gate。

因此按门禁进入相同 seed/budget 的 2000-step pilot。失败报告：
`artifacts/t12/residual_boundary_atomicity_2seed_pilot/t12_residual_boundary_atomicity_2seed_failure_report.yaml`。

- seed101 完成 1000-step checkpoint 并继续训练；在第二个 1000-step 区间的活动 episode 内，
  先出现 oscillation/recovery，再发生 `config_seq 2` activation timeout 和 `move_base` -11。
- 下一 reset 边界正确检测 action server 已消失并 fail-closed，没有继续执行不安全的 anchor restore；
  说明边界修复有效，但不足以保证 2000-step 在线调参稳定性。
- seed101 未完成，无完整 validation；seed102 按门禁未启动；三方法配对未授权。
- 本轮不是“学习性不提升”，而是“全预算完整性未通过”；因此不得进入 Residual radius/projection
  学习因素修改。下一审查对象必须仍是系统完整性：活动 episode 中 planner oscillation/recovery
  与在线 dynamic-reconfigure/activation 的交互。

## 9. Activation barrier 与静态 footprint 生命周期结果

系统 amendment：
`experiments/manifests/t12/residual_activation_timeout_recovery_barrier_amendment.yaml` 与
`experiments/manifests/t12/residual_static_footprint_reconfigure_amendment.yaml`。

- activation timeout 异常路径不再立即恢复 snapshot，而是把恢复延迟到下一个原子 episode
  边界；recovery 后边界静默期为 1.0s。
- TEB dynamic-reconfigure 回调只更新受 mutex 保护的 `TebConfig`，静态 polygon footprint
  仅在 planner 初始化时加载一次，不再在每个 SAC action 上重建并替换 planner robot-model 指针。
- TEB 原生测试 6/6、项目 Python 测试 135/135 通过；TEB package Release 构建成功。
- 1300-step stress report：
  `artifacts/t12/residual_static_footprint_reconfigure_stress/t12_residual_static_footprint_reconfigure_stress_report.yaml`。
  189 episodes、1422 steps、五训练场景齐全、首步 anchor 最大误差 0、footprint load=1、
  0 crash/activation timeout/interface fault/collision/emergency，test 7/7 goal。

完整两 seed 报告：
`artifacts/t12/residual_static_footprint_2seed_pilot/t12_residual_static_footprint_2seed_report.yaml`。

| seed | validation 1000 | validation 2000 | change | selected | projection | safety | test |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 101 | 23.6056 | 24.0583 | +0.4527 | 2000 | 65.1% | 46.4% | 7/7 goal |
| 102 | 23.7860 | 23.7076 | -0.0784 | 1000 | 69.5% | 45.55% | 7/7 goal |

- 两 seed 均覆盖五训练场景，首步 anchor 最大 normalized L1=0；0 crash/SIGSEGV、
  dynamic-reconfigure failure、test collision/emergency/interface fault；每 run footprint load=1。
- `validation_improved_each_seed=false`，因此 `learning_gate_passed=false`；不得启动新的
  selected SAC / zero-residual / TEB-Tuned 配对，也不得扩训练预算。
- 审计限定：seed101 boundary audit 记录一次已恢复的 activation-timeout barrier（config_seq 19），
  但 `steps.csv` 的 transition-drop 计数为 0，生成汇总据此把 `zero_activation_timeout` 置为 true。
  因此可说“运行完成且未崩溃”，不能说“严格零 activation-timeout 事件”。该口径差异已冻结在
  `t12_residual_static_footprint_conclusion.yaml`，不影响学习门失败和禁止配对的决定。

## 10. 当前结论与下一入口

- 系统完整性修复有效：消除了此前可复现的 footprint 重建/指针替换崩溃链，并使两 seed
  完整跑完 2000 steps、validation 和 test。
- 学习提升不稳定：仅 seed101 上升，seed102 下降；不能宣称 Residual SAC 有复现增益。
- projection 仍为 65.1%/69.5%，明显高于理想动作可执行率；下一单因素优先审查
  Residual action 与 projection 边界的匹配，不同时改 reward、radius、EMA 或 hold。

## 11. 最新 Residual / projection 离线诊断

冻结报告：
`artifacts/t12/residual_static_footprint_2seed_pilot/t12_residual_projection_alignment_diagnosis.yaml`。
该报告只读解析两 run 的 `episodes.csv`、`steps.csv` 与 `model_selection.yaml`，没有训练、
Gazebo 启动或参数修改；同目录 `.sha256` 校验通过。

- validation change 仍为 +0.4527/-0.0784；跨 seed 均值 +0.1872 不能替代“每 seed 都上升”门槛。
- 训练 projection rate 为 65.1%/69.5%，均值 67.3%；无前一安全干预时仍为 44.82%/48.81%，
  主因是 `max_vel_theta:ackermann_turning_radius`。固定 anchor 位于
  `max_vel_theta=max_vel_x/1.2` 边界，而 residual mapping 的 `max_vel_theta` 行为零，
  因此降低 `max_vel_x` 时候选动作天然越过该耦合边界。
- 前一步被安全层修改后，下一步 projection rate 为 92.48%/98.91%，均值 95.7%；
  `min_obstacle_dist`、`inflation_dist`、`weight_obstacle`、`weight_viapoint` 的 rate-limit
  原因几乎全部出现在此条件下。这说明大幅候选/执行差异主要是 WARNING 安全 profile 后
  residual 立即回指固定 anchor 所致，并非单纯 Residual radius 过大。
- 原始 SAC action 各 eta 平均绝对值约 0.51--0.53，饱和比例仅 3.73%--3.91%；EMA/hold 后
  action L1 约为原始值的 37.6%--38.6%。当前没有“探索动作严重饱和”的证据。
- 各 theta 的 residual radius 平均利用率最高仅约 14%，p95 最高约 34%；当前没有依据先缩
  radius。训练 episode 平均 7.75--8.20 steps，仅约 18.5% 不超过一个 hold 窗口。
- 安全干预率 46.4%/45.55%，全部为 NORMAL/WARNING；0 EMERGENCY、0 FAULT、0 fallback。
  奖励仍由 progress 与 terminal 主导，parameter-adjustment 平均每步仅约 -0.011/-0.012，
  不能用训练 return 或 14/14 test goal 单独证明学习增益。

冻结决定不变：不扩预算、不启动新三方法配对、不宣称 Residual SAC 增益。下一次训练前只允许
先登记一个“Residual 候选动作与实际可执行边界/安全后恢复路径对齐”的学习因素 amendment；
不得同时修改 reward、radius、EMA、hold、SAC 或 safety。诊断本身不授权直接开跑。
