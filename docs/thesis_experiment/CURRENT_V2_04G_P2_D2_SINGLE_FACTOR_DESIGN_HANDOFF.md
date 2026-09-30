# V2-04G-P2-D2 单因素性能设计交接

更新时间：2026-07-24

## 1. 结论

`V2-04G-P2-D2` 已完成纯离线单因素设计闭环，确定性构建状态为
`offline_single_factor_design_closure_pass`，独立审查状态为
`offline_single_factor_design_independent_review_pass`。

本阶段只完成“下一轮该怎么公平地做”的冻结，不是性能实验，也不是速度上限安全认证：

- `formal_result=false`
- `runtime_ready=false`
- `execution_ready=false`
- `execution_authorized=false`
- authorized evidence budget = 0
- authorized training steps = 0
- 不存在 execution release、exact executable schedule、attempt、journal 或新仿真结果
- 未启动 ROS、Gazebo、move_base 或训练
- 未修改 active runtime profile、P1/I5/T12 冻结证据或 `/home/robot/robot_ws`

因此现在不能声称性能提升、RL 提升、1.40 m/s 已安全、winner、部署就绪或实车可用。

## 2. 为什么只选这个因素

P1 的正式 270-episode 研究已完整结束，但效率门失败：

- circle-contact 相对 legacy 的 restricted completion time 降低 `3.7624%`
- 单侧 95% 下界 `2.7904%`
- 低于冻结的最小效果门

P2-D1 随后只读分析了 30 个 semantic-clear treatment traces：

- frozen-prefix same-path envelope 点估计余量 `12.0144%`
- 单侧 95% 下界 `11.2649%`
- 达到 10% 目标的一阶速度估计约 `1.38349 m/s`
- 当前 `anchor_cruise.max_vel_x=1.20 m/s` 且已处在旧 bank 上界
- 历史单 seed 方向检查中，速度坐标具有目标量级敏感性，而
  `weight_optimaltime` 只有约 `1.25%` 量级

所以 D2 唯一行为因素被冻结为：

```text
TebLocalPlannerROS.max_vel_x
  ↕ typed registry
anchors.anchor_cruise.values.max_vel_x
```

控制值为 `1.20 m/s`，calibration grid 为
`[1.20, 1.25, 1.30, 1.35, 1.40] m/s`。除 versioned registry 上界元数据外，
候选只能改这个行为字段；TTC、circle-contact estimator、overlay、加速度、净空、
障碍权重、优化器权重、变化率、transaction、scene 与 evaluator 全部冻结。运行时不得读取
scene label 或 expected status，既有 dynamic overlay 仍必须生效，不能为了速度绕过安全门。

候选 amendment 目前只是一份离线 patch 规范，没有物化到 active runtime bank。

## 3. 安全边界

拟议设计上限为 `1.40 m/s`。独立审查从冻结合同重算：

- Ackermann 最小转弯半径：`1.20 m`
- circle-contact cruise `max_vel_theta=0.65 rad/s`
- fixed TEB `max_vel_theta=1.00 rad/s`
- 五个速度档均满足冻结的
  `max_vel_theta <= max_vel_x / minimum_turning_radius`
- 冻结 `acc_lim_x=0.80 m/s²` 不超过 simulator candidate 的
  `1.20 m/s²`
- service deceleration 使用 `1.60 m/s²`

保守延迟项包含 laser delay+jitter、一个 5 Hz supervisor 周期、command delay+jitter、
actuator time constant、transaction ack/readback/activation，以及从 1.40 回落到 1.20 时按
`0.50 m/s/s` 变化率产生的 0.40 s：

- 正常切换总延迟：`0.99 s`
- 正常切换所需前向余量（含 0.25 m margin）：`2.2485 m`
- 再叠加 0.50 s command timeout 的故障余量：`2.9485 m`

这些数值只是 future qualification 的设计门，不是安全认证。后续每个非控制速度档必须先通过：

1. 制动 probe；
2. 高速到 conflict overlay 的切换 probe；
3. Ackermann circle probe；
4. compiled scene forward-free-path 对 `2.9485 m` fault envelope 的审计；
5. collision contact、footprint clearance 与 raw trace replay；
6. transaction/command-timeout fail-closed probe。

任一档失败都必须保留，不能重试、替换 seed 或删除。

## 4. 公平基线

`TEB-Default` 只作为透明历史参考，不能充当主要的“弱基线”。

calibration 必须同时运行：

- `TEB-Tuned-V1`：精确复用 T08 冻结工程基线；
- 固定 TEB：只在相同 `max_vel_x` grid 上独立调优；
- circle-contact control/candidates：使用同一速度 grid。

固定 TEB 候选与 circle-contact 候选分别在 calibration-only 数据上按相同的
success/collision/emergency/clearance/integrity 资格门筛选。独立固定 TEB 取合格者中
equal-role-weighted restricted completion time 最短者；主公平基线再从
`TEB-Tuned-V1` 与该独立固定 TEB 中取安全合格且最快者。1% 内并列时采用更保守的预注册
tie rule。基线身份必须在 held-out test 打开前冻结，test 或 P1 结果不得参与选择。

最终阳性门不是“只比 circle-contact v1.20 快”：

- 主效果必须相对 calibration-frozen 最强公平固定基线；
- 还必须同时证明相对 circle-contact v1.20 的单因素因果效果；
- 两个 estimand 都要求点估计至少降低 `10%`、单侧置信下界至少降低 `5%`。

## 5. 全局有限预算、split 与误差控制

整个 post-P1 性能搜索最多三轮：

| 轮次 | 因素 | 新仿真证据上限 | 训练环境步上限 | 单侧 α |
| --- | --- | ---: | ---: | ---: |
| 1 | 当前 `max_vel_x` 单因素 | 792 | 0 | 0.025 |
| 2 | 未来 trace 驱动的单因素 | 704 | 60,000 | 0.015 |
| 3 | 未来 trace 驱动的单因素 | 704 | 60,000 | 0.010 |
| 合计 | 最多 3 个因素 | 2,200 | 120,000 | 0.050 |

未使用预算或 α 不得回收、跨轮转移或扩容；首次完整阳性后必须停止性能搜索。若某轮失败，
只能冻结阴性结果，再根据原始 trace 登记一个新的单因素，不能同时改多个不可归因因素。

Round 1 的 fresh seed firewall：

- calibration：5301--5330，三类 scene role 各 10 blocks；
- qualification：5331--5342，4 个非控制速度 × 3 类 probe；
- compile-support only：5343--5348，不计证据；
- held-out test：5401--5490，三类 role 各 30 blocks；
- calibration 11 个 unique methods，共 330 units；
- qualification 12 units；
- held-out test 最多 5 个 unique methods，共 450 units；
- Round 1 总上限 `792 units`；
- 每个 frozen identity 只允许 attempt=1，不允许 retry、resume、replacement 或 seed substitution。

D2 本身没有训练；如果未来某个单因素涉及 RL，必须使用后来轮次的独立 training/validation/test
split、新 seed、版本化 action domain 和单独 release，不能使用本轮 held-out test 做 checkpoint
或阈值选择。

## 6. 最终阳性门

所有条件必须同时成立：

- exact matrix、journal、failure index、raw resource hash 与 deterministic replay 完整；
- 未解释 crash / integrity failure = 0；
- 无删除、替换或重试 identity；
- success、collision、emergency 相对主公平基线非劣；
- 每个 role 不得出现 treatment harmful-discordance excess；
- clearance difference 单侧下界不低于 `-0.05 m`，各 role 点估计也不低于 `-0.05 m`；
- normal projection rate 严格小于 `10%`；
- true-conflict TTC status match fraction 至少 `95%`；
- invalid parameter transaction = 0；
- 相对最强公平基线和相对 circle-contact v1.20 的两个效率门均达到 `10% / 5%`；
- Round-1 familywise one-sided α=`0.025`，固定 gatekeeping 后 secondary 使用 Holm。

任何一项失败，结论固定为
`obvious_performance_improvement_not_demonstrated`，不得事后降低门槛。

## 7. 机器证据

主要文件与 SHA256：

- 合同：
  `config/thesis_experiments/v2/v2_04g_r6_i1_p2_d2_offline_single_factor_design_contract.yaml`
  — `962d7f30eae0955972518ffec960fbf892eaf5f40b8bedb61e3cb5867bbf87ef`
- 核心：
  `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d2.py`
  — `2e79ae34b5f8a581df404494222591480eb315c00777ee40027b984843712d02`
- candidate amendment：
  `artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design/v2_04g_p2_d2_candidate_bank_amendment.yaml`
  — `de077b15ed2d6e57556fb3d7a04cd3c0a116edb9a06b626abe915cd379fa43da`
- design report：
  `artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design/v2_04g_p2_d2_design_report.yaml`
  — `f6be3e25757e95abeff384a56e7e87938001b73df759a4125eafcdb6dced4882`
- independent review：
  `artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design/v2_04g_p2_d2_offline_review.yaml`
  — `dc9b3e4b8a2a636e14b84ece5a2268f75f3c303b00957900aaca13b7cdf8a4df`
- source evidence freeze：
  `artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design/p2_d2_source_evidence_freeze.sha256`
  — `fe0d4dd6ecc9048bae25761255db4575a57e5170a347dae44d9784af5850c93d`

验证命令：

```bash
cd /home/robot/robot_ws_base_rl
python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i1_p2_d2.py
python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p2_d2_offline_designer.py \
  --check-only
python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p2_d2_offline_reviewer.py \
  --check-only
```

当前结果：`5 passed`，designer 与 reviewer 均 byte-equal check pass。

## 8. 下一安全入口

下一阶段必须是新的纯离线 `P2-Q1`（名称可版本化，但不能复用历史 stage identity）：

1. 在新 artifact/config namespace 物化 5 档 circle-contact 与 fixed-TEB candidate banks；
2. 对每个 bank 执行 exact nonfactor diff audit，活动 runtime 配置仍不改；
3. 物化 5301--5348 的 calibration/qualification scenes；
4. 实现 exact 342-row calibration+qualification schedule、runner、raw replay adapter、
   journal/failure-index schema 与终止策略；
5. 完成 independent offline review，确认 held-out 5401--5490 仍不存在且不可访问；
6. 此时仍不得执行模拟。

只有上述离线闭环通过后，才能创建唯一的 frozen calibration release 和 exact 342-unit budget，
并通过对应阶段的单独仿真授权门。calibration 完成并冻结 candidate/baseline 后，held-out test
还必须另建唯一 release、exact schedule/budget 和单独授权，不能把当前持续指令解释成一个
尚不存在 release 的通配执行许可。
