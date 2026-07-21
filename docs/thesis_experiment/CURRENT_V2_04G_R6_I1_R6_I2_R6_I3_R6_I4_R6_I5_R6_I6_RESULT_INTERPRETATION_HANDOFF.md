# V2-04G-R6-I6 纯离线结果解释与设计闭环交接

日期：2026-07-21

## 1. 阶段结论

V2-04G-R6-I6 已完成，状态为：

```text
offline_result_interpretation_design_closure_pass
```

本阶段只读取、校验和解释既有证据，并预注册一项未来性能实验设计。它没有启动 ROS、Gazebo、`move_base` 或训练进程，没有写入在线 TEB 参数，没有创建执行 release、seed schedule、attempt root 或 journal，也没有消耗任何执行或训练预算。

I5 保持终态且不可重跑、不可续跑：

- release SHA-256：`9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43`
- assessment report SHA-256：`8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599`
- 已完成单元：`6/6`
- retry / resume / forfeiture：`0 / 0 / 0`
- fresh execution seeds：`5161, 5162, 5163`
- 最终判定：`simulation_integration_validation_pass`
- `formal_result=false`
- `runtime_ready=false`

## 2. 冻结证据

关键证据冻结清单：

```text
artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/i5_evidence_freeze.sha256
SHA-256 40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935
```

清单固定了 13 项关键资源，包括 I5 release、preregistration、authorization、dependency closure、readiness review、stage report、execution report，以及 R5、TTC-D1、R6-I1、失败 I3 release 和 I4 review 的历史证据。`sha256sum -c` 对 13 项全部通过。

这些文件的含义是保全历史链，而不是把失败阶段改写成成功阶段：

- R5 仍为终止且无 winner；
- TTC-D1 仍为纯离线诊断；
- R6-I1 仍因模拟时钟 bootstrap 顺序死锁而终止；
- I3 的失败 release 仍被原样保留；
- I4 仍为无执行授权的离线修复闭环；
- I5 仍是唯一已完成的六单元新鲜种子集成执行。

## 3. I6 合同与机器复核

离线解释合同：

```text
config/thesis_experiments/v2/
  v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_r6_i6_offline_result_interpretation_design_contract.yaml
SHA-256 d140ff1e11f6b8c2f77e015a3da726a973253261c8ddcebe5180b513c31d5613
```

确定性复核器：

```text
src/tools/thesis_experiment/scripts/
  v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_r6_i6_result_interpretation_reviewer.py
SHA-256 13414d948749fcd4a2bc044acb71b82bf0a5039873f103f3cf17b99eb15f3d9e
```

持久化复核报告：

```text
artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/
  v2_04g_r6_i6_result_interpretation_review.yaml
SHA-256 c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68
```

测试：

```text
src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/
  test_v2_04g_r6_i6_result_interpretation.py
SHA-256 7caf22e8fc0a81a3e4477636daec5a4d7c4042fd8d49c1c8450153ad09f38672
```

复核器重新验证：

- 合同和未来设计的封闭 schema；
- 14 项 source binding 的规范路径与 SHA-256；
- I5 六项 journal binding；
- I5 36 项 raw-resource binding；
- release readiness 的直接计数证据；
- 六个单元的有限 TTC / 非 `NONE` 样本计数；
- I5 执行报告中的终态、预算和隔离结论；
- 未来设计不存在执行授权、release 或预算；
- 持久化报告与当前确定性重建结果逐字段相等。

## 4. I5 的直接观察

按冻结 schedule 顺序，六个单元的语义样本计数为：

| 顺序 | 语义实现 | finite TTC | non-`NONE` conflict |
|---:|---|---:|---:|
| 1 | legacy control | 31 | 25 |
| 2 | circle-contact | 30 | 19 |
| 3 | circle-contact | 32 | 19 |
| 4 | legacy control | 28 | 30 |
| 5 | legacy control | 0 | 18 |
| 6 | circle-contact | 0 | 0 |

六个单元均完成导航和语义评估；预期/观察 TTC 状态逐行一致。semantic-clear 配对中，legacy control 产生 18 个非 `NONE` 冲突样本，而 circle-contact 为 0。该观察支持“语义实现差异能在完整执行链中被检测和记录”，但不能单独支持任何性能优越性结论。

## 5. 论文现在可以宣称的结论

以下表述有冻结证据直接支持：

1. I5 在新鲜模拟种子上完成了六个预注册的配对集成单元，未重试、未续跑、未发生终态失败。
2. 预注册的 expected / observed TTC 语义状态在全部六行一致。
3. journal、原始资源、直接计数、报告和终态隔离可由确定性 assessor 重放，且复核未发现完整性失败。
4. circle-contact TTC 语义与 legacy control 在受控模拟场景中的行为可区分。
5. 当前结果证明的是新鲜模拟条件下的语义/执行集成，不是形式化安全、部署就绪或性能提升。
6. 历史失败和失败 release 被保留；后续修复没有覆盖或重写先前证据。

建议论文用语：

> 在预注册的新鲜模拟种子集成验证中，所提出的 TTC 语义实现通过了执行链、事务、日志和确定性复核的一致性检查；该试验用于验证集成正确性，不用于估计导航性能效应。

## 6. 论文现在不可宣称的结论

以下表述均不受 I5 或 I6 支持：

1. circle-contact 语义提升了到达率、耗时、路径效率或最小间距；
2. 已证明统计显著的性能提升；
3. 已证明对未见场景、未见机器人或真实车辆的泛化；
4. 已证明无碰撞、形式化安全或风险上界；
5. 已选出 winner、已校准部署阈值或 `runtime_ready=true`；
6. 已授权训练、在线参数写入、真实车辆或新的模拟执行；
7. semantic-clear 单个计数差能够代替多种子配对性能统计；
8. I5 的 6 个集成单元构成独立同分布的性能样本。

违反这些边界的论文文字必须在提交前删除或改写。

## 7. 最后一轮性能实验的预注册设计

未来设计文件：

```text
experiments/manifests/v2/integration/
  v2_04g_r6_i6_future_multiseed_performance_design.yaml
SHA-256 a5b74aa99cb63785aa3993ba7cae40974baa3ab9b6aace71ee9cb815e08d379c
```

独立未来阶段名为 `V2-04G-P1`。当前仅完成设计预注册，不是执行授权：

- `execution_authorized=false`
- `evidence_budget_authorized=0`
- `additional_training_steps=0`
- `checkpoint=null`
- 不允许创建 release、journal、attempt root 或启动模拟

设计要点：

- confirmatory contrast：circle-contact 与 legacy control；
- contextual secondary：固定 TEB reference，仅作背景比较，不进入主确认性检验；
- 3 个预注册 scene roles；
- 每个 role 30 个全新 seed block；
- 共 90 个 scene-seed block；
- 每 block 运行 3 个方法，共 270 个计划 episode；
- 主确认性配对为 180 个 episode；
- 全新 seeds：5201--5290；编译支持 seeds：5291--5294；
- 不允许 retry、resume、replacement 或样本量扩张；
- 不增加训练预算，不训练或挑选新 checkpoint。

主性能量为带超时惩罚的受限完成时间，采用 scene role 分层的配对 log-ratio；碰撞/成功率、最小间距和真实冲突保持构成先行非劣门。只有完整性门、安全门、主效率门和机制门全部通过，才允许写“在预注册的新鲜模拟动态交互分布上性能提升”。

设计中的 30 对/role 来自显式功效假设（配对 log-time 标准差 0.15、目标效应 10%、单侧 alpha 0.05、power 0.90），不是从 I5 估计出的效应量。二元安全非劣门使用 role-stratified 的 90-block pooled interval，并要求每个 role 都不存在 treatment harmful-discordance excess；若零 harmful discordance，90-block 和 60-conflict-block 的单侧精确上界约为 3.28% 和 4.87%，因此 5 percentage-point margin 在极低伤害差异下是可达到而非自动通过的。未来实际结果可能通过，也可能失败；预注册设计本身不保证性能提升。

## 8. 后续唯一安全入口

在任何性能执行之前，必须另行完成：

1. 独立设计审查，确认指标可计算、scene derivation 固定且 fresh-seed firewall 有效；
2. 生成新的封闭依赖 release；
3. 分离的、明确的用户模拟执行授权；
4. 新预算与不可复用 seed schedule；
5. prejournal 全量校验通过；
6. 执行结束后的独立确定性评估和论文 claim gate。

不得把本交接、I6 合同或未来设计文件解释为执行授权。不得重跑 I5，不得复用 5161--5163，不得复用历史失败阶段的身份或预算，也不得以“论文必须成功”为由改动停止规则、替换失败样本或事后扩大样本量。

## 9. 离线复核命令

```bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_r6_i6_result_interpretation_reviewer.py \
  --workspace /home/robot/robot_ws_base_rl \
  --check-only

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i6_result_interpretation.py

sha256sum -c \
  artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/i5_evidence_freeze.sha256
```

预期结果分别为：

```text
offline_result_interpretation_design_closure_pass
4 passed
13/13 成功
```
