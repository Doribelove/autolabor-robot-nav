# 第 5 章 I5 新鲜种子语义/执行集成结果

## 5.1 实验目的

I5 是一项有界、六单元、纯模拟的集成验证。它回答的问题是：在完成 bootstrap、参数事务、导航、语义采集、证据落盘和 teardown 的整条运行链中，`circle-contact` 与 `legacy control` 两种 TTC 语义能否产生预注册且可确定性重放的状态。该实验不以完成时间、路径长度、成功率差或最小间距差为确认性终点，因此不具备证明性能提升的设计效力。

该定位尤其重要。此前单因子校准曾在 crossing 场景中观察到零个有限 TTC；离线诊断随后发现参与者比机器人早约 4.0053 s 到达交叉点，冻结的 5.0/4.5/4.0 s horizon 不能区分 21 个可达 crossing 样本。后续第一次集成执行又在语义 episode 前因暂停模拟时钟的 bootstrap 顺序死锁而终止。I5 不是删除这些失败，而是在保留其字节和身份的前提下，用新的 stage、release、种子和预算检验修复后的完整链。

## 5.2 预注册设计

### 5.2.1 因素与场景角色

唯一受控语义因素为：

- `r6_semantics_legacy_control`：历史控制语义；
- `r6_semantics_circle_contact`：基于圆接触的 TTC 语义。

场景包括三个角色：单一动态冲突、多动态冲突和时间分离的 semantic-clear crossing。每个角色使用一个全新执行 seed，并让两种语义实现共享同一场景实现，形成配对。为降低固定顺序与方法的混淆，三对的运行顺序交替安排。

### 5.2.2 精确调度

| 顺序 | 场景角色 | 方法 | seed | 预期 TTC 状态 | attempt 上限 |
|---:|---|---|---:|---|---:|
| 1 | single conflict | legacy control | 5161 | `OBSERVED_CONFLICT` | 1 |
| 2 | single conflict | circle-contact | 5161 | `OBSERVED_CONFLICT` | 1 |
| 3 | multi conflict | circle-contact | 5162 | `OBSERVED_CONFLICT` | 1 |
| 4 | multi conflict | legacy control | 5162 | `OBSERVED_CONFLICT` | 1 |
| 5 | semantic-clear | legacy control | 5163 | `NO_CONFLICT_IN_HORIZON` | 1 |
| 6 | semantic-clear | circle-contact | 5163 | `NO_CONFLICT_IN_HORIZON` | 1 |

精确 schedule SHA-256 为 `b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402`。授权证据预算为 6 个单元；不允许重试、续跑、替代 seed 或用历史剩余预算补充。5164--5167 只用于离线编译支持，不构成实验数据。

### 5.2.3 成功判定

预注册成功判定要求：

1. release、authorization 和依赖闭包在执行前通过；
2. `/clock` 正进展后才能等待导航服务；
3. 每个身份只执行 attempt 1；
4. 激活、事务、导航、语义评估和 teardown 形成完整 journal；
5. 观察 TTC 状态与调度逐行一致；
6. readiness 与语义计数可从绑定原始资源直接重算；
7. 全部终态资源可由确定性 assessor 重放；
8. 最终进程隔离清洁。

其中没有“circle-contact 的耗时必须更短”或“成功率必须更高”这一条件。因此通过该 gate 只能得到集成结论。

## 5.3 执行完整性结果

I5 使用唯一 release，其 SHA-256 为：

```text
9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43
```

六个计划身份全部在 attempt 1 达到 `evidence_complete`：

- planned / completed：`6 / 6`；
- evidence units authorized / consumed：`6 / 6`；
- unattempted forfeiture：`0`；
- retry count：`0`；
- resume：`false`；
- terminal failure：无；
- training started：`false`；
- real vehicle used：`false`。

每个 episode 的场景快照均在运行后复核，参数恢复和两阶段 teardown 均通过。最终检查未发现遗留 ROS、Gazebo 或导航进程。

### 5.3.1 readiness 直接计数

预注册要求关键 readiness stream 至少有 20 个直接样本。assessor 从原始资源重算得到：

| 顺序 | activation tracker | activation context | evaluation tracker | evaluation context | readiness |
|---:|---:|---:|---:|---:|---|
| 1 | 89 | 30 | 418 | 140 | pass |
| 2 | 88 | 30 | 318 | 107 | pass |
| 3 | 88 | 30 | 1181 | 400 | pass |
| 4 | 88 | 30 | 319 | 107 | pass |
| 5 | 89 | 30 | 292 | 98 | pass |
| 6 | 89 | 30 | 280 | 94 | pass |

所有直接计数均高于最低要求。该表说明评估基于持续数据流，而不是 topic 名称存在或单个样本；计数大小本身不是性能指标。

## 5.4 TTC 语义结果

六个单元的重放结果如下。

| 顺序 | 场景角色 | 方法 | observed TTC | finite TTC samples | non-`NONE` overlays | 与预期一致 |
|---:|---|---|---|---:|---:|---|
| 1 | single conflict | legacy control | `OBSERVED_CONFLICT` | 31 | 25 | 是 |
| 2 | single conflict | circle-contact | `OBSERVED_CONFLICT` | 30 | 19 | 是 |
| 3 | multi conflict | circle-contact | `OBSERVED_CONFLICT` | 32 | 19 | 是 |
| 4 | multi conflict | legacy control | `OBSERVED_CONFLICT` | 28 | 30 | 是 |
| 5 | semantic-clear | legacy control | `NO_CONFLICT_IN_HORIZON` | 0 | 18 | 是 |
| 6 | semantic-clear | circle-contact | `NO_CONFLICT_IN_HORIZON` | 0 | 0 | 是 |

预期/观察 TTC 状态在全部六行一致，`semantic_schedule_pass=true`。单冲突和多冲突两对中，两种语义均观察到有限 TTC；这说明 circle-contact 没有在所选真实冲突角色中丢失预期状态。semantic-clear 配对中，两种实现均没有有限 TTC，但 legacy control 仍生成 18 个非 `NONE` overlay，而 circle-contact 为 0。该差异展示了两种语义在无 horizon 内接触时的可辨识性。

不能直接用表中的 25 对 19、30 对 19 计算性能改善。不同 episode 的采样时长和消息数不同，这些是 episode 内相关的语义事件，不是独立样本。唯一合法的统计单位是未来实验中的 scene-seed block。

## 5.5 确定性重放

最终 assessor 逐项重放：

- 6 个 canonical journal；
- 每个 episode 的 activation、clearance、evaluation、process log、teardown receipt 和 trace，共 36 个 raw-resource binding；
- 场景快照、readiness 直接计数、TTC/overlay 计数、恢复和 teardown 状态。

重放结果为：

```text
status: simulation_integration_validation_pass
assessment_result: pass
integrity_failures: []
ttc_status_matches_preregistration: true
semantic_schedule_pass: true
readiness_direct_counts_pass: true
two_phase_teardown_restore_pass: true
integration_validation_pass: true
```

持久化 execution report SHA-256 为 `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599`。后续纯离线解释复核再次检查关键历史证据、全部 journal/raw bindings 和六行语义计数，并得到 `offline_result_interpretation_design_closure_pass`；其报告 SHA-256 为 `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68`。

## 5.6 结果解释

### 5.6.1 可支持的结论

I5 支持以下结论：

1. 修复后的 bootstrap、运行时绑定、参数事务、语义评估、journal 和 teardown 能在六个新鲜模拟执行身份上完成；
2. circle-contact 和 legacy control 的预注册 TTC 状态均可在完整运行链中复现；
3. 两种语义在 semantic-clear 条件下产生可辨识的 overlay 行为；
4. 关键结论可由绑定原始资源而非 runner 摘要独立重算；
5. 六个身份没有通过重试、续跑或 seed 替换筛选结果；
6. 失败阶段与失败 release 仍被保留，成功 I5 没有更改其历史身份。

论文可采用如下限定表述：

> 六个预注册的新鲜种子模拟单元均完成，TTC 状态与预期逐行一致，且 journal 与原始资源的确定性重放未发现完整性失败。这一结果验证了两种 TTC 语义在完整执行链中的集成与可辨识性。

### 5.6.2 不可支持的结论

I5 不支持：

- 导航到达率、时间、路径长度或最小间距改善；
- 统计显著的性能提升；
- circle-contact 在全部动态场景优于 legacy control；
- 未见分布或真实车辆泛化；
- 形式化无碰撞保证；
- 已完成阈值校准、winner 选择或部署准备；
- 任何训练效果或 SAC 收敛结论。

其原因并非“样本少”这一点而已，而是研究设计的终点就是语义/执行集成。即使六行全部通过，也没有预注册性能指标、足够独立配对 block 或性能 claim gate。

## 5.7 与历史失败链的关系

I5 的证据解释必须连同先前阶段报告：

1. **初始 TTC 校准失败**：预期冲突未进入预测窗口，有限 TTC 为零；没有 winner。
2. **离线诊断**：重现零有限 TTC，并发现参与者到达 crossing 比机器人早约 4.0053 s；说明 5.0/4.5/4.0 s horizon 在所选样本上缺乏辨识度。
3. **首次集成执行终止**：暂停模拟时钟导致服务等待先于解除暂停，语义执行前即超时。
4. **离线完整性修复**：加入正进展 `/clock` 屏障、规范路径+哈希闭包、严格授权、单次打开解析、确定性 assessor 和凭据安全环境。
5. **release 预检失败**：遗留 YAML 的整数 seed key 与过宽 parser 要求冲突；没有进入 execute。
6. **解析范围修复与 I5**：对精确资源名册重哈希，只解析需要语义理解的合同，随后用全新身份完成六单元执行。

这条链显示，实验失败既可能来自语义设计，也可能来自执行基础设施。保留两类失败使最终成功的适用范围更清晰：I5 证明修复后的链能工作，不证明早期假设或性能目标自动成立。

## 5.8 对最终性能实验的启示

I5 给最后一轮性能研究提供了三个设计约束，但不提供效应量：

1. semantic-clear 场景必须进入确认性样本，因为它能暴露无有限 TTC 时的错误 overlay；
2. single- 和 multi-conflict 场景必须作为安全保持条件，防止减少触发只是漏检真实冲突；
3. 结果必须以 scene-seed block 配对，而不能以高频 TTC 样本数扩大名义样本量。

未来设计固定 3 个 scene role、每个 30 个全新 block，共 90 个 block。每个 block 运行 circle-contact、legacy control 和固定 TEB reference，共 270 个 episode；主确认性对比使用 circle-contact 与 legacy control 的 180 个 episode。新增训练步数为 0。

主性能量为带失败/超时惩罚的受限完成时间。只有以下门同时通过，论文才可在限定的新鲜模拟分布上宣称性能提升：

1. 全部哈希、schedule、journal 和缺失值规则通过完整性门；
2. 成功率与碰撞率不超过 5 个百分点的预注册非劣界，最小间距差不低于 -0.05 m，并保持真实冲突语义；
3. 配对效率点估计至少改善 5%，且单侧置信下界高于 0、调整后 $p\le0.05$；
4. semantic-clear 错误冲突触发至少减少 30%，且单侧置信下界高于 0、调整后 $p\le0.05$。

成功/碰撞二元门在 90 个 block 上按 scene role 分层后汇总，并增加逐 role 的有害不一致方向保护；零 harmful discordance 时单侧精确上界约为 3.28%。真实冲突保持还单独覆盖 60 个 single/multi-conflict block，其零 harmful discordance 上界约为 4.87%。这使 5 percentage-point 非劣 margin 在极低伤害差异下具有可检验性，同时不允许某个场景角色的伤害被总平均掩盖。

如果任一门失败，结果应写为“未证实性能提升”或“存在安全/效率权衡”。不能因论文目标而重试失败 block、替换 seed、改变主指标或扩大样本量。

## 5.9 有效性限制

I5 的主要限制包括：

- 只有 3 个 scene-seed block，不足以估计跨种子性能分布；
- 语义消息是 episode 内相关观测，不能视为独立统计样本；
- 场景来自模拟器，感知、动力学和参与者行为与真实环境仍有差距；
- TTC 使用短时相对运动模型，无法覆盖模型外加速度与遮挡；
- 通过的是集成预期状态，不是最优阈值或最优 Anchor Bank；
- 固定 schedule 验证可复核性，但没有覆盖更广的环境组合。

## 5.10 本章小结

I5 在全新 seeds 5161--5163 上完成 6/6 个预注册模拟集成单元，无重试、续跑、作废预算或完整性失败。两种 TTC 语义在 single conflict、multi conflict 和 semantic-clear 三类场景中的观察状态与预期全部一致，且六个 journal 和 36 项原始资源可被确定性重放。该结果建立了语义/执行集成证据，同时明确保留性能结论为空。性能提升须由后续全新 multi-seed 配对统计实验独立检验。
