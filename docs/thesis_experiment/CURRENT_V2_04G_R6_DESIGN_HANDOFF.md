# CURRENT V2-04G-R6 DESIGN HANDOFF

更新时间：2026-07-19

## 1. 阶段结论

`V2-04G-R6-DESIGN` 的独立 design/preregistration review 已建立。权威机器结论为：

`artifacts/v2/design_review/v2_04g_r6/v2_04g_r6_design_review.yaml`

当前 machine review 状态为：

```text
status: design_preregistration_review_pass_execution_not_authorized
review_result: pass
design_only: true
offline_only: true
formal_result: false
runtime_ready: false
execution_ready: false
execution_authorized: false
```

本阶段只定义并离线审查一个 runtime/evaluator 语义对齐的 categorical factor，同时为
D1 发现的六项执行完整性风险实现纯 Python、candidate-symmetric 的 fail-closed
协议和单元测试。本阶段没有执行证据，因此六项修复的最高结论只能是：

```text
design_fix_status: OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED
execution_validation_status: NOT_RUN_NOT_AUTHORIZED
```

不得把该结论解释为 ROS/Gazebo 集成已验证、execution ready、性能有效或安全性已证明。

## 2. 唯一 categorical factor

唯一允许比较的 profile 字段为：

```text
supervisor.dynamic.conflict_estimator_id
```

两个 design level 为：

| Candidate ID | Factor value | 角色 | winner eligible |
| --- | --- | --- | --- |
| `r6_semantics_legacy_control` | `legacy_class_conditioned_geometry_v1` | 冻结 legacy 语义 control | 否 |
| `r6_semantics_circle_contact` | `shared_circle_envelope_first_contact_v1` | evaluator-aligned design candidate | 否 |

本轮把以下三部分共同定义为一个不可拆分的 categorical estimator identity：

1. conflict eligibility primitive；
2. tracked footprint radius interpretation；
3. multi-track conflict selection order。

它们不是三个可独立调节的因素。任何把 footprint 规则、multi-track 排序、TTC horizon
或其他阈值独立改变的设计都会破坏本轮单因素合同。

### 2.1 Legacy control

legacy level 保持冻结行为：

- `CROSSING` 使用中心线交叉时间；
- 其他动态类使用 point closest approach；
- footprint radius 使用 footprint 点的最大绝对 `x` extent；
- 原 overlay taxonomy、priority 和 release 逻辑不变。

### 2.2 Aligned candidate

aligned level 直接复用冻结 evaluator producer 的：

```text
nav_world_model.risk_evidence.relative_collision_ttc
```

其语义为：

- runtime track 位于当前机器人坐标系；
- `vx/vy` 是相对速度；
- finite circle-envelope first-contact TTC 是 non-`NONE` conflict eligibility
  的必要且充分条件；
- 没有 finite TTC 时必须输出 `NONE`；
- motion class 只决定 finite TTC 已成立后的 overlay label，不能再单独触发 conflict；
- `STATIONARY` 和 `DEPARTING` 保持排除；
- runtime 不得读取 scene label、Gazebo truth 或 evaluator truth。

## 3. 冻结数值、footprint 与 multi-track 原子定义

两个 factor level 共同冻结：

```text
runtime/evaluator TTC horizon:       5.0 s
minimum track confidence:            0.45
robot circle radius:                 0.62 m
minimum relative speed:              0.05 m/s
world-model classification horizon:  2.0 s
legacy closest-approach threshold:   1.35 m
overlay release confirmation:        0.20 s
```

本轮不启用 D1 曾离线讨论的 `1.5 s` 或 `1.0 s` horizon，也不修改 motion classifier、
transition/dwell、Anchor、TEB 参数、mechanism、join、evaluator 或 scene。

### 3.1 Footprint radius

未来 R6 runtime adapter 必须接收仍带有原始 footprint 点集的 track，不能接受已经由
legacy node 预先压缩好的 radius：

- legacy level：`max(abs(x_i))`；
- aligned level：`max(hypot(x_i, y_i))`，即 footprint circumradius；
- 空 footprint 的共同冻结 fallback：`0.25 m`；
- 禁止从 scene truth 或硬编码 actor 尺寸取得 runtime radius。

对于 D1 的 `0.55 m x 0.55 m` 方形 actor，machine review 复核：

```text
legacy radius:   0.275 m
aligned radius:  0.3889087296526012 m
```

aligned 值与冻结 evaluator 的 actor circle radius 语义一致。

### 3.2 Multi-track selection

aligned level 对每条合格 track 调用同一个 circle-contact primitive，然后按以下完整
tuple 选取唯一冲突：

```text
(earliest finite TTC,
 frozen overlay priority,
 track_id)
```

冻结 overlay priority 为：

```text
HEAD_ON < CROSSING < FOLLOW < OVERTAKE_OR_YIELD
```

因此相同 TTC 时先按既有 overlay priority，再按 `track_id` 确定性决胜。该顺序是
categorical factor 的原子组成部分，不能在未来作为另一个调参维度。

## 4. D1 frozen replay 的设计辨识度

R6 review 只读使用 D1 seed5111 frozen trace 作为 design input：

```text
trace rows:                         193
legacy proxy non-NONE rows:          25
shared-circle finite-TTC rows:        0
expected changed rows:               25
```

25 个 legacy non-`NONE` row 由 `21 CROSSING + 4 OVERTAKE_OR_YIELD` 组成。aligned
定义在同一 frozen trace 上得到 0 个 finite TTC，因此预计这 25 行全部从 legacy
non-`NONE` 变为 aligned `NONE`。

这只证明 proposed categorical factor 在该冻结输入上具有 offline design
identifiability。它不是新 evidence：

- seed5111 没有被重新消费；
- evidence unit 消费为 0；
- 不能推断 future fresh scenes 的 coverage；
- 不能推断安全、性能、泛化或 winner。

## 5. 六项执行完整性修复

六项协议对 control 和 aligned candidate 完全对称，不构成额外实验因素。

| D1 risk | Offline protocol | 当前状态 |
| --- | --- | --- |
| `D1-RISK-READINESS-DIRECT-COUNTS` | activation/evaluation identity 直接绑定，并分别硬检 tracker/context 至少 20 条；aggregate boolean 不足以代替原始计数 | `OFFLINE_PROTOCOL_IMPLEMENTED_AND_UNIT_VERIFIED` / `NOT_RUN_NOT_AUTHORIZED` |
| `D1-RISK-COMPILED-SCENE-TOCTOU` | index 与 child hash 校验、单次捕获 source bytes、attempt-local content-addressed snapshot、exclusive create、pre-spawn/post-episode revalidation，command 只能引用绑定 snapshot | 同上 |
| `D1-RISK-SIGINT-IN-PROGRESS` | 完整 identity 派生的原子 journal；`KeyboardInterrupt -> terminal_interrupted`；orphan -> `terminal_unclean_shutdown`；唯一 canonical journal root 内同 identity 并发和 resume 均拒绝 | 同上 |
| `D1-RISK-ASSESSMENT-RAW-BINDING` | assessor 必须遍历 attempt ledger，并直接绑定 activation、evaluation、trace、clearance、log 与 teardown；缺失证据必须明确记录且匹配当前 journal lifecycle，预附 terminal bundle 后禁止继续推进；post-episode terminal 必须绑定六项 produced raw evidence | 同上 |
| `D1-RISK-EXECUTION-HASH-CLOSURE` | future entrypoint 必须机械生成 Python、dynamic load、launch、config、scene 全依赖闭包；missing、unreachable 或 unresolved dependency 均 fail closed | 同上 |
| `D1-RISK-TEARDOWN-RESTORE` | backend 存活时请求 restore，要求 transaction ack/readback 与独立 final readback，精确匹配 startup profile；成功 token 与完整 identity、active journal provenance 绑定后才允许停止 launch | 同上 |

当前没有 R6 execution entrypoint，因此没有持久化 execution dependency manifest；
现有 closure fixture 只是 prospective offline protocol 测试。未来 node/runner 一旦出现，
必须重新机械生成 closure，并由 execution guard 在创建 ledger 或 subprocess 之前验证。

teardown fixture 也没有真实 execution receipt。未来如果导航目标成功但 startup profile
恢复失败，attempt 必须终止为：

```text
terminal_teardown_failure
```

不能只保留“导航成功”结论，也不能在 restore 完成前关闭 launch。

## 6. 本阶段明确没有创建的内容

当前机器边界为：

```text
execution authorization artifacts:  0
seed schedule present:               false
seed values:                         []
evidence budget authorized:          0
seed/evidence units consumed:        0
persisted runtime candidate configs: 0
scene manifests/compiled scenes:     0
ROS nodes/launch files:              0
episode runners/batches:             0
execution dependency manifest:       absent
```

新增的 `r6_relative_ttc_supervisor.py` 是未接入 ROS node/launch 的纯设计参考实现；
`v2_04g_r6_integrity.py` 是未接入 executor/assessor 的纯协议实现。它们本身不构成
runtime config、execution entrypoint 或执行授权。

## 7. R5 终止状态和持续禁令

R5 仍保持 terminally stopped：

- 唯一已尝试 identity 为 `r5-readiness-r5_ttc_h450-s5111`，`attempt=1`；
- expected 为 `OBSERVED_CONFLICT`，observed 为 `NO_CONFLICT_IN_HORIZON`；
- R5 只消费 `1/69` evidence units；
- 剩余 68 units 已作废；
- component 为 `0/3`，navigation 为 `0/60`；
- passing candidate、ranking 和 winner 均不存在。

继续禁止：

- retry 或 resume R5 seed5111；
- 消费 R5 剩余 68 units；
- 执行 R5 component 或 navigation；
- 替换 seed、删除失败证据或事后修改 gate/label；
- freeze winner；
- 使用 held-out seeds `5001--5010`；
- 启动 V2-05、SAC 或任何训练；
- 连接或驱动实车；
- 写实车 TEB 参数。

R6 design review 不能作为解除任何 R5 stopping rule 的依据。

## 8. R6 design/prereg 文件

本阶段的权威文件为：

- `config/thesis_experiments/v2/v2_04g_r6_semantic_alignment_design_contract.yaml`
- `experiments/manifests/v2/preregistrations/v2_04g_r6_semantic_alignment_preregistration.yaml`
- `experiments/manifests/v2/preregistrations/v2_04g_r6_semantic_candidates.yaml`
- `src/application/teb_mode_manager/src/teb_mode_manager/r6_relative_ttc_supervisor.py`
- `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_integrity.py`
- `src/tools/thesis_experiment/scripts/review_v2_04g_r6.py`
- `src/tools/thesis_experiment/tests/test_v2_04g_r6.py`
- `artifacts/v2/design_review/v2_04g_r6/v2_04g_r6_design_review.yaml`
- `docs/thesis_experiment/CURRENT_V2_04G_R6_DESIGN_HANDOFF.md`

资源数量、测试计数和最终 SHA256 以 reviewer 生成的 machine report 及最终验证输出为准；
本 handoff 不预先冻结可能在最终审查中更新的计数或 digest。

## 9. 后续唯一合法入口

当前没有获准的 R6 execution 动作。任何 future R6 execution 必须先经过另一轮独立审查，
并取得用户新的明确授权。该后续阶段至少需要：

1. 分配全新的 scene/seed firewall 和明确 evidence budget，不得复用 seed5111 或
   R5 剩余 68 units；
2. 创建并审查实际 runtime config、scene、ROS node/launch、runner、batch 和 assessor；
3. 把本轮六项 offline protocol 接入真实 call site，并完成离线 integration tests；
4. 从实际 entrypoint 机械生成完整 execution dependency closure；
5. 将 node/runner/assessor、scene/index/child、config、launch 和全部 transitive
   dependency 的 SHA256 绑定到新的 preregistration；
6. 保持 held-out `5001--5010` 不可见；
7. 另行创建 execution authorization，并让 guard 精确绑定本 design review、
   future execution preregistration 和完整 dependency closure；
8. 在授权前继续保持所有 execution、ROS/Gazebo、seed consumption、winner freeze、
   V2-05、training 和 real-vehicle 字段为 `false`。

即使未来完成上述准备，本 handoff 和当前 machine report 也不会自动授权启动
ROS/Gazebo；仍必须等待用户独立、明确的 execution 指令。
