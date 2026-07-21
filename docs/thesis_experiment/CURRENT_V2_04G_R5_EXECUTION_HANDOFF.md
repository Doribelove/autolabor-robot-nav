# CURRENT V2-04G-R5 EXECUTION HANDOFF

更新时间：2026-07-18

## 结论

V2-04G-R5 获得的 bounded Gazebo calibration 授权已按 fail-closed 合同结束。

第一个 readiness identity
`r5-readiness-r5_ttc_h450-s5111` 只执行了 1 次。transaction、activation、join、
atomic input 和导航接口全部通过，但冻结 evaluator 期望的 TTC 状态未覆盖：

```text
expected: OBSERVED_CONFLICT
observed: NO_CONFLICT_IN_HORIZON
finite_ttc_sample_count: 0
minimum_predicted_ttc_s: null
```

因此 readiness 在 `1/6` 立即终止，未执行剩余 5 个 readiness、3 个 TTC
component identity 或 60 个 navigation episode。禁止重试或 resume。R5 没有
passing candidate，没有 winner，未执行 ranking/freeze/held-out/V2-05/SAC/实车。

机器可读结论：

- `artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_stage_report.yaml`
- `artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_assessment.yaml`

stage report SHA256：
`d0da836c0a80506a8edd227ce057b46eef815b9e23e59555ed4cd585174d7077`。

## 冻结起点与执行前核验

执行前重新运行了 R5 validator，并核验：

```text
preregistration SHA256:
0adcfd6a7a686b799b6dc55394cdf1e90fa140cee636d4283e0fb807f14134c6

dry-run audit SHA256:
d7a3113c89b08889dc754a72f4e792c422225f19504ab3218d9712cf46dee8e1

declared frozen resources: 39/39 hash match
ROS/Gazebo/training processes before execution: 0
```

分支仍为 `base_rl`，本地领先 `origin/base_rl` 1 个 commit。既有 dirty 子模块未清理、
未 reset/checkout；未修改 `/home/robot/robot_ws`；未 commit 或 push。

## 唯一修改因素与候选

唯一行为因素保持：

```text
supervisor.dynamic.predicted_ttc_max_s
```

候选保持：

| profile | 值 | 角色 | winner eligible |
| --- | ---: | --- | --- |
| `r5_ttc_control_h500` | 5.0 s | m030 timing control | 否 |
| `r5_ttc_h450` | 4.5 s | TTC timing repair candidate | 是 |
| `r5_ttc_h400` | 4.0 s | TTC timing repair candidate | 是 |

R4-R1 的 Maneuver `min_obstacle_dist=0.30 m` 仍仅为固定、非排名输入，不是系统
winner。材料化配置的行为差异只有上述 runtime field；Anchor、mechanism 和其余
supervisor 字段未变。

## 预算与实际消费

冻结预算：

```text
6 readiness + 3 TTC component + 60 navigation = 69 evidence units
```

实际：

```text
readiness attempted: 1
TTC component attempted: 0
navigation attempted: 0
total attempted: 1 / 69
remaining 68 units: terminal stop 后作废，不得继续消费
```

只消费 calibration seed `5111`。readiness seeds `5112--5113`、navigation seeds
`5121--5135` 未运行；compile-support-only seeds `5114--5117` 未运行；held-out
`5001--5010` 未消费。

## 终止 identity 的证据

identity：

```text
sequence: 1
profile: r5_ttc_h450
scene: v2-04g-r5-readiness-dynamic-conflict-s5111
seed: 5111
attempt: 1
```

通过的接口证据：

```text
activation listener hard gates: PASS
transaction messages: 30
transaction valid fraction: 1.0
transaction activated fraction: 1.0
mechanism messages: 30
join valid fraction: 1.0
world-model sequence mismatch: 0
world-model input join fault: 0
backend/unknown transaction fault: 0 / 0
navigation termination: SUCCESS
collision: false
minimum clearance: 1.661492443468028 m
tracker health valid fraction: 1.0
```

失败门：

```text
expected TTC: OBSERVED_CONFLICT
observed TTC: NO_CONFLICT_IN_HORIZON
tracker messages: 290
finite TTC samples: 0
```

运行期仍观察到非 `NONE` overlay：`CROSSING=14`、
`OVERTAKE_OR_YIELD=4`。这证明 runtime overlay 曾激活，但不满足冻结 evaluator 的
finite-TTC coverage 合同。不能把 overlay 激活解释为 readiness TTC gate 通过。

保留证据：

- readiness summary SHA256：
  `5c74ead279ed1201cd91e2e9a85c9c00a81323b56dbbc1d87245cf930d8e55a1`
- activation report SHA256：
  `5a58fc20fb977e30010032116ce537e9b51e3d49f976f790a219fdd3c1e57357`
- evaluation SHA256：
  `638734a495298bc65217b19bc0507cbf74c85e4b709f58fda24913091d99bd9e`
- trace SHA256：
  `e23990fbfd6e3f80f6abe8f841b90eb096204cbdf55d0d66ca52bc28f99b7bbc`
- assessment SHA256：
  `6d0b2a9b8456fbb75c9b2497f135bebcbbeecea370babbf636b1cd95a2d46b7e`

全部 episode、log、trace、失败 seed 和不利结果保留。

## 实现与测试

新增的 bounded execution/assessment 层包括：

- `v2_04g_r5_execution_guard.py`
- `v2_04g_r5_activation_probe_listener.py`
- `v2_04g_r5_readiness_batch.py`
- `probe_v2_04g_r5_ttc_states.py`
- `v2_04g_r5_mechanism_episode.py`
- `v2_04g_r5_bounded_calibration.py`
- `assess_v2_04g_r5.py`

这些 wrapper 固定 exact schedule、attempt=1、原子 attempt ledger、terminal
no-resume、seed firewall 和 downstream authorization=false。旧 R4/R4-R1 中的 retry、
inventory/resume 和覆盖日志语义没有复用。

最终离线测试：

```text
R5 design/execution/assessment: 40 passed
all V2 tests: 142 passed
```

## Assessment

最终 assessment：

```text
status: calibration_terminally_stopped
attempted evidence units: 1
valid combined readiness identities: 0
ranking_performed: false
qualified_candidate_ids: []
winner_candidate_id: null
```

freeze、held-out、V2-05、SAC、实车运动和实车参数写入授权全部为 `false`。

## 下一步边界

按预注册 stopping rule，本阶段已结束并保持 `no winner`。现有授权下没有下一个
Gazebo 动作：

- 不得重试 seed 5111；
- 不得从 readiness 2/6 resume；
- 不得消费剩余 68 evidence units；
- 不得执行 TTC component 或 navigation 60；
- 不得复用/替换 seed、放宽 TTC 门槛或事后重标场景；
- 不得 freeze、held-out、V2-05、SAC 或实车。

如果用户希望继续研究，建议另开一个独立、offline-only、diagnosis-only 的阶段，只读
解释以下冻结差异：

```text
healthy tracker + non-NONE runtime overlay
versus
finite_ttc_sample_count=0 + evaluator NO_CONFLICT_IN_HORIZON
```

该诊断不得重启 R5、不得运行新 Gazebo、不得改 threshold/scene/evaluator。只有诊断审查
后，用户才能决定是否预注册一个全新的、fresh-seed 阶段；不能把它当作 R5 resume。
