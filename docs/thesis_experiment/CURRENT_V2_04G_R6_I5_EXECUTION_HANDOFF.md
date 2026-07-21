# CURRENT V2-04G-R6-I5 BOUNDED SIMULATION EXECUTION HANDOFF

更新时间：2026-07-21

## 1. 权威结论

用户明确授权的独立 `V2-04G-R6-I5 bounded simulation execution` 已完成。唯一
canonical exact-hash release 先通过完整 prejournal gate，随后冻结的 6-unit paired schedule
以 `attempt=1` 按顺序全部执行完毕。deterministic assessor 对 6 份 journal 及其 raw evidence
逐项重放后给出：

```text
stage:                         V2-04G-R6-I5
runner exit:                   0
assessment status:             simulation_integration_validation_pass
assessment result:             pass
planned / completed units:     6 / 6
evidence consumed / authorized:6 / 6
terminal failure:              null
retry count / resume used:      0 / false
unattempted units forfeited:    0
integrity failures:             []
simulation only:                true
training / real vehicle:        false / false
formal_result / runtime_ready:  false / false
winner / downstream authority:  false / false
```

这个 `pass` 的最大含义是：fresh simulation runtime evaluator、semantic factor 和 execution
integrity 的冻结集成验证通过。它不是安全、性能、泛化、训练收敛或实车部署结论，不授权 winner
ranking/freezing、held-out seeds、V2-05、SAC/training、实车或在线 TEB 参数写入。

## 2. 唯一 release 与 prejournal

Canonical release 只创建一次，未覆盖、未重建：

```text
path:   experiments/manifests/v2/integration/v2_04g_r6_i5_execution_release.yaml
SHA256: 9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43
```

Release 绑定 29 个 exact path+SHA resources，包括 I5 contract/preregistration/authorization、
fresh scene index 与 14 children、execution runner、I5 validator/tests、I5 closure/review，以及
历史 I4 validator/closure/review 和 failed I3 release。完整 prejournal gate 在任何 attempt root、
journal 或 subprocess 前验证：

```text
bound resources:        29 / 29
authorization roster:   12 / 12
schedule identities:    6
schedule SHA256:         b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402
execution state before: absent
forbidden processes:    0
reserved ports:         clear
full prejournal result: pass
```

Canonical I5 offline closure/review 在 release 创建前冻结：

```text
closure file SHA256:    b351d892263adf00ec0b8362b0afd1b98b00342d327090b5171463227120d97d
closure logical SHA256: 3be6228193bdedad02900855e06af70b01150a36e0f271153aeb55fe93b839e3
machine review SHA256:  d4456bba057ee231aab7df77276df05407daaa14723a67a6658d3f49009b18c4
closure coverage:       134 local / 183 edges / 14 scene children
                        313 external / 49 Python / 9 runtime / 0 unresolved
offline directed tests: 88 passed
```

第一次外层 clean-shell 调用因 ROS setup 在 `set -u` 下读取未定义 `ROS_DISTRO` 而在进入 runner
前退出。随即只读确认 attempts/journals/reports 仍 absent、release SHA 未变、主机隔离干净，状态
仍为 0/6。之后仅在外层环境显式设置 `ROS_DISTRO=noetic`；release、runner、schedule、seed 和
budget 均未改变。由于 runner 从未被调用、没有 execution state 或 subprocess，这不是 unit
attempt、retry 或 resume。

## 3. 冻结 schedule 与结果

所有 identity 都使用 fresh execution seeds 5161--5163、`attempt=1`。5164--5167 仅用于编译
support scenes，未成为 evidence units。

| seq | profile | scene / seed | expected → observed TTC | finite TTC | non-NONE overlay | result |
| ---: | --- | --- | --- | ---: | ---: | --- |
| 1 | legacy control | single conflict / 5161 | OBSERVED_CONFLICT → OBSERVED_CONFLICT | 31 | 25 | evidence_complete |
| 2 | circle contact | single conflict / 5161 | OBSERVED_CONFLICT → OBSERVED_CONFLICT | 30 | 19 | evidence_complete |
| 3 | circle contact | multi conflict / 5162 | OBSERVED_CONFLICT → OBSERVED_CONFLICT | 32 | 19 | evidence_complete |
| 4 | legacy control | multi conflict / 5162 | OBSERVED_CONFLICT → OBSERVED_CONFLICT | 28 | 30 | evidence_complete |
| 5 | legacy control | semantic clear / 5163 | NO_CONFLICT_IN_HORIZON → NO_CONFLICT_IN_HORIZON | 0 | 18 | evidence_complete |
| 6 | circle contact | semantic clear / 5163 | NO_CONFLICT_IN_HORIZON → NO_CONFLICT_IN_HORIZON | 0 | 0 | evidence_complete |

这组证据确认冻结 semantic factor 的目标语义：两个 conflict 场景在两种 profile 下都有 finite
TTC 并匹配 `OBSERVED_CONFLICT`；clear 场景两者均无 finite TTC，而 circle-contact 的 overlay
为 NONE，legacy control 保留 18 个 non-NONE overlay 作为 identifiability control。该 schedule
没有预注册导航性能比较，因此不能从这些计数推出 circle-contact 更安全、更快或总体更优。

## 4. Readiness、bootstrap 与 teardown

每个 unit 的四条 direct readiness stream 都超过冻结 minimum 20 messages：

| seq | activation tracker/context | evaluation tracker/context | teardown restore |
| ---: | ---: | ---: | --- |
| 1 | 89 / 30 | 418 / 140 | true |
| 2 | 88 / 30 | 318 / 107 | true |
| 3 | 88 / 30 | 1181 / 400 | true |
| 4 | 88 / 30 | 319 / 107 | true |
| 5 | 89 / 30 | 292 / 98 | true |
| 6 | 89 / 30 | 280 / 94 | true |

6/6 positive-progress `/clock` bootstrap barrier 均进入 `service_wait_released`；所有 unit 都完成
scene snapshot post-episode verification 与 two-phase teardown/restore。相邻 unit 之间执行主机
隔离复核，最终 ROS/Gazebo/move_base 进程和保留端口均为空。

## 5. Canonical terminal evidence

```text
stage report:
  path:   artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/
          v2_04g_r6_i5_stage_report.yaml
  SHA256: 501ab197eacd9926d9760402e2c8586e37edc4d288c5c4cb0610024a8a1c5fef
  status: execution_complete_pending_assessment

execution assessment:
  path:   artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/
          v2_04g_r6_i5_execution_report.yaml
  SHA256: 8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599
  status: simulation_integration_validation_pass
```

Stage report 的 `pending_assessment` 是冻结的 assessor 输入边界，不表示 assessment 未完成；
最终状态由单独、O_EXCL 创建的 execution report 表达。终态只读复核确认：

- 6 份 canonical journal 全部 `evidence_complete`，另有 6 份对应 lock receipt；
- execution report 引用的每份 journal、activation、evaluation、trace、clearance、process log、
  teardown receipt 均重新计算 SHA256 并匹配；
- 独立审计递归重哈 stage/assessment/6 journals 中 196 个 path+SHA 引用（65 个 unique
  file/digest），36/36 raw resources 与 6/6 journals 均为 0 mismatch；finite TTC、overlay
  bucket 和 readiness counts 也从 raw trace/evaluation 独立重算并逐项一致；
- 从 canonical stage report 重新运行只读 `build_assessment`，得到的内存 document 与已持久化
  execution report exact-equal；
- `ttc_status_matches_preregistration`、`semantic_schedule_pass`、
  `readiness_direct_counts_pass`、`two_phase_teardown_restore_pass` 和
  `integration_validation_pass` 全部为 true；
- `integrity_failures=[]`，没有 retry/resume、seed replacement、budget expansion、held-out
  5001--5010、winner、training 或 real-vehicle 行为。

## 6. 保留的历史边界

- R5 terminal failure 与剩余 budget 继续冻结，不得 retry/resume。
- R6-I1 仍为 1 consumed / 5 forfeited，不得复用 identity、seed 或 budget。
- failed I3 release 继续保留在 SHA
  `5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6`，未修改且不可执行。
- I4 仍是 offline repair/readiness closure，自身没有 execution budget。
- I5 已终态完成，6/6 budget 全部消费；不得再次调用 I5 `--execute`，不得把它作为可 resume
  stage。

## 7. 下一阶段入口

当前应停止执行。最合理的下一项是先做独立 offline result interpretation/design closure：冻结
I5 的语义结论和 claim limit，明确论文要回答的是“semantic correctness”还是“navigation
performance”。若要证明性能增益，需要新的预注册、fresh seeds、足够重复次数和配对统计，不能
把本次 3-seed/6-unit integration schedule 外推为性能结论。

任何新的 simulation、training、held-out、winner selection、V2-05 或实车阶段都需要新的明确
授权与独立 contract/release。实车前还必须完成 footprint、轴距/转向、制动、时延和安全阈值
标定；当前所有 deployment thresholds 继续 `runtime_ready=false`。
