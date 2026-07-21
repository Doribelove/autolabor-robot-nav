# CURRENT V2-04G-R6-I3 BOUNDED SIMULATION AUTHORIZATION HANDOFF

更新时间：2026-07-21

## 1. 权威结论

独立逻辑阶段 `V2-04G-R6-I3` 已建立 fresh-seed bounded simulation
preregistration、closed-schema authorization envelope、纯离线 deterministic reviewer 和
定向测试。本轮只完成授权材料的建立与复核，没有启动 ROS、Gazebo、move_base 或 TEB，
没有物化 fresh scene，没有建立实际 execution entrypoint、journal、receipt 或 raw evidence，
也没有消费任何 evidence unit。

当前必须按以下两层状态理解：

```text
authorization envelope valid:              true
authorization manifest execution_authorized:true
separate execution release required:        true
separate execution release received:        false
execution_ready:                            false
execution may start now:                    false
evidence budget authorized / consumed:      6 / 0
ROS/Gazebo/move_base/TEB started:            false
formal_result / runtime_ready:               false / false
```

authorization manifest 中的 `execution_authorized: true` 是 R6-I2 冻结 closed schema 所要求的
bounded authorization envelope 字段；它不等于用户已下达本阶段的启动指令。preregistration
和 machine review 共同保留独立 execution-release gate。收到新的明确用户执行授权之前，
不得物化场景、建立 journal、spawn subprocess 或启动 ROS/Gazebo。

权威文件：

- `experiments/manifests/v2/integration/v2_04g_r6_i3_execution_preregistration.yaml`
- `experiments/manifests/v2/integration/v2_04g_r6_i3_bounded_simulation_authorization.yaml`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_authorization_review/v2_04g_r6_i3_authorization_review.yaml`
- `src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py`
- `src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i3_authorization_review.py`

最终 SHA256：

```text
R6-I3 preregistration:
  a8295c723c1cf973c2c35c86e5b2d5c07361bdf0e92f36a0e8d12d2364ce6268
R6-I3 authorization envelope:
  ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2
R6-I3 machine authorization review:
  20a058f15a79aebc448497374071c7028363faa185d1ebe820f1102c6b330913
R6-I3 reviewer:
  906df1914635fc7d996bb1d1073efba21e09e62d3edbf1759216bd7b31563dfb
R6-I3 directed test:
  0663dec5c746c627df7b4e919c5dac22254245a3e16f4a03bc25349b9054a955
exact schedule canonical digest:
  ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c
```

## 2. 独立身份、fresh seed 与新预算

R6-I3 不是 R5 或 R6-I1 的 retry、resume、amendment execution，也不继承任何历史
identity、seed 或 forfeited budget。机器化历史扫描覆盖 20 个 selected authoritative
pre-I3 YAML seed evidence，发现 217 个 1--9999 experiment-seed namespace reference，
历史 high-watermark 为 5147；新 seed block 5151--5157
此前引用数为 0。

冻结分配：

```text
execution seeds:                 5151, 5152, 5153
compile-support-only seeds:      5154, 5155, 5156, 5157
evidence units authorized:       6
evidence units consumed:         0
attempt limit per identity:      1
retry / resume:                  forbidden
replacement seed:                forbidden
budget expansion:                forbidden
first terminal failure:          stop stage and forfeit all unattempted units
```

5154--5157 只预留给 future scene compilation support，不是 evidence unit。held-out
5001--5010、R5 allocated 5111--5135 和 R6-I1 allocated 5141--5147 均继续禁止访问或复用。

## 3. 冻结 exact schedule

| Seq | Profile | Scene identity | Seed | Attempt | Expected TTC |
| ---: | --- | --- | ---: | ---: | --- |
| 1 | `r6_semantics_legacy_control` | `v2-04g-r6-i3-dynamic-conflict-single-s5151` | 5151 | 1 | `OBSERVED_CONFLICT` |
| 2 | `r6_semantics_circle_contact` | `v2-04g-r6-i3-dynamic-conflict-single-s5151` | 5151 | 1 | `OBSERVED_CONFLICT` |
| 3 | `r6_semantics_circle_contact` | `v2-04g-r6-i3-dynamic-conflict-multi-s5152` | 5152 | 1 | `OBSERVED_CONFLICT` |
| 4 | `r6_semantics_legacy_control` | `v2-04g-r6-i3-dynamic-conflict-multi-s5152` | 5152 | 1 | `OBSERVED_CONFLICT` |
| 5 | `r6_semantics_legacy_control` | `v2-04g-r6-i3-dynamic-semantic-clear-s5153` | 5153 | 1 | `NO_CONFLICT_IN_HORIZON` |
| 6 | `r6_semantics_circle_contact` | `v2-04g-r6-i3-dynamic-semantic-clear-s5153` | 5153 | 1 | `NO_CONFLICT_IN_HORIZON` |

顺序、profile、scene identity、seed、attempt 和 expected semantics 均为 exact schedule 的
一部分。禁止重排、跳过后补、替换 seed、retry、resume 或在失败后消费剩余 unit。

## 4. 冻结 single factor 与共同值

唯一 factor 保持：

```text
runtime field: supervisor.dynamic.conflict_estimator_id
legacy level:  legacy_class_conditioned_geometry_v1
aligned level: shared_circle_envelope_first_contact_v1
```

共同冻结值保持：

```text
runtime/evaluator TTC horizon:       5.0 s
minimum track confidence:            0.45
robot circle radius:                 0.62 m
minimum relative speed:              0.05 m/s
legacy closest-approach threshold:   1.35 m
overlay release confirmation:        0.20 s
world-model classification horizon:  2.0 s
```

D1 中离线可区分的 1.5/1.0 s 没有启用。tracker/classifier、Anchor Bank、mechanism、
transaction/join、scene behavior、evaluator 和 TEB 参数均未改变。

## 5. R6-I2 trust boundary 与机器复核

R6-I3 authorization envelope 绑定并重新验证 R6-I2 最终证据：

```text
R6-I2 machine review SHA256:
  b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61
R6-I2 dependency closure file SHA256:
  63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58
R6-I2 dependency closure logical SHA256:
  2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6
R6-I2 authorization component review SHA256:
  55e7c3d7aebcb561edc9acd794347355d6f60df46868462fa1beb069c7eb4c59
```

I3 reviewer 使用 caller-supplied exact authorization SHA、single-open/no-follow 读取和
closed、type-sensitive schema 校验，逐项验证 exact schedule、budget 与 12 个 bound
resource。它还在同一 Python process 内机械重建 R6-I2 review 并重哈 closure targets：

```text
local files / edges:                 106 / 146
external files:                      301
external Python bindings:            45
inherited binding coverage:          39
runtime bindings:                    5
unresolved dependencies:             0
```

machine review 结果为：

```text
bounded_authorization_review_pass_execution_release_required
```

该 pass 只证明 preregistration、authorization envelope、fresh-seed firewall 和继承 closure
在本轮离线复核中一致；不构成 simulation execution evidence、semantic outcome、性能结论、
安全证明或 deployment readiness。

## 6. 尚未建立的 execution material

本轮刻意不创建以下材料：

```text
fresh compiled scene index:          absent
fresh compiled scene children:       absent
actual R6-I3 execution entrypoint:   absent
R6-I3 execution dependency closure:  absent
execution journal / receipt:         absent / absent
semantic/raw evidence:               absent
```

reviewer 还对 `artifacts/v2`、`experiments/manifests/v2`、
`config/thesis_experiments/v2` 和 `src` 执行 I3 ownership-prefix inventory；除本轮
preregistration、authorization、reviewer、test 和 machine review 外，受控 roots 内任何
带 I3 ownership 命名的 release、scene、runner、journal、receipt 或替代目录都会使当前
authorization review fail closed。该 inventory 用于证明本轮 absence，不是 future
execution 的完整安全边界；future entrypoint 必须依赖下述专用 release validator。

旧 R6-I1 runner 仍保留其已确认的 paused-clock bootstrap deadlock，不能用于 R6-I3。
R6-I2 repair harness 是 offline-only verifier，也不是 ROS executor。因此当前
`execution_ready=false` 是完整性要求，不是待忽略的文档状态。

## 7. 下一条明确执行指令后的前置门

只有用户再次明确授权 R6-I3 simulation execution 后，才可按 normal implementation step
完成以下动作；这些动作仍必须发生在任何 journal 或 subprocess 之前：

1. 由冻结 R6-I1 scene behavior 派生并编译新 I3 identities，只允许替换 stage、scene ID 和 seed；
2. 机器验证 behavioral scene diff 为空，并绑定 fresh compiled index 与每个 child 的 path+SHA；
3. 建立实际 R6-I3 entrypoint，集成 R6-I2 的正进展 `/clock` barrier、authorization guard、
   credential-safe child env/log 和 fail-closed lifecycle；
4. 建立独立 R6-I3 execution dependency closure，并进行新的 machine review；
5. 建立独立 execution release manifest
   `experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml`，精确绑定当前
   preregistration/auth SHA、fresh scene index 及全部 children、actual entrypoint、独立
   execution closure 和新的 integration machine review；future entrypoint 必须要求
   dedicated closed/type-sensitive、single-open/no-follow release schema/validator、负向测试和
   caller-supplied exact release SHA，且必须在建立任何 journal、execution directory 或
   subprocess 前通过；authorization envelope 单独存在不能启动；
6. 在任何 journal/subprocess 前重新验证 authorization/release exact SHA 和全部 closure target；
7. 只有全部前置门通过，才可从 exact schedule sequence 1 开始执行。

任一前置门失败都必须 fail closed，保持 0 unit consumed，不得借此切换到旧 I1 runner、
复用历史 identity 或修改 schedule。

future entrypoint 还必须在 import inherited I2 authorization/reviewer code 前，以 hardcoded
path+SHA 和 no-follow regular-file 检查关闭 trust bootstrap；不能把当前离线 reviewer 的
import-then-closure-rehash 顺序直接当作 execution security boundary。

## 8. 验证结果与复现命令

```text
R6-I3 directed tests:                19 passed
R6-I2 inherited directed tests:      55 passed
R6-I3 reviewer --check-only:         pass
R6-I2 reviewer --check-only:         pass
frozen R6 design reviewer:           pass
ROS/Gazebo execution started:        false
```

纯离线复核命令：

```bash
cd /home/robot/robot_ws_base_rl
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py \
  --workspace /home/robot/robot_ws_base_rl \
  --authorization-sha256 ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2 \
  --check-only
```

## 9. 持续禁止项

R5 retry/resume、R5 剩余 68 units、R6-I1 retry/resume、R6-I1 forfeited 5 units、
held-out 5001--5010、winner ranking/freezing、V2-05、SAC/任何训练、实车运行和实车 TEB
写入继续禁止。`formal_result=false`、`runtime_ready=false`，所有 deployment threshold
继续保持 `runtime_ready=false`。
