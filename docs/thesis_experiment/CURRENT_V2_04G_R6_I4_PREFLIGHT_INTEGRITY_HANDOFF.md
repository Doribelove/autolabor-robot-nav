# CURRENT V2-04G-R6-I4 PREFLIGHT-INTEGRITY REPAIR HANDOFF

更新时间：2026-07-21

## 1. 权威结论

用户明确授权的独立 `V2-04G-R6-I4 offline preflight-integrity
repair/readiness closure` 已完成。该阶段只修复 R6-I3 full preflight 对 historical
authorization resources 的过度 YAML 解析，不执行仿真，不创建 execution state，不启动
ROS/Gazebo，不消费 seed 或 budget。

最终机器结论为：

```text
stage:                    V2-04G-R6-I4
review_result:            pass
status:                   preflight_integrity_repair_readiness_closure_pass_future_release_absent
execution_authorized:     false
execution_ready:          false
formal_result:            false
runtime_ready:            false
future I4 release:        absent
I4 seed / schedule / unit: [] / [] / 0
source I3 units:          6 authorized / 0 consumed / 0 forfeited
ROS / Gazebo / training:  not started / not started / not started
```

`pass` 只表示 offline repair/readiness closure 完整，不是 simulation execution 许可。必须停止并
等待新的明确 simulation execution 授权；下一条授权还必须明确 future identity/seed/schedule/
budget 是否以及如何分配，I4 不推定可复用 I3 的 5151--5157 或 6 units。

## 2. Failed I3 release 保留状态

R6-I3 failed-preflight release 未删除、未覆盖、未重新序列化：

```text
path:   experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml
SHA256: 5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6
```

旧 I3 runner、validator、validator test、closure 和 readiness review bytes 均保持原样。
I4 将它们作为 terminal historical snapshots 复核，不调用旧的 `release absent` builder/reviewer
去推断当前状态。failed release 在 I4 下明确 `executable_or_reusable=false`。

以下状态全部 absent：

- future `experiments/manifests/v2/integration/v2_04g_r6_i4_execution_release.yaml`；
- I3/I4 attempts、journals、receipts、raw/semantic evidence；
- I3/I4 stage report 与 execution report；
- I4 `ros_home`、`ros_logs`。

因此 I3 继续保持 6 authorized / 0 consumed / 0 forfeited；I4 自身是 0/0/0。

## 3. 单一 parser-scope 修复

历史 validator 对 authorization-bound 每个 `.yaml` 做 strict parse，导致不被语义消费的
R6-I1 scene derivation 中 5141--5147 integer keys 触发 fail closed。I4 没有放宽 strict loader
或 string-key policy，而是建立 versioned validator：

```text
src/tools/thesis_experiment/src/thesis_experiment/
v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py
SHA256: 9b9dd3fc580d0f880705bf87e120cdb9d30fdce812f585ee99e3f0fdf1fa3994
```

真实 authorization roster 被 closed 为精确 12 label/path：

1. 12/12 resources 均通过 component-wise no-follow、single-open、regular-file 和 exact
   bytes+SHA256；
2. 仅按 closed label 解析 `preregistration` 与
   `inherited_r6_i2_dependency_closure`；
3. 其余 10 项强制 `document=None`，不再按文件后缀决定 parse；
4. parsed 两项仍拒绝 duplicate、merge、non-string key、非有限浮点和类型漂移；
5. legacy scene derivation 的 integer-key 正例通过，但 byte drift 和 symlink 负例仍 fail
   closed。

版本化 validator test SHA 为：

```text
663fd2da6a8781e3cc4041aad46141e3ffdfee38883d4bcf2fe0e1eb59cc3a89
```

定向测试还覆盖真实 roster 缺 label、多 label、path swap，以及所有 forbidden execution-state
和 process-audit fail-closed 分支。

## 4. Offline runner 与 trust hash

I4 runner 是 offline-only preflight harness：

```text
path:   src/tools/thesis_experiment/scripts/
        v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_bounded_validation.py
SHA256: ae4fa36c255d56b6e739235e55f472dad4afc966bb6b2ffd39199c1e94a72f4e
```

它在加载前 hardcode 并验证 repaired validator 与真实-roster test SHA，只从同一已验证 bytes
snapshot 加载 validator。它只有 `--check-only`，没有 `--execute`、`execute()`、ROS import 或
subprocess import。

runner 实际重放当前 failed release 后通过：

```text
release bindings rehashed:       22 / 22
authorization resources rehashed: 12 / 12
authorization parsed/hash-only:   2 / 10
historical schedule identities:   6
historical schedule SHA256:       ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c
forbidden host processes:         0
execution side effects:           0
```

process isolation 只排除当前 reviewer PID；若它由 `roslaunch` 等 forbidden ancestor 启动也会
被拒绝。`/proc` 枚举失败或非瞬时 PID cmdline 读取失败同样 fail closed。

## 5. 独立 dependency closure

Canonical I4 closure：

```text
path:          artifacts/v2/integration/v2_04g_r6_i1/
               r6_i4_preflight_repair_review/execution_dependency_closure.yaml
file SHA256:   e9d27ed1522ca744f1bbf4a91832287ac2e780aa395c1fa1d147e3c587099b0f
logical SHA256:dceb73df8619849f5b5a0442b739be09815bfc86939a188873c94993fe4d5b74
```

新 closure 的直接图为 54 local files / 65 edges / 307 external files / 47 Python bindings /
9 runtime bindings，`unresolved=[]`。它还独立机械重哈两个 inherited terminal closures：

| snapshot | file SHA | logical SHA | local | external | runtime |
| --- | --- | --- | ---: | ---: | ---: |
| R6-I3 | `55f0e343...94b37` | `f83beb04...ed4f` | 136 | 307 | 9 |
| R6-I2 | `63c4e7ba...fac58` | `2be410c3...1fe6` | 106 | 301 | 5 |

I4 不重新运行 ROS binding resolver；它先逐项重哈 frozen I3 external table，再要求新 I4
发现的 Python/runtime bindings 是该 47/9 表的子集。强制 monkeypatch
`subprocess.run` 抛错时，完整 closure/review 仍通过。

Hash graph 刻意排除 closure artifact 自身、final machine review 和 future I4 release，保留
DAG；failed I3 release 是已绑定 historical input。任何未来 release 必须另行绑定本 closure
和 machine review。

## 6. Machine review 与验证结果

Canonical machine review：

```text
path:   artifacts/v2/integration/v2_04g_r6_i1/r6_i4_preflight_repair_review/
        v2_04g_r6_i4_preflight_integrity_readiness_review.yaml
SHA256: 3f183768e657ec17f4fd1045ffd0749d13213cc53017a09c6e166d840d647b12
```

Reviewer 两次确定性重建 closure、两次调用 offline runner，并在进程内执行真实 legacy roster
正例及 byte-drift、symlink、parsed-I2-numeric-key、missing/extra/path-swap 负例。最终：

```text
review_result:                   pass
dependency unresolved:          0
real authorization roster:      12/12
parsed / hash-only:              2 / 10
state before == state after:     true
forbidden processes before/after: 0 / 0
ROS/Gazebo/move_base/training:   false / false / false / false
seed or budget consumed:         false
```

最终两份 test 文件共 `49 passed`。reviewer `--check-only` 已再次从当前 bytes 精确重建 persisted
closure 与 review 并通过。

## 7. 冻结 authority hashes

```text
I4 contract:       a5ac55ecd84a59a847e92e4268c95983312e39720fbd1839871bb25f90e158a4
I4 preregistration:4abd2bdaf50ef5b0100494d5ef23d6628e0db03a7513f8a7aa3803f1224e701a
I4 transition:     99161770dbc0da0699868ce9a37b53b22d42214edff10bcb2ea191c3f2f49c2d
I4 validator:      9b9dd3fc580d0f880705bf87e120cdb9d30fdce812f585ee99e3f0fdf1fa3994
validator tests:   663fd2da6a8781e3cc4041aad46141e3ffdfee38883d4bcf2fe0e1eb59cc3a89
offline runner:    ae4fa36c255d56b6e739235e55f472dad4afc966bb6b2ffd39199c1e94a72f4e
preflight tests:   7fbcca44813689eacba048da8ee0e49b6dc9b212a2e69bae3161bad79c30c4ca
dependency module:b85348083b946bfac65c0d1f17f0f522edcf39cb4d09206d4f4f3719d6f5f5c9
generator:         15a00f0d749f716e885081532e104fe91fe93a8be52387f15ab0f0cb18fdb122
reviewer:          7658556587f03fc46b12b9ad94c0f20c61c85dc4baec948e5cb68371d12e8d8f
```

## 8. 下一阶段唯一入口

当前必须停止，不能：

- 用 failed I3 release 或旧 I3 authorization 调用执行；
- 创建 future I4 release、attempt、journal、receipt 或 evidence；
- retry/resume R5 或 R6-I1；
- 推定可复用 5151--5157、换 seed 或扩预算；
- 启动 held-out 5001--5010、winner selection、V2-05、SAC/training；
- 启动实车或在线 TEB 写入。

只有收到新的明确 bounded simulation execution 授权后，才能先重新复核本 handoff、frozen
hashes、closure、machine review、主机进程隔离和新的 seed/schedule/budget 决策，然后在不同
canonical path 创建唯一 exact-hash future release。即使未来获准，也必须在任何 attempt root、
journal 或 subprocess 之前完成完整 prejournal gate。

推荐复核命令：

```bash
env -i HOME=/home/robot USER=robot LOGNAME=robot SHELL=/bin/bash \
  LANG=C.UTF-8 PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 /bin/bash --noprofile --norc -c '
    source /opt/ros/noetic/setup.bash
    source /home/robot/robot_ws_base_rl/devel/setup.bash
    /usr/bin/python3 /home/robot/robot_ws_base_rl/src/tools/thesis_experiment/scripts/\
v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_preflight_integrity_reviewer.py \
      --workspace /home/robot/robot_ws_base_rl \
      --output /home/robot/robot_ws_base_rl/artifacts/v2/integration/\
v2_04g_r6_i1/r6_i4_preflight_repair_review/\
v2_04g_r6_i4_preflight_integrity_readiness_review.yaml \
      --check-only'
```
