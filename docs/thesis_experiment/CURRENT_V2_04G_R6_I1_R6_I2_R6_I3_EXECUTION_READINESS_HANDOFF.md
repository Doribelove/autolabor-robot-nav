# CURRENT V2-04G-R6-I3 EXECUTION-READINESS CLOSURE HANDOFF

更新时间：2026-07-21

## 1. 权威结论

用户明确授权的 `V2-04G-R6-I3 execution-readiness closure` 已完成。该授权只允许离线
物化 fresh scenes、实现 actual runner、dedicated release validator、独立 execution
dependency closure 和 machine review；本轮没有创建 execution release、attempt root、
journal、receipt、stage execution report 或 raw evidence，也没有启动 ROS/Gazebo/move_base/
TEB，没有消费 seed 或 evidence unit。

当前状态为：

```text
stage:                                      V2-04G-R6-I3
readiness machine review:                   pass
review status:                              execution_readiness_closure_pass_release_absent
authorization envelope valid:              true
separate execution release required:        true
separate execution release present:         false
execution_ready:                            false
execution may start now:                    false
evidence budget authorized / consumed:      6 / 0
fresh execution seeds:                      5151, 5152, 5153
compile-support-only seeds:                 5154, 5155, 5156, 5157
formal_result / runtime_ready:               false / false
winner ranked/frozen:                       false
training / real vehicle:                    forbidden / forbidden
```

machine review 的 pass 只证明 execution-readiness 静态材料闭合；它不是 simulation
execution evidence、semantic outcome、性能或安全结论，也不是 deployment readiness。

## 2. 阶段转换与冻结授权证据

本轮新增独立 phase transition 和 readiness contract：

- `config/thesis_experiments/v2/v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_contract.yaml`
  - SHA256 `65b2477324fa3764ecf28763ee21f59b070db7bb633897f2bf9672fb20912c2a`
- `experiments/manifests/v2/integration/v2_04g_r6_i3_execution_readiness_transition.yaml`
  - SHA256 `9ca4d1661546adf826fd2e66818f73b1b4ef93cd1c94610c01d3560dab1b9890`

authorization-only 阶段的以下文件保持 byte-for-byte 不变：

```text
preregistration:             a8295c723c1cf973c2c35c86e5b2d5c07361bdf0e92f36a0e8d12d2364ce6268
authorization envelope:     ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2
historical machine review:  20a058f15a79aebc448497374071c7028363faa185d1ebe820f1102c6b330913
historical reviewer:        906df1914635fc7d996bb1d1073efba21e09e62d3edbf1759216bd7b31563dfb
historical directed test:   0663dec5c746c627df7b4e919c5dac22254245a3e16f4a03bc25349b9054a955
exact schedule digest:      ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c
```

旧 authorization reviewer 的 closed-world inventory 证明的是当时 fresh scene/runner 的
absence。授权物化后不再重建该旧 review；它作为 terminal historical phase snapshot 被新
transition 和 closure 固定绑定。不得修改旧 reviewer/report 来让其在新 phase 下重新通过。

## 3. Fresh scene 物化与行为等价

scene derivation：

- `experiments/manifests/v2/integration/v2_04g_r6_i3_scene_derivation.yaml`
  - SHA256 `7c0a493370c5e47770adcf388b1234ac37385ddfbf71284fe2e547797050c7e9`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/v2_04g_r6_i3_scenes.yaml`
  - SHA256 `7d56821d54e02b3e659e1f39fc74cd6af76e7a3c28944ffff6b425d2d819bae6`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/compiled_scenes/compiled_scene_index.yaml`
  - SHA256 `d71a60cef681b497694d244df95d99c79110768b26a5875e5328430a156f4d3e`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/v2_04g_r6_i3_scene_behavior_equivalence.yaml`
  - SHA256 `5d87dd4ddcf8399ca25ce85075a070be76404b96332d15e1b51323249837e908`

7 个 source scene 只允许修改 top-level `manifest_id` 和每个 scene 的 `scene_id`、`seed`。
layout、trajectory、obstacle、randomization specification、evaluator-only 字段和 timing 全部
保持。reviewer 在内存中重新编译 7 个 fresh scene，并对 index 的 exact roster、7 组
`.instance.yaml + .world`、14 个 child SHA 和 bytes 逐项复核。

3 个 execution scene 的 randomization 全为零；reviewer 另外对 I1→I3 compiled instance
和 world 做 identity/seed normalization，3/3 均行为等价。4 个 compile-support scene 含
seed-dependent randomization，只证明 spec 等价与 fresh deterministic recompile；它们不是
execution identity，不消费 evidence，也不得进入结果。

## 4. Actual runner 与 bootstrap 顺序

canonical runner：

- `src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py`
  - SHA256 `ef3a477c6d8b6924455c24b6fbe7821472e23cc61369a3fb91a37d6c72b233d5`

I3 stage adapters：

```text
activation listener: dfd574ad5865b4115aa4512e30547084ec6ff8dfffd003546b8e300ffe8c094e
mechanism episode:   bc815fc75ccb50aa46398b3605b4a999571b3e92c2b9749a1d84a9fc3f4f14a4
runtime control:     a815a5e8c065637a361d0723adcbc0c529d1fb8f81b0cf1a7475f8a5f2641de9
```

runner 的 module import surface 只含 Python standard library。future `--execute` 必须先接收
caller-supplied exact release SHA 和固定 authorization SHA；它从 single-open/no-follow bytes
验证 dedicated validator SHA，执行已验证 bytes，再让 validator 对 release、authorization、
preregistration、fresh index/14 child、closure、machine review 和全部 local/external target
重哈。所有 gate 通过前不会创建目录、journal、stage report 或 subprocess。

R6-I2 repaired bootstrap 的唯一顺序为：

```text
base spawn
-> wait only for /gazebo/unpause_physics discovery
-> unpause request
-> successful unpause acknowledgement
-> first strictly-positive post-ack /clock sample
-> second strictly-greater positive post-ack /clock sample
-> R6I2PositiveClockBarrier.release_service_wait
-> wait for /move_base/TebLocalPlannerROS/set_parameters
```

runner 只使用 closure-bound canonical executables，不做 PATH runtime selection；child environment
使用 R6-I2 allowlist、loopback ROS master、attempt-local ROS_HOME/ROS_LOG_DIR 和 credential-safe
logging。其他进程的完整 argv 不进入诊断，terminal error/evidence/traceback 只保留脱敏文本。

已知 bootstrap trust 边界：已哈希 validator bytes 在读取 YAML release 时需要导入系统 PyYAML；
PyYAML package 随后会被完整 closure 重哈，但它是 closure 解析前的 bootstrap parser trust root。
不得宣称所有 external imports 都在首次 import 前完成 hash validation。

## 5. Dedicated execution-release validator

- module：`src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_r6_i2_r6_i3_release.py`
  - SHA256 `c79551a518d36bd04ddec3dbc0cd99a8b5a881e41cd3edeec041bea8004ef43a`
- negative tests：`src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/test_v2_04g_r6_i3_release_validator.py`
  - SHA256 `e04aa910166236603d5ac38c46a7ab0ac135ec18bc51b6c98ffd462aab2440f2`

validator 强制 caller exact release/auth 双 SHA、closed/type-sensitive schema、duplicate/merge key
拒绝、component-wise no-follow、single-open snapshot reuse、full 7-field exact schedule、全部 safety
firewall、fresh child exact roster、local/external closure logical digest 和 machine-review absence gate。
它返回 closure-bound runtime executable map，但不创建文件或 subprocess。

canonical future release 路径仍为：

```text
experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml
```

该文件在本轮必须且确实不存在。readiness contract、authorization envelope 或 machine review
均不能替代它。

## 6. 独立 dependency closure 与 machine review

权威机器文件：

- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/execution_dependency_closure.yaml`
  - file SHA256 `55f0e343788409301258da96f355e78d9fb689bdcc270ef8f5b9fe54b06a4b37`
  - logical SHA256 `f83beb04dc6e7cd1e43c2611997f86dbd5bf07c36f33d51fc979128f2cb4ed4f`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/v2_04g_r6_i3_execution_readiness_review.yaml`
  - SHA256 `3a8e9e466a08f7d3ef65542b285cf5020e6ce53dbb63cbe9b2316afb717a680e`

closure 结果：

```text
local files / edges:                 136 / 187
compiled fresh children:             14
external files:                      307
external Python bindings:            47
runtime bindings:                     9
unresolved:                           0
inherited R6-I2 local targets:       106 / 106 rehashed
inherited R6-I2 external targets:    301 / 301 rehashed
```

9 个 runtime bindings 为：

```text
$(find gazebo_ros)/launch/empty_world.launch
command-executable:roslaunch
command-executable:rosservice
command-executable:rostopic
node:gazebo_ros:gzserver
node:gazebo_ros:spawn_model
node:move_base:move_base
node:robot_state_publisher:robot_state_publisher
package-executable:xacro:xacro
```

hash DAG 保持无环：closure 包含 runner/validator/tests/scenes 和全部执行依赖，但不包含自身、
future release 或最终 machine-review artifact；machine review 绑定 closure；future release 再绑定
closure 与 machine review。

## 7. 最终离线验证

```text
release-validator + readiness directed tests:  24 passed
machine reviewer generation:                   pass
machine reviewer --check-only:                 pass / byte-equivalent
runner --offline-review:                       pass
fresh in-memory deterministic recompile:       14 / 14 byte-equal
compiled execution behavior equivalence:       3 / 3 pass
release / attempts / journals:                 absent / absent / absent
stage report / raw evidence:                   absent / absent
evidence units consumed:                       0
ROS/Gazebo/move_base started by this work:      false
```

最终主机检查观察到 stable workspace `/home/robot/robot_ws` 的既有 operator GUI/rosmaster；
本阶段未连接、终止或修改它们，也没有 Gazebo/move_base/R6-I3 execute process。该外部 ROS
状态不属于 R6-I3 evidence；future execution preflight 会按设计拒绝任何 live ROS/Gazebo/
move_base/training process，执行前必须先由现场建立独占仿真边界，不能复用 stable workspace
master。

## 8. 下一阶段唯一入口

现在必须停止并等待新的明确 simulation execution 指令。下一条授权若要执行，应明确允许：

1. 在重新验证本 handoff 的 frozen SHA、closure logical digest、machine review 和 host process
   isolation 后，创建唯一 canonical exact-hash execution release；
2. 将该 release 的 caller-supplied exact SHA 与固定 authorization SHA 交给 canonical runner；
3. 按冻结 6-identity schedule 执行，任一 terminal failure 立即停止并 forfeits 所有未尝试 unit；
4. 只持久化 atomic journal、receipt、raw evidence 和 terminal assessment，不 retry、resume、换
   seed 或扩预算。

在收到该新指令前，不得创建 release/attempt/journal，不得运行 `--execute`，不得启动
ROS/Gazebo。held-out 5001--5010、R5/R6-I1 identities、winner、V2-05、SAC/training 和实车
继续禁止；`formal_result=false`、`runtime_ready=false`。
