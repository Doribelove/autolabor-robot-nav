# CURRENT V2-04G-R6-I2 BOOTSTRAP/INTEGRITY REPAIR HANDOFF

更新时间：2026-07-19

## 1. 权威结论

独立逻辑阶段 `V2-04G-R6-I2` 已建立 bootstrap/integrity repair contract、
preregistration、实现、纯 Python/静态测试和 integration review。本阶段只修复
R6-I1 已确认的启动顺序与五项 post-review 完整性缺口；没有改变 semantic factor、
7 项 threshold、scene、evaluator、Anchor、mechanism 或任何冻结的 R6-I1 证据。

当前边界为：

```text
stage:                              V2-04G-R6-I2
scope:                              design / implementation / tests / static integration review
review result:                      pass
execution_authorized:               false
execution_ready:                    false
runtime_ready:                      false
seed_values:                        []
execution_schedule:                 []
evidence_budget_authorized:         0
evidence_budget_consumed:           0
ROS/Gazebo/move_base/TEB started:   false
formal_result:                      false
winner ranked/frozen:               false
```

权威机器文件：

- `artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/v2_04g_r6_i2_integration_review.yaml`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/execution_dependency_closure.yaml`
- `artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/v2_04g_r6_i2_authorization_assessment_review.yaml`

最终摘要 SHA256：

```text
R6-I2 machine integration review:
  b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61
R6-I2 dependency closure file:
  63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58
R6-I2 dependency logical digest:
  2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6
```

integration review 的 pass 不构成 execution authorization，也不是 ROS/Gazebo
集成执行证据、semantic outcome、性能结论或安全证明。

## 2. R6-I1 终止状态与本阶段边界

R6-I1 的唯一尝试仍永久保持：

```text
sequence:                            1
profile:                             r6_semantics_legacy_control
scene:                               v2-04g-r6-i1-dynamic-conflict-single-s5141
seed / attempt:                      5141 / 1
units consumed / forfeited:          1 / 5
terminal status:                     terminal_failure
retry/resume:                        forbidden
semantic execution reached:          false
```

确认根因为 `paused_sim_time_bootstrap_order_deadlock`：base launch 以
`paused:=true` 启动，runner 却先等待 move_base/TEB dynamic-reconfigure service，
而 `/gazebo/unpause_physics` 排在该 wait 之后；ROS 时间保持 `0.0 s`，目标 service
最终超时。该失败发生在 transaction、startup profile capture、arm、直接 readiness
计数和 semantic episode 之前。

R6-I2 不是 R6-I1 retry、resume 或 amendment execution。它：

- 不复用 seed5141、R6-I1 sequence 2--6 或已 forfeited 的 5 units；
- 不复用任何 R6-I1 identity；
- 不创建 seed schedule 或 evidence budget；
- 不写 R6-I1 stage report、journal、raw evidence 或 authorization；
- 只建立 future execution 所需的对称、fail-closed 修复和静态审查。

## 3. 六项独立修复

### 3.1 Bootstrap 顺序与正时间屏障

未来 adapter 的唯一合法顺序被冻结为：

1. base spawn；
2. 发出 unpause request；
3. 收到 successful unpause acknowledgement；
4. 收到 ack 后第一条严格正值 `/clock`；
5. 收到 ack 后第二条严格更大的正值 `/clock`；
6. 只有此时才释放 move_base/TEB service wait。

zero-only clock、重复正值但不前进、时间回退、unpause failure、deadline 或 base
在屏障释放前退出都必须 fail closed。R6-I2 的 harness 使用注入 callback 和纯状态机
验证调用顺序；本轮不导入 ROS、不连接 ROS master，也不 spawn subprocess。

### 3.2 External/runtime dependency path+SHA closure

机械 closure 关闭：

- R6-I1 遗留的 39 个 external Python binding name；
- 全部 R6-I2 本地与传递依赖；
- 以下 5 个 future runtime binding：

```text
$(find gazebo_ros)/launch/empty_world.launch
node:gazebo_ros:spawn_model
node:move_base:move_base
node:robot_state_publisher:robot_state_publisher
package-executable:xacro:xacro
```

每个 external/provider/runtime 文件都必须记录 absolute canonical path、SHA256 和
size；builtin/frozen Python binding 必须显式绑定 interpreter provider；所有外部文件
必须可重新 hash，`unresolved` 必须为空。I2 spawn wrapper 不再使用失效的
`$(find xacro)/xacro` 假定，而要求 future caller 提供并校验 closure 绑定的 canonical
xacro executable。当前 review 只解析 launch XML，不运行 launch。

### 3.3 Closed、type-sensitive authorization enforcement

future authorization validator 对 top-level 和全部 nested mapping 使用 closed schema，
拒绝未知字段、缺失字段和类型混淆；它必须逐项强检：

- 独立 stage 与非复用声明；
- preregistration 的 exact profile×scene×seed schedule 及顺序；
- 每个 resource path+SHA；
- closure file digest 与独立 logical closure digest；
- budget、scope、simulation-only 和全部 safety/firewall flag；
- 空 ledger 或与 exact schedule 严格一致的 replay prefix。

这些测试使用临时、合成、离线 fixture；没有创建真实 R6-I2 authorization、seed 或
budget。未来 validator 必须在创建 journal 或 subprocess 之前运行。

### 3.4 Single-open/no-follow hash+parse

YAML/resource reader 对每个相对路径组件使用 directory file descriptor 与
no-follow 约束，从一个 regular-file descriptor 只读取一次 bytes；SHA256 和 parse
必须共同使用这一份 bytes snapshot。duplicate YAML key、symlink、非普通文件、路径
越界或读取前后 file identity 变化均 fail closed，从而消除“先 hash、再另读 parse”
的 TOCTOU 窗口。

### 3.5 Deterministic assessor

R6-I2 assessor 的 stage、stage report digest 和其他 source digest 都由显式参数传入，
不读取未定义的 free variable，不修改 I1，不创建 execution artifact。相同输入的重复
build 必须 byte-equivalent；测试同时覆盖原 I1 assessor 中已确认的 `NameError` 类问题。

### 3.6 Credential-safe child environment 与日志

future child process 只能接收 exact allowlist 环境，ROS master 必须是 loopback，
`ROS_HOME` 与 `ROS_LOG_DIR` 必须限制在 attempt root 内。credential-like key/value
不得复制到 child audit；argv 中疑似 credential 必须拒绝；日志必须在持久化前进行
value-safe redaction。secret injection 只存在于临时单元测试 fixture，不进入 artifact
或 handoff。

## 4. 冻结实验边界

唯一 future semantic factor 仍为：

```text
supervisor.dynamic.conflict_estimator_id
```

两个 level 保持：

```text
r6_semantics_legacy_control:
  legacy_class_conditioned_geometry_v1
r6_semantics_circle_contact:
  shared_circle_envelope_first_contact_v1
```

7 项冻结 threshold 保持：

```text
runtime/evaluator TTC horizon:       5.0 s
minimum track confidence:            0.45
robot circle radius:                 0.62 m
minimum relative speed:              0.05 m/s
legacy closest-approach threshold:   1.35 m
overlay release confirmation:        0.20 s
world-model classification horizon:  2.0 s
```

R6-I1 的 runtime profile 对仍只允许
`dynamic.conflict_estimator_id` 这一处 leaf 差异。compiled scene index、全部 compiled
child、scene schedule 语义和冻结 evaluator 均 byte-for-byte 保持；本阶段没有把 D1
离线讨论的 `1.5 s` 或 `1.0 s` 写入 threshold，也没有修改 tracker/classifier、
Anchor、TEB parameter、transaction、scene 或 evaluator。

## 5. 独立 stage 与兼容性存储前缀

R6-I2 的逻辑身份、合同、preregistration 和授权边界完全独立于 R6-I1。部分物理文件名
使用 `v2_04g_r6_i1_r6_i2_*`，machine artifacts 位于：

```text
artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/
```

测试位于旧 reviewer 已声明的：

```text
src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/
```

这是因为冻结的 R6 design reviewer 是 closed-world reviewer；它在 R6-I1 建立前已把
上述 R6-I1 downstream ownership prefixes 声明为未来隔离区。R6-I2 物理文件放入这些
预声明前缀，只是为了保持该冻结 reviewer 的兼容性，不表示 R6-I2 属于 R6-I1 execution，
也不允许继承其 authorization、seed、budget 或 identity。

旧 R6 design reviewer、R6-I1 contract、runner、assessor、closure、authorization、
stage report、terminal assessment、journal、scene、profile 和 artifacts 均未改动任何
字节；没有为了让旧测试通过而改写历史 SHA256。

主要入口：

- `config/thesis_experiments/v2/v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml`
- `experiments/manifests/v2/integration/v2_04g_r6_i2_stage_transition.yaml`
- `experiments/manifests/v2/integration/v2_04g_r6_i2_repair_preregistration.yaml`
- `src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_reviewer.py`
- `src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_repair_harness.py`
- `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_r6_i2_bootstrap.py`
- `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_r6_i2_authorization.py`
- `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_r6_i2_dependency.py`
- `src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_execution_integration.launch`
- `src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch`

## 6. 本阶段明确不存在的执行状态

当前必须同时满足：

```text
R6-I2 execution authorization manifest:  absent
R6-I2 seed values:                       []
R6-I2 execution schedule:                []
R6-I2 evidence budget:                   0
R6-I2 execution root/journal:            absent
R6-I2 stage report/execution receipt:    absent
R6-I2 semantic episode/raw evidence:     absent
ROS/Gazebo/move_base/TEB processes:      not started by this stage
```

文件名包含 `authorization_assessment_review` 的 component report 只是离线 validator/
assessor 修复的机器审查，不是 authorization，不能授权任何 subprocess、seed 或参数写入。

## 7. 最终验证

```text
R6-I2 directed tests:               55 passed
all 28 test_v2*.py files:           240 passed
closure local files / edges:        106 / 146
closure external files:             301
closure Python bindings:            45
inherited I1 Python coverage:        39 / 39
runtime bindings:                    5
compiled-scene children:             14
unresolved dependencies:             0
contract-bound resources:            34 / 34 hash match
R5 artifact tree:                    68 files / ecb1f330... exact
I1 frozen critical hashes:           exact match
launch validation:                   XML parse only, pass
git diff --check:                    pass
ROS/Gazebo/move_base/training:       0 matching processes
```

closure 和 integration review 均已从当前文件重新机械构建并与持久化内容完全相等。
测试没有启动 launch；没有创建 authorization、journal、stage report 或 execution receipt。

## 8. 持续禁令

- 不得 retry 或 resume R5 seed5111；
- 不得消费 R5 剩余 68 units；
- 不得 retry/resume R6-I1 seed5141 或执行 sequence 2--6；
- 不得恢复或复用 R6-I1 已 forfeited 的 5 units；
- 不得复用 R6-I1 identity、seed 或 budget；
- 不得访问 held-out `5001--5010`；
- 不得 rank/freeze winner；
- 不得启动 V2-05、SAC 或任何训练；
- 不得连接、驱动实车或写实车 TEB 参数；
- 不得把本 review 的 pass 解释为 execution、performance 或 safety evidence。

## 9. 后续唯一合法入口

当前没有可执行的 R6-I2 authorization。若用户决定进行 future simulation validation，
必须另发新指令，并在本 repair review 固化之后：

1. 建立独立 execution stage，不得称为 R5/R6-I1 retry 或 resume；
2. 分配全新、未使用的 seed 和全新 evidence budget；
3. 创建并单独审查 exact schedule 与 closed-schema authorization；
4. 将 authorization 绑定到本阶段最终 machine review、dependency closure 和 logical
   digest；
5. 在任何 journal、ROS 或 subprocess 前重新验证所有 path+SHA、scope、安全 flag 和
   fresh-seed firewall；
6. 仍需用户另行明确授权后，才可启动 simulation execution。

在该新指令和独立 authorization 出现之前，合法动作仅限只读审计、离线测试与文档维护。

## 10. 下次会话上手检查点

本轮工作已保存到当前 dirty worktree，尚未 stage、commit 或 push。当前主仓分支为
`base_rl`，跟踪 `origin/base_rl` 且本地领先 1 个 commit。工作树同时保留更早的
R5、D1、R6 design、R6-I1 变更和既有 dirty submodule；下次不得用 reset、checkout、
clean 或递归 submodule 操作覆盖这些内容。

新会话按以下顺序恢复上下文：

1. 读取仓库根目录 `AGENTS.md`；
2. 读取 `docs/thesis_experiment/DEVELOPMENT_STATUS.md`；
3. 读取 `CURRENT_V2_04G_R6_I1_EXECUTION_HANDOFF.md`；
4. 最后以本文件作为 R6-I2 权威状态；
5. 在任何后续动作前只读检查 Git、authorization/execution 路径和 live process。

快速只读复核：

```bash
cd /home/robot/robot_ws_base_rl
git status --short --branch
sha256sum \
  config/thesis_experiments/v2/v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml \
  artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/execution_dependency_closure.yaml \
  artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/v2_04g_r6_i2_integration_review.yaml
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/tools/thesis_experiment/scripts/v2_04g_r6_i1_r6_i2_reviewer.py \
  --workspace /home/robot/robot_ws_base_rl --check-only
```

预期三个 file SHA 依次为：

```text
0e9da40014b4cfea23a9b983cfdd44752a0faa21aa30820d56c4e27384b3b132
63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58
b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61
```

reviewer 预期输出：

```text
repair_integration_review_pass_execution_not_authorized
```

若下一轮要继续仿真，建议用户先给出以下独立指令；在该指令出现前不得自行执行：

> 基于 R6-I2 最终 machine review 和 dependency closure，建立独立 R6-I3
> bounded simulation authorization review；使用全新未消费 seed 和新预算，冻结
> exact schedule。先只创建 preregistration/authorization 并复核，不启动
> ROS/Gazebo，等待再次明确授权执行。
