# CURRENT V2-04G-R6-I3 EXECUTION RELEASE PREFLIGHT HANDOFF

更新时间：2026-07-21

## 1. 权威结论

用户已明确授权 `V2-04G-R6-I3 bounded simulation execution`。执行前重新验证 frozen
hash、dependency closure、readiness machine review 和主机进程隔离均通过；随后创建了唯一
canonical release：

```text
path:   experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml
SHA256: 5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6
```

release 自身 closed schema、冻结 6-identity schedule、logical closure digest 和 22 个
path+SHA resource binding 全部通过静态复核。但是 canonical runner 的真实全量
`_execution_preflight` 在重新解析 authorization-bound 历史 YAML 时 fail closed：

```text
experiments/manifests/v2/integration/
v2_04g_r6_i1_scene_derivation.yaml.seed_roles contains a non-string key
```

因此没有调用 `--execute`，没有创建 attempt root、journal、receipt、stage report 或 raw
evidence，没有启动 ROS/Gazebo/move_base/TEB，没有消费 seed 或 evidence unit。当前状态是：

```text
canonical release present:             true
release closed schema / 22 bindings:   pass / pass
full execution preflight:              fail closed
attempts / journals / stage report:    absent / absent / absent
evidence budget authorized / consumed: 6 / 0
unattempted units forfeited:            0
execution_ready:                        false
formal_result / runtime_ready:          false / false
```

该失败发生在 handoff 定义的 prejournal gate，早于 runner 的首次 state creation 和
`base_roslaunch_spawn_requested` consumption boundary，因此不是 schedule identity 的 terminal
attempt；按冻结规则保持 0 unit consumed，不触发未尝试预算 forfeiture。

## 2. 唯一已定位 blocker

release validator 会对 authorization 的每一个 `.yaml` bound resource 执行 strict YAML parse，
并对整个 data tree 强制所有 mapping key 为字符串：

- `src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_r6_i2_r6_i3_release.py`
  - `_validate_data_tree`：只接受 string mapping keys；
  - `load_and_validate_execution_release`：对全部 authorization-bound YAML 无差别 parse。

冻结 authorization 同时绑定：

```text
experiments/manifests/v2/integration/v2_04g_r6_i1_scene_derivation.yaml
SHA256: 5f3e756609828b70ba0612c59a510d1d3750bd8fe33161829da3f9b7e706ce12
```

该历史文件的 `seed_roles` 使用 5141--5147 七个整数 YAML key。只读全 roster 审计确认，
authorization-bound YAML 中仅此文件存在非字符串 key。仅在内存 counterfactual 中跳过这七个
key 后，其余真实 gate 全部通过，包括 schedule、authorization 语义、14 scene children、全部
local/external closure rehash、machine review、runtime executable 和进程隔离。该 counterfactual
不构成执行许可，也绝不能用于绕过 canonical runner。

原 24 项 directed tests 通过，但 release-validator synthetic fixture 的 authorization 只绑定
preregistration 与 I2 closure，没有覆盖真实 legacy scene-derivation numeric-key resource，因此未
发现此不兼容。

## 3. 不允许的原地修补

不得通过以下方式继续本阶段执行：

- 不得编辑或重新序列化冻结 R6-I1 scene derivation；其 exact SHA 已由 authorization 绑定；
- 不得 monkeypatch loader、在内存放宽规则或绕过 `_execution_preflight`；
- 不得原地修改 validator；validator SHA 被 runner hardcode，并被 closure、machine review 和
  release 共同绑定；
- 不得覆盖、删除或重新生成当前 failed-preflight canonical release；
- 不得调用 `--execute`、切换旧 I1 runner、retry/resume、换 seed 或扩预算。

## 4. 主机隔离审计

主机无 ROS/Gazebo/move_base/training 进程，无 11311 listener，磁盘和内存充足。默认登录 shell
受 `.bashrc` 影响，混入 `/home/robot/arena_ws` 与 `/home/robot/catkin_ws` 的 ROS/Python/Gazebo
路径，不能用于未来执行。所有 future review/runner 必须从 `env -i` non-login shell 启动，且只
source：

```text
/opt/ros/noetic/setup.bash
/home/robot/robot_ws_base_rl/devel/setup.bash
```

## 5. 下一阶段唯一入口

当前必须停止。任何后续工作都需要新的明确用户指令，建立独立、offline-only 的
preflight-integrity repair/readiness closure。建议的单一修复边界是：

1. 全部 authorization-bound resources 继续 exact bytes+SHA 重哈；
2. 只对 validator 实际语义消费的 preregistration 与 inherited I2 closure 做 YAML parse；
3. 增加覆盖当前真实 authorization roster 与 legacy integer-key YAML 的回归测试；
4. 机械重建 runner trusted hash、独立 dependency closure 和 machine review；
5. 保留当前 failed release 和 0-consumption 事实，不创建 execution state；
6. repair/readiness 再次独立通过后，仍需另一条新的 simulation execution authorization。

该 repair 不得改变 semantic factor、七个阈值、场景行为、evaluator、exact schedule、训练边界或
实车边界。新阶段是否沿用未消费 seeds/budget 必须另行显式决定，不能在本 handoff 中推定。

