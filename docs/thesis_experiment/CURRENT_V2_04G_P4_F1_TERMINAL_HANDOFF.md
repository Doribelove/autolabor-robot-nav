# FAM-TEB V2-04G-P4-F1 终态交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`global_budget_exhausted_obvious_improvement_not_demonstrated`

## 1. 一句话结论

本轮有限性能搜索已经终止：历史 P1 正式结果的受限完成时间点估计降低
`3.7624%`、单侧 95% 下界 `2.7904%`，低于新性能计划冻结的 `10% / 5%`
明显提升门；后续三轮搜索又因执行完整性失败没有形成完整 calibration 或
held-out test 矩阵。全局 `2200` 个仿真证据容量已全部消耗或退役，因此结论必须是：

> **未证明明显提升。**

这不是“效果为零”，也不是“FAM-TEB 必然无效”；它表示在预注册门槛、有限预算和完整性规则下，
没有得到足够证据支持相对最强公平固定 TEB 基线的明显性能提升。

## 2. 保留的历史正式结果

P1-E1 是独立且完整的正式阴性结果：

- 90 paired blocks / 270 episodes，全部 attempt=1；
- integrity、安全/任务非劣、机制门通过；
- efficiency point=`0.03762364346211233`；
- one-sided 95% lower=`0.027903858836313078`；
- `performance_improvement_demonstrated=false`；
- four-gate assessment SHA：
  `89147e95f19c461926e302982ab4db3772f1847fa0ca273e05b0df73fe2cd6df`。

P1 不得重跑、追加 seed、删除失败或改阈值。P4-F1 没有覆盖或改写 P1，只是在更严格的
`10% / 5%` 明显提升门下保留它作为唯一完整性能证据。

## 3. post-P1 三轮有限搜索总账

| round | capacity | consumed | retired | 终点 |
|---|---:|---:|---:|---|
| 1 | 792 | 2 | 790 | P2-C1-R1 sequence 2 运行身份映射失败 |
| 2 | 704 | 1 | 703 | P3-I1 Anchor Bank 离线 lifecycle/provenance schema 被严格 parser 拒绝 |
| 3 | 704 | 1 | 703 | P4-I1 执行环境无法解析 `m2_gazebo` |
| **总计** | **2200** | **4** | **2196** | **剩余 0** |

训练上限原为 120000 steps，实际消耗 0；终态全部退役，剩余授权 0。预算、alpha、seed
都不得回收、跨轮转移或扩张。

## 4. 单因素 amendment 链

### 4.1 P2-C1-R1 → P3

P2-C1-R1 的 primary cause 是把 evidence identity
`qualification_overlay_v125` 误传给只接受冻结语义 profile 的 supervisor。
P3 只改一个因素：运行消费者使用 `r6_semantics_circle_contact`，候选方法身份继续只写入
schedule、journal、raw trace 和 assessor。

P3-I1 证明该修复达到 supervisor 与 bootstrap 边界：

- supervisor 正常启动；
- Gazebo 成功 unpause；
- 两个严格递增正时钟样本；
- move_base/TEB service wait 被释放。

随后在事务前的 initial readback 中，冻结 Q1 Bank 被严格 parser 拒绝。

### 4.2 P3-I1 → P4

P4 只改一个新因素：

- `status: offline_materialized_not_runtime_ready`
  → `uncalibrated_simulation_candidate`；
- 删除 `source_provenance.p2_q1`。

五个 Bank 都通过严格离线 parser；TEB 参数、边界、anchor、overlay、transaction、安全门
逐树相等，行为字段变更数为 0。P4-Q1 independent review SHA：
`26c03a7e31a06d1f45ccc7348cdd980974b2a00c2b9b7f8e8879af38f0d4b3fd`。

这只能证明候选配置格式可解析，不能证明其 live runtime、安全或性能。

## 5. 最后一次 P4-I1 执行

唯一 release：

- release SHA：
  `1bcd0392d475cdf17d7a2bf6540e024a61136150011945d323cae1234b568220`；
- seed=`5924`；
- attempt=`1`；
- probe=`high_speed_to_conflict_overlay_transition`；
- candidate=`circle_contact_v125`；
- authorized/consumed/completed=`1/1/0`；
- retry/resume/replacement/seed substitution=`false`。

直接原始错误：

```text
RLException: [m2_v2_04g_p2_q1_execution.launch] is neither a launch file
in package [m2_gazebo] nor is [m2_gazebo] a launch file name
```

因此：

- consumption boundary 已越过；
- Gazebo 未启动；
- move_base 未启动；
- 参数事务未启动；
- raw trace 未产生；
- held-out test 未访问；
- 没有残留 ROS/Gazebo/move_base 进程。

离线归因表明 launch 文件本身存在且已 hash-close；执行 child environment 继承
`ROS_PACKAGE_PATH`，但 release 没有绑定其值，preexecution 也没有执行 package-resolution
probe。直接失败是 package resolution；主要完整性缺口推断为
`workspace_overlay_environment_not_bound_or_resolved_before_consumption`。

不得以“只要 source 一下 workspace 就能跑”为理由重试：身份已在预注册 consumption boundary
消耗，最后一轮余量已按 failure action 退役。

## 6. 终态证据

| artifact | SHA-256 |
|---|---|
| terminal evidence freeze | `1cc03f4094bb8db7c63950762350dc8c02a2dcf977d1206f599ba8a0271073d3` |
| global budget accounting | `3d1189c7f6f779e4ddd0dae8eaa5f9c373734c49038745d692ec2e8bc7f567cc` |
| terminal conclusion | `8ce6736dd7ea1f13d6c272f0420f4588d1e0c9e25949b7b1eabf5bd028b3eeb7` |
| independent review | `e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8` |
| terminal closure | `9dfb99adcedc7fa5f5c15367c491f6e0728064ea2536a87e1fe81600c1f67ef4` |

终态目录：

`artifacts/v2/integration/v2_04g_r6_i1/p4_f1_terminal_program_closure/`

## 7. 论文合法表述

建议正文使用：

> 历史 P1 正式配对试验的受限完成时间点估计降低 3.7624%，单侧 95% 下界为
> 2.7904%，均未达到本轮预注册的 10%/5% 明显提升门；后续有限预算搜索因执行完整性失败
> 未形成完整校准或 held-out test 矩阵。全局 2200-unit 预算现已耗尽并关闭，因此本研究未证明
> FAM-TEB 相对最强公平固定 TEB 基线有明显性能提升。

禁止写成：

- FAM-TEB 已明显优于 TEB；
- 高速 Anchor Bank 已通过 live safety/performance 验证；
- RL 或参数大调带来学习增益；
- 已选出 winner；
- 已具备部署或实车安全资格。

## 8. 后续边界

性能搜索终止，`next_safe_entry=none_performance_search_terminal`。不得：

- 重跑 P2/P3/P4 的任何 seed 或 release；
- 修复环境后复用 seed5924；
- 新增 round、回收 2196 个 retired units 或 120000 training steps；
- 访问新 held-out test；
- 降低 10%/5% 或任一安全/完整性门；
- 修改 `/home/robot/robot_ws`、实车、CAN 或在线实车参数。

允许的后续工作仅限不改变结论的离线论文整理、证据校验和代码归档。

## 9. 最终自检

- P3-I1-D1、P4-D1、P4-Q1、P4-F1 的 builder/reviewer `--check`：全部通过；
- P4-I1 release 离线重放：650 个本地资源、301 个外部资源全部通过；
- 新增/相关 YAML 47 份：全部可解析；
- 新增 Python 文件：`py_compile` 通过；
- `git diff --check`：通过；
- 相关 P2-D2/P2-Q1/C1-R1 测试（排除仅适用于 execution-root-absent 的历史
  fresh-state case）：`12 passed, 1 skipped, 1 deselected`；
- 完整原测试直接运行时为 `12 passed, 1 skipped, 1 failed`，唯一失败是
  C1-R1 `require_fresh_state=True` 正确检测到必须保留的历史 execution root；不得删除证据来
  使该测试变绿；
- 最终 ROS/Gazebo/move_base 进程：0。
