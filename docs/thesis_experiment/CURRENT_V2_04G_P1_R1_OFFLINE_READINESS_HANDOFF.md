# V2-04G-P1-R1 Offline Readiness Handoff

更新时间：2026-07-21

## 结论与权限边界

`V2-04G-P1-R1` 已完成纯离线 scene-seed、顺序平衡 schedule、统计核心、path+SHA
闭包、fresh-seed firewall、资源估算和独立 review。机器状态为：

```text
p1_r1_offline_implementation_readiness_closure_pass_release_absent
```

这不是执行授权。当前严格保持：

```text
formal_result=false
runtime_ready=false
execution_ready=false
execution_authorized=false
planned_evidence_units=270
authorized_evidence_units=0
consumed_evidence_units=0
training_budget_steps=0
I5 rerun/resume=false
ROS/Gazebo started=false
```

## 90 个 fresh blocks

三个冻结 I5 动态模板各使用 30 个 fresh simulator stochastic seeds：

| evaluator-only role | seed block | expected status |
| --- | --- | --- |
| `single_true_conflict` | 5201--5230 | `OBSERVED_CONFLICT` |
| `multi_true_conflict` | 5231--5260 | `OBSERVED_CONFLICT` |
| `time_separated_semantic_clear` | 5261--5290 | `NO_CONFLICT_IN_HORIZON` |

5291--5294 仅用于满足五类 V2 manifest 编译完整性，不进入 schedule，也不是 evidence。
执行 scene ID 统一为 opaque `v2-04g-p1-s{seed}`，不向 policy process 暴露 role。

派生规则只允许 evidence template 修改 `scene_id`、`seed` 和 evaluator split (`test_id`)；
geometry、trajectory、timeout、manifest randomization、sensor/actuator profile 均保持 I5 模板不变。
future release 必须重新证明同一个 seed 共同驱动 Gazebo RNG 和 actuator `dynamics_seed`，且
seeded stochastic path 非退化。

因此未来阳性结果只允许覆盖“这三个冻结动态模板上的 fresh stochastic-seed 仿真分布”；
不得写成 90 个独立几何场景，不得宣称 geometry/timing 或整个 DYNAMIC family 泛化。

关键 SHA：

- scene derivation：`63ef206eb7bad279c3f2e3da19e9f38a1179445b9b57b50873420e8ec86c9d1d`
- 94-scene manifest：`18bc03726473b9994a758cf0294673ee9ea45ff062121e2674777368b6bd5ddf`
- compiled index：`d9a5ee405e06575164b9c0251aaed6d8acf3ae323637a66b4f76894480ce4d56`
- compiled children：188，全部逐项 path+SHA 绑定

## 270-episode schedule

schedule 由 30 个 wave 构成。每个 wave 含三个连续 paired blocks，每个 block 在同一
scene/seed 上运行：

```text
r6_semantics_legacy_control
r6_semantics_circle_contact
fixed_teb_reference
```

三种 role 使用全部六个排列循环五次；每个 role 内 method 也使用全部六个排列循环五次，
method permutation index 固定为 `(wave + 2 * canonical_role_index) mod 6`。静态审计证明：

- 90 blocks / 270 episodes；180 confirmatory + 90 contextual；
- 每 role 30 blocks、每 method 90 episodes；
- 每 role 内每 method 在位置 1/2/3 各 10 次；
- 每 role 内六种 method permutation 各 5 次；
- 每 block 三行连续，共享同一 compiled instance/world，`attempt=1`；
- retry、resume、replacement、追加 seed 全部禁止。

schedule SHA：`6c43cced674ae475dcf972fcdcca1872639e3b70fd0dc3156bc38c3b738e79fd`。

## 统计评估器

纯 Python 统计核心：
`src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_statistics.py`，SHA
`67a41aa9b8b226c8965b509031c481c0a9dc98131bcd8ec120cc896c35bd82a8`。

冻结定义：

1. Restricted completion time：只有 clean success 使用 navigation time；collision、timeout、
   planner failure 和其他算法失败均保留并赋 80 s。禁止 successful-only 分析。
2. 二元非劣使用 paired harmful-discordance rate 的单侧 Clopper--Pearson 上界，作为 paired
   risk difference 的保守界。`k=0,n=90` upper=0.032738；`k=1,n=90` upper=0.051626。
3. Clearance 使用 90 对按 role 分层 paired bootstrap lower bound；每 role point estimate 也须
   不低于 -0.05 m。
4. 60 个 true-conflict blocks 中，treatment expected-status match fraction >=0.95 且不低于
   control。未记录 selected-track identity，禁止声称多目标选择正确性。
5. Efficiency 在 30 clear pairs 上检验 `log(RCT_control/RCT_treatment)`；geometric time
   reduction >=5%，单侧 lower>0，adjusted p<=0.05。
6. Mechanism 需要从 goal dispatch 到 terminal 的 timestamped overlay intervals 无缝覆盖窗口；
   检验 paired absolute occupancy decrease，报告 `1-mean(T)/mean(C)`，point reduction>=30%。
   control mean=0 时门失败。
7. Resampling 使用 SHA-256 counter RNG；bootstrap=10,000/base seed 20260721；
   sign-flip=100,000/base seed 20260722；nearest-rank 单侧 quantile。
8. Fixed sequence：`p_adj_eff=p_eff`，`p_adj_mech=max(p_eff,p_mech)`。

未来四类总门是 `integrity`、`safety_and_task_noninferiority`、
`efficiency_superiority`、`mechanism_reduction`。只有四门全部通过，才允许
`performance_improvement_demonstrated=true` 和 `claim_authorized=true`。完整但阴性的研究仍为
`formal_study_complete=true`，不得 outcome-dependent 地抹去阴性结果。

当前核心接收 closed-schema normalized terminal rows。future raw replay adapter 尚未创建；
它必须从 journal-bound raw files 重算 success、collision、time、clearance、TTC 和 overlay dwell，
不能信任 summary。

## 失败、完整性和资源

- collision、timeout、planner failure、algorithmic emergency：有效 ITT outcome，保留；
- process/interface/hash/schema failure、missing/duplicate identity、overlay trace 不完整：
  integrity failure，持久化 ledger、停止、claim=false、禁止 retry/replacement；
- pre-consumption failure：不消费 unit，停止并重新 review；
- missing pair 绝不允许删除后只分析成功样本。

P1 closure 重哈 428 个 workspace-local resources，其中递归历史传递资源 218 个；unresolved=0。
独立 reviewer 使用 component-wise `openat`、`O_NOFOLLOW` 和 single-FD stable snapshot，并在
内存中逐字节重建 194 个 P1 bundle files。

- closure file SHA：`825f636169f7f381eada2f261f8ed7932f1e115fc1fd297c50225345f24d6c66`
- closure logical SHA：`cb3ec8ed465e63d85c2c9ad041f6bfa9f3b22c9f49727cf9c78e294bce5de5f2`
- independent review SHA：`805316f6a7cb676ba25e1a93b88aeba1cdc1ac65b62aa25c9d5fc8be928c0230`
- I5 freeze：13/13 resources rehashed；I5 bytes 未修改
- 回归：13 passed，包括阳性四门、阴性正式结果、missing pair、interface/process failure、
  timeout/collision retention、overlay gap/unknown state 和 exact CP boundary

准确 lifecycle 是：5201--5294 已由 I6 `reserved`，P1-R1 只做 offline materialization；不能称为
“从未引用”。当前 authorized=0、consumed=0，release/attempt/journal 均不存在。

资源估算不是预算授权：serial concurrency=1；I5 最大 attempt 1,027,181 bytes，270 units 线性
上界约 277 MB；canonical cap 512 MiB、transient cap 2 GiB，future release 前至少 4 GiB free。
典型 4--6 h，全 timeout 规划 7--10 h，future per-identity cap 240 s，stage cap 20 h。

## 下一安全入口

`fixed_teb_reference` 已冻结为独立 `execution_mode=fixed_teb`，禁止 silent alias，且不启动
rule supervisor、typed transaction 或 dynamic obstacle bridge。现有组件已定义三种 method 的
语义，但 P1 exact 270-row runner adapter 和 raw replay adapter 均尚未创建。

下一安全入口不是直接运行，而是一个单独明确授权的 P1 execution-implementation/release stage：

1. 实现并独立测试 exact schedule runner 和三方法统一 raw schema/replay；
2. 复核 seed 到 Gazebo/actuator RNG 的非退化 plumbing；
3. 重建完整 closure，创建唯一 release 和精确 270-unit budget；
4. 再由用户明确授权 simulation execution。

在此之前不得启动 ROS/Gazebo，不得创建 release/attempt/journal，不得重跑 I5，不得复用历史
seed/budget，也不得扩训练预算。

## 只读复核

```bash
cd /home/robot/robot_ws_base_rl
PYTHONDONTWRITEBYTECODE=1 python3 src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_builder.py --check-only
PYTHONDONTWRITEBYTECODE=1 python3 src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_dependency_generator.py --check-only
PYTHONDONTWRITEBYTECODE=1 python3 src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_reviewer.py --check-only
```

