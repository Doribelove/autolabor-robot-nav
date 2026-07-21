# CURRENT V2-04G-R2-R1 HANDOFF

日期：2026-07-14

## 结论

V2-04G-R2-R1 readiness fault-taxonomy 修复轮次已完成，使用全新 seeds
4981--4986，6/6 readiness 全部通过。该轮次只验证 readiness 接口，没有启动
TTC、导航、SAC 或实车。

权威机器可读结论：
`artifacts/v2/calibration/v2_04g_r2_r1/v2_04g_r2_r1_readiness_freeze_report.yaml`，
SHA256 为 `4e47f09bfa6173f276184373ed09ae67d7a5f47264fe57b48d611241e7a4261b`。

## 冻结与唯一修改

以下运行时依赖保持不变并由 SHA256 校验：

- R1 bounded join 源码及 32 条缓存、1.0 s 到达年龄、序号差 2、时间戳差 0.45 s。
- R2 candidate bank 的全部 Anchor、机制和 supervisor 数值。
- R2 幂等 typed TEB transaction runtime。
- R2 simulation launch、动力学、传感器时延和噪声。

唯一修改是 readiness 观测语义：

1. 测量前要求 10 条连续机制消息满足 join valid、transaction valid、activated 且无 fault。
2. 保存每一个非空 `fault_reason` 及事务、ContextState、join 和 mechanism 快照。
3. 将原因分成 clean、预期 fail-closed context hold、backend fault、unknown fault。
4. backend 和 unknown fault 仍要求严格为 0；预期 context hold 最多允许 1 次，且事务有效率、
   激活率和 join 有效率仍必须不低于 0.95。

## 6/6 结果

| Profile | seed | 稳定窗口 | 事务有效率 | 激活率 | join 有效率 | taxonomy | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| balanced | 4981 | 10 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| balanced | 4982 | 10 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| balanced | 4983 | 10 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| aggressive | 4984 | 10 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| aggressive | 4985 | 10 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| aggressive | 4986 | 10 | 0.9667 | 0.9667 | 1.0000 | 29 clean + 1 expected hold | PASS |

总计 180 条事务：179 clean、1 次预期 context hold、0 backend fault、0 unknown fault。

## 唯一非空原因审计

seed4986 的完整快照证明：

- `fault_reason=invalid_or_faulted_context_hold_previous_executed`；
- ContextState 为 invalid/FAULTED，原因为 `world_model_sequence_mismatch`；
- 同一 world model sequence 的 join 为 `EXACT_SEQUENCE_JOIN`，序号差 0，join valid；
- typed loop 正确保持上一 executed profile，没有发生活动参数写故障；
- 不属于 request、ack、readback、timeout、restore 或 unknown fault。

这也解释了上一轮 R2 将所有非空原因混为 transaction fault 的误判来源。

## 验收

- fault taxonomy、稳定窗口、seed 防火墙、资源哈希和 readiness-only 边界测试：5/5 通过。
- `teb_mode_manager`、`thesis_experiment`、`m2_gazebo`、`nav_world_model` 定向构建通过。
- 6 个探针均为一次尝试，没有重跑或预算扩张。
- seeds 5001--5010 未消费。
- `runtime_ready=false`、`formal_result=false`。

## 当前授权边界

6/6 通过只授权新建一个全新的 full calibration-only 预注册轮次，不授权恢复已经失败的
V2-04G-R2。下一轮必须继续冻结本轮 taxonomy、稳定窗口、R1 join 和 R2 候选数值，并使用
新的 readiness/navigation seeds，依次执行 readiness、TTC、Fixed 与三候选导航比较。

当前仍不授权候选冻结、held-out validation、V2-05、SAC 或实车闭环。
