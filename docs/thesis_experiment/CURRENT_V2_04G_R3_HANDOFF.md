# CURRENT V2-04G-R3 HANDOFF

日期：2026-07-14

## 结论

V2-04G-R3 已按要求建立为全新的 full calibration-only 轮次，并冻结 R1 bounded
join、R2 candidate bank、R2 幂等 typed transaction runtime 以及 R2-R1 fault taxonomy。
使用了全新的 readiness seeds 4991--4996 和预注册的 navigation seeds 5021--5035。

执行在 readiness 第 6/6 个探针被 hard gate 正确停止：5/6 通过，seed4996 出现两次
`EXPECTED_FAIL_CLOSED_CONTEXT_HOLD`，超过每个探针最多 1 次的预注册限制，同时事务有效率和
激活率均为 0.9333，低于 0.95。依据执行顺序和停止条件，TTC 与 60 个导航 episode 均未启动。

权威停止报告：
`artifacts/v2/calibration/v2_04g_r3/v2_04g_r3_stop_report.yaml`。

## 预注册边界

- readiness：6 个探针，每个只允许 1 次执行，seeds 4991--4996；
- TTC：三状态确定性 component probe，只能在 readiness 6/6 后执行；
- navigation：Fixed TEB 15 个 + 三个 R2 candidate 各 15 个，共 60 个；
- navigation seeds：5021--5035；
- 5001--5010 继续保留为未来 held-out validation，没有消费；
- 总预算 69 evidence units，不允许扩张；
- `runtime_ready=false`，不授权训练、V2-05 或实车。

合同与预注册文件：

- `config/thesis_experiments/v2/v2_04g_r3_full_calibration_contract.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r3_preregistration.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r3_scene_derivation.yaml`

## readiness 结果

| profile | seed | transaction valid | activated | join valid | taxonomy | 结论 |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| balanced | 4991 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| balanced | 4992 | 0.9667 | 0.9667 | 1.0000 | 29 clean + 1 expected hold | PASS |
| balanced | 4993 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| aggressive | 4994 | 1.0000 | 1.0000 | 1.0000 | 30 clean | PASS |
| aggressive | 4995 | 0.9667 | 0.9667 | 1.0000 | 29 clean + 1 expected hold | PASS |
| aggressive | 4996 | 0.9333 | 0.9333 | 1.0000 | 28 clean + 2 expected holds | FAIL |

总计 180 条事务：176 clean、4 次预期 context hold、0 backend fault、0 unknown fault。
每个探针均达到 10 条连续稳定消息；失败不是启动不就绪，也不是 typed TEB backend 故障。

## 故障定位

seed4996 的两次完整快照均显示：

- typed loop 原因是 `invalid_or_faulted_context_hold_previous_executed`，正确保持上一 executed profile；
- ContextState invalid/FAULTED 原因是 `world_model_sequence_mismatch`；
- mechanism join 30/30 均为 `EXACT_SEQUENCE_JOIN`，序号差为 0；
- backend、ack、readback、timeout、restore 和 unknown fault 均为 0。

因此 R1 bounded transaction/geometry join 和 fail-closed 行为是正确的。剩余缺陷位于更上游的
`rule_context_supervisor_node.py`：它分别保存 LocalGeometry、TrackedObstacleArray 和
WorldModelHealth 的“最新一条”消息，定时器要求三者 sequence 完全相同。世界模型依次发布三个
topic 时，timer 可能读到仅部分更新的一组消息，从而产生瞬时 mismatch。

## 验收与隔离

- 新增 R3 合同、场景、readiness/TTC/navigation runner、assessor 和 fail-closed 测试；
- R3/R2-R1/R2 相关测试 17/17 通过；
- 定向 catkin 工作空间构建通过；
- readiness 只运行一次，没有重试失败 seed；
- TTC 报告和 navigation episode 不存在；
- 没有启动 SAC、V2-05、实车或实车参数写入。

## 下一入口

不能恢复或重试本轮 R3。下一轮应是独立的 calibration-only 世界模型输入对齐修复：对
geometry/tracks/health 建立有界 sequence/timestamp 缓存 join（或发布原子组合快照），同时继续
冻结 R1 transaction join、R2 candidate 数值、幂等 transaction runtime 和现有 taxonomy。
必须使用新的 readiness seeds 再次达到 6/6，之后才能预注册另一个 full calibration 轮次。
