# CURRENT V2-04G-R4-R1 HANDOFF

日期：2026-07-15

## 结论

V2-04G-R4-R1 已完成独立 calibration-only 单因素修复：仅同步修改 Aggressive 的
`anchor_maneuver_forward` 与 `anchor_maneuver_reverse` 的 `min_obstacle_dist`，比较
0.28 m 原样控制、0.30 m 和 0.32 m。其余 supervisor、动态策略、时间参数、倒车状态机、
切换/滞回、typed transaction、join、evaluator 和原 hard-gate 阈值全部冻结。

readiness 6/6、TTC 组件 3/3 和导航 60/60 evidence 均完整。0.30 m 候选成功修复
Maneuver 净空，并保持成功率、时间、倒车、切换、事务和 bounded join 门，但新的动态
场景中仅得到 1 个 observed conflict 和 2 个 no conflict，未达到冻结的 2/1 TTC gate。
因此本轮没有 winner，不能冻结配置或进入 held-out validation。

权威机器结论：
`artifacts/v2/calibration/v2_04g_r4_r1/v2_04g_r4_r1_stage_report.yaml`。
SHA256：`e1ad0aeb7739e8c1abad0f17059f8dbe31c671dd03584d96637830033e5ab22a`。

## 单因素与 fresh seeds

- 控制：`r4r1_aggressive_control_m028`，0.28 m，不具 winner 资格；
- 修复候选：`r4r1_clearance_m030`，0.30 m；
- 修复候选：`r4r1_clearance_m032`，0.32 m；
- readiness seeds：5081--5086；
- navigation seeds：5091--5105；
- reserved held-out 5001--5010 未消费；
- 0.52 m inflation distance 保持不变，0.32 m 档仍满足 0.20 m coupling gap。

R4 的失败点经 scan 与 Gazebo truth 交叉审计后确认是真实几何余量不足，而非传感器或
evaluator 假象：scan 0.2414 m，truth 0.24495 m。

## 结果

| 方法 | 成功 | 总时间比 | Maneuver scan 最小净空 | truth 最小净空 | reverse | TTC conflict/clear/invalid | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| Fixed | 15/15 | 1.0000 | 参考 | 参考 | 0 | 2/1/0 | valid reference |
| 0.28 control | 15/15 | 1.0495 | 0.24568 m | 0.25485 m | 138 | 1/2/0 | scan/TTC 失败且不可选 |
| 0.30 repair | 15/15 | 1.0263 | 0.25210 m | 0.26348 m | 94 | 1/2/0 | 仅 TTC gate 失败 |
| 0.32 repair | 14/15 | 1.1118 | 0.25452 m | 0.26254 m | 128 | 1/2/0 | 成功率、TTC、总时间失败 |

0.30 m 相对控制将最小 scan 净空提高约 6.43 mm、truth 净空提高约 8.63 mm，并且三个
Maneuver episode 的 scan 净空全部越过 0.25 m。其 Maneuver 时间中位数相对 Fixed 改善
1.65%，总时间比 1.0263；因此净空修复本身有效。

TTC 失败集中在 fresh dynamic-conflict seed5094：Fixed 为 `OBSERVED_CONFLICT`，三个
Aggressive 变体均为 `NO_CONFLICT_IN_HORIZON`。由于 Maneuver 单因素不可能改变 Dynamic
anchor，这说明冻结的 Aggressive 动态交互时序对新的 seed jitter 不够稳健，不能把该失败
归因于 0.30 m 净空修复，也不能事后修改标签或 TTC 门。

## 边界

- 没有调用 winner freezer，没有生成 winner 配置或版本；
- 不复用 5091--5105 调参，不扩本轮预算，不改变 2/1 TTC 门；
- `runtime_ready=false`、`formal_result=false`；
- 不授权 held-out、V2-05、SAC、实车闭环或实车参数写入。

## 下一入口

若继续，应建立新的独立 calibration-only TTC 鲁棒性轮次：冻结本轮已证明有效的 0.30 m
Maneuver 净空值和全部其他已通过机制，只允许改变 Dynamic conflict 的预测/释放时序机制，
并先用全新 seeds 做 TTC activation/coverage readiness，再执行完整 fresh calibration。
阶段编号必须在新预注册时确定。
