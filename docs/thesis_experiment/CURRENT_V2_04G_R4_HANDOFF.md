# CURRENT V2-04G-R4 HANDOFF

日期：2026-07-14

## 结论

V2-04G-R4 已按预注册顺序完整执行 readiness、TTC 三状态、Fixed TEB 15 episode、
三个候选各 15 episode 和 fail-closed hard-gate assessment。69 个证据单元全部在固定
预算内完成，60/60 导航 episode 均成功、无碰撞、无 persistent interface failure。

本轮没有候选通过全部 hard gates，因此没有冻结 winner，也没有生成 winner 配置、版本
或 SHA256。权威结论为：
`artifacts/v2/calibration/v2_04g_r4/v2_04g_r4_stage_report.yaml`。

## 固定边界

- 阶段：`V2-04G-R4`，calibration-only、simulation-only；
- readiness seeds：5051--5056；
- navigation seeds：5061--5075；
- 未来 held-out seeds 5001--5010 未消费；
- R3-R1 world-model input join、R1 transaction join、R2 candidate bank、typed runtime、
  taxonomy、supervisor 阈值、evaluator 和 hard gates 均保持冻结；
- 没有启动 SAC、V2-05、实车闭环或实车参数写入；
- 所有输出继续 `runtime_ready=false`、`formal_result=false`。

## Readiness 与 TTC

readiness 6/6 一次通过：180/180 transaction 为 CLEAN，transaction valid、activated、
transaction join valid 均为 1.0；world-model sequence mismatch、input-join fault、backend
fault 和 unknown fault 均为 0。

TTC 组件探针按预注册顺序稳定得到：

1. `OBSERVED_CONFLICT`；
2. `NO_CONFLICT_IN_HORIZON`；
3. `TRACKER_INVALID`。

导航 evidence 中每个候选也均为 2 个 observed conflict、1 个 no conflict、0 tracker invalid。

## 导航与候选结论

| 方法 | 成功/总数 | 碰撞 | 总时间比/Fixed | 最小净空 | Maneuver reverse | 结论 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Fixed TEB | 15/15 | 0 | 1.0000 | 仅作为有效性参考 | 0 | reference valid |
| `r2_control_g2` | 15/15 | 0 | 1.0990 | 0.4018 m | 0 | 机动、总时间和优先族效率失败 |
| `r2_target_balanced` | 15/15 | 0 | 1.1241 | 0.2649 m | 170 | 总时间与 Maneuver 时间失败 |
| `r2_target_aggressive` | 15/15 | 0 | 1.0302 | 0.2414 m | 123 | 仅最小净空 gate 失败 |

Aggressive 是最接近通过的候选：成功率、碰撞、TTC、typed transaction、bounded join、
chatter、总时间、全部 family time、priority family time 和 Maneuver reverse 全部通过。
唯一失败来自 `v2-04g-r4-maneuver-s5073` 的 0.2414019459 m 净空，低于预注册的
0.25 m。该差值不能通过事后放宽阈值消除。

Balanced 的三个 Maneuver reverse 样本分别为 53、57、60，但其中两个 episode 明显变慢，
导致 Maneuver 中位时间相对 Fixed 退化 67.49%，总时间比为 1.1241。

## 关键证据哈希

- stage report：`8fc0e980c84ff973bd4842ef4c542e5b136fb0a432d215093d7367f934c9ebd1`
- preregistration：`d68c6f04b029a573b68cf87bfbfd2ff1044bda910600b7a23b0daa613d85aa10`
- contract：`695fe8400649997a63a0aa2ade322d534e493c3a8367f064c38f315399af87d6`
- readiness summary：`cbc0836c3475a345d0cb1506f9af991ba0a47a4a1c29f236bfd503a1e2812807`
- TTC report：`672743846c96e5c5b32da6bf8343f2cc718d2bf5f87d36ed3171540fb1f9875a`
- navigation progress：`adf493dea0e57e10636affcaebaa9482998ed0097fba6e72d93606b58653519b`
- assessment：`188c3b57e7637479c03547b1ac25755167cb273dcb570f656f659ab20b6e55cc`

## 验收

- R4 与冻结依赖核心 pytest：38/38 通过；
- `catkin_make` 定向工作空间构建通过；
- readiness 6/6、TTC 3/3、navigation 60/60 evidence complete；
- freeze fail-closed 测试证明无 winner 时不会写出配置；实际 freeze 未被调用。

## 下一入口

R4 已完成但出口 gate 未通过。不得继续调 R4、不得复用 5061--5075 调候选、不得放宽
0.25 m 净空阈值，也不得生成 held-out validation。

如果继续，应建立独立的 calibration-only 单因素机制修复阶段，冻结本轮全部证据，针对
Aggressive 的 Maneuver 最小净空增加保守约束，同时尽量保持其已通过的时间与倒车机制；
必须使用全新的 calibration seeds，并重新依次执行 readiness、TTC 和完整导航比较。只有
新候选通过全部 hard gates 后才能冻结 winner。V2-05、SAC 与实车仍未授权。
