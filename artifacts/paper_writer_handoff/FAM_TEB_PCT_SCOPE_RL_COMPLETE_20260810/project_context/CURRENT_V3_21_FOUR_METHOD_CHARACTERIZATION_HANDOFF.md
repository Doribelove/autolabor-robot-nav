# V3-21 四方法失效计入式表征终态交接

日期：2026-08-06

## 终态结论

V3-21-F1 已使用 fresh seeds 8201--8230 完成 30 个 paired scene--seed blocks、120/120 个
attempt-1 Gazebo episodes。四方法为：

1. `v3_spatial_safe` 主方法；
2. 实际构建的 `teb_local_planner/TebLocalPlannerROS` 0.8.4 主基线；
3. `fixed_v120_shielded` 固定比较策略；
4. `v3_no_candidate_conditioning` 单因素消融。

所有已消费失败都按 22 s 受限时间保留，没有 retry、resume、replacement、contact 或端口失败。

## 结果

| 方法 | 成功 | 受限均时 | 最小经验净空 | 回退 |
|---|---:|---:|---:|---:|
| V3 spatial-safe | 28/30 | 7.785 s | 0.4764 m | 2/30 |
| Actual TEB 0.8.4 | 30/30 | 7.770 s | 0.0395 m | 0/30 |
| Fixed-v1.20 shielded | 0/30 | 22.000 s | 1.0460 m | 30/30 |
| No candidate conditioning | 13/30 | 15.4017 s | 0.6640 m | 17/30 |

配对 block-bootstrap 比较：

- V3 vs Actual TEB：`-0.193050%`，双侧 95% CI `[-19.866838%, 12.982005%]`，Holm
  `p=0.755412`；未证明失效计入式时间优势；
- V3 vs fixed-v1.20：`64.613636%`，95% CI `[57.681818%, 69.250000%]`；
- V3 vs no conditioning：`49.453522%`，95% CI `[37.001028%, 58.800244%]`。

后两项是探索性比较，其中去候选条件化结果支持冻结总体内的候选条件机制价值。固定策略的 0/30
只代表该具体直行屏蔽策略过于保守。

## 协议偏差

sequence107、seed8227、V3 的 gate x 为 -2.567714 m，距 -2.6 m 的误差 0.032286 m，比
0.03 m 容差多 0.002286 m。该回合 goal reached，其他完整性/安全门通过，按 ITT 保留。
排除 block27 后 V3-vs-TEB 点效应为 -0.643444%，结论不变。因这一偏差：

```text
formal_result=false
characterization_result_available=true
performance_improvement_vs_actual_teb_demonstrated=false
```

## 前驱失败保留

- V3-19-Q1：24/24 完成，但 V3 5/6 goal 未过资格门，正式 seeds 8001--8030 退役；
- V3-20-F1：sequence78 的 ROS master 端口落入 ephemeral range，77 完成、1 terminal failed、
  42 forfeited；不重试；
- V3-21 只把端口公式改为 `24000 + 2*sequence`，科学方法和统计不变。

## 冻结证据

- release SHA：`94fee18fc9a809706d760fdbab3e7f95292dde2b553d160c965139eed66cc963`
- assessment SHA：`c195fc7cea4c649582bb3c0017e55a1b98f5de3b7f971737b1f2abcb5596ccc8`
- canonical CSV SHA：`772a7ee0f26d5c1541d995b928f37249626aadb4be4140937c18950300e438fe`
- evidence manifest SHA：`e9c1276096ca6c54018722bdb80907b13e8bbb3cd1c7ccd8d8bee3c04546c59d`
- independent review SHA：`450f4103e58e8e622c3adc192599a6e74fdc587430277432a642403729e6dd90`
- terminal freeze SHA：`66863adb769b6902aaacb723b281b24ca0b6d2bef58765ef7aa702586783ebe6`

## 论文解释

V3-11 的 12.9288% 是独立合同下的历史正式阳性；V3-21 是新的失效计入式表征，未复现 Actual
TEB 时间优势。两阶段必须分别报告，禁止池化、删除 timeout 或只选择有利 seed。合法终态是：

> 候选条件化消融机制证据明确，历史限定阳性存在，但新的严格失效计入式复现未证明 V3 相对
> Actual TEB 的时间优势。

## 禁止与下一入口

不得重跑/续跑 V3-19、V3-20 或 V3-21，不得复用 seeds 7901--8230、回收 forfeited/retired
budget、修改失败记录、放松安全门或升级为感知/OOD/多人/形式安全/部署/实车结论。

论文材料整理已经由 V3-22 完成。任何新的科学研究必须另立合同、fresh seeds、release、预算和
独立 review；实车运动或在线 TEB 写入仍须现场逐次批准。

