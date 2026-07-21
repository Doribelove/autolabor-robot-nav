# FAM-TEB V2-04E/F Supervisor Repair and Fresh Validation Handoff

更新时间：2026-07-14 CST

## 当前结论

V2-04E--E4 calibration-only 规则监督器修复和 V2-04F 全新 held-out 三方法配对均已完成。
V2-04D 的 seeds 4601--4610 未进入候选生成、阈值选择、停止判断或冻结。V2-04F 的 seeds
4801--4810 只在监督器冻结后才生成和预注册，validation 结果没有回写监督器或 Anchor Bank。

新的无标签监督器已经修复两个核心机制缺陷：Cruise 不再被单个静止 track 的
`static_persistence=1` 吸收到 Static；Maneuver 可通过“前方受限 + 双侧围合 + 后方可退”的
局部口袋几何触发。它还加入了 active-mode exit hysteresis、challenger score margin 和独立的
Balanced 退出确认。冻结文件为
`src/application/teb_mode_manager/config/v2_04e4_rule_supervisor_frozen.yaml`，SHA256 为
`2c3a8669418d9221be6c314df06d1ce936a1ba414117ce5e47fc6b35f9bdfe1d`。V2-04C
Anchor Bank 保持原哈希
`b1365f2b14e4a77f4e715bc46369fa4277e9adbb8522190153f7abd2ce635b9d`。

V2-04F 的 30/30 episode 全部形成有效证据。Fixed、Balanced、Rule 均为 10/10 成功、
0 Gazebo contact collision、0 持久接口失败，因此**任务成功率不退化已经在全新 held-out
seeds 上证明**。但是全部预注册 hard gate 没有通过，性能有效性仍未证明，V2-05、SAC 和实车
继续不授权，`runtime_ready=false`。

## 监督器修复内容

1. Static persistence 密度支撑：新 profile 不再用 `max(density, static_persistence)` 让单个
   静止 cluster 直接占据 Static；persistence 必须乘以 obstacle-density support。
2. Maneuver 口袋证据：除 dead-end 和反向路径航向外，新增 front/left/right/rear clearance
   组合；运行时仍不读取 family、manifest、Gazebo truth 或 evaluator 输出。
3. 抖动控制：每个 mode 有 exit confidence；非当前 challenger 必须超过 score margin；进入确认
   与退回 Balanced 的退出确认分离，冻结值分别为 0.8 s 和 10.0 s。
4. 统计窗口：模式占比和切换次数从 readiness 完成、导航目标即将发送时开始，启动等待流量不再
   计入导航行为。
5. `FeatureSnapshot` 正式接入已有 `signed_heading_error_rad`、left/right clearance；旧
   V2-03/V2-04D 配置缺少新字段时保留兼容语义，不改写历史实验。

## Calibration-only 证据链

| Stage | Seeds | Budget | 结论 |
|---|---|---:|---|
| V2-04E | 4701--4705 | 4 candidates × 5 = 20 | 20/20 success；Cruise/Static 分离，但 4 候选 Maneuver 均 0%，无 winner |
| V2-04E2 | 4711--4715 | 3 candidates × 5 = 15 | 口袋证据使 Maneuver 激活；候选 G 5/5 success、Maneuver 23.4%、max switch 3，但 readiness 污染 Cruise 占比 |
| V2-04E3 | 4721--4725 | 1 fixed candidate × 5 = 5 | 只修 evaluator 窗口；5/5 success、Maneuver 23.6%，但 Cruise/Static switches 5/4 |
| V2-04E4 | 4731--4735 | 1 single-factor candidate × 5 = 5 | 退出确认 4.5→10.0 s；5/5 success、0 collision、全部 hard gate 通过 |

V2-04E4 冻结证据：Cruise 中 Static fraction 0；Maneuver fraction 20.44%；五类
switch counts `[1,3,1,0,1]`，最大 3、均值 1.2。assessment 为
`artifacts/v2/calibration/v2_04e4/v2_04e4_assessment.yaml`，SHA256
`ee08d4b5010d3ac76e25cee348a8ed6351d76bf2347355eef4231c4368dd215e`；freeze report 为
`artifacts/v2/calibration/v2_04e4/v2_04e4_supervisor_freeze_report.yaml`，SHA256
`71d2a144032e5190867269113e0aa39a08acc4904806ed8d9eb11e87059b2ae2`。

最初 V2-04E 的 calibration split adapter 有两次 evaluator hash mismatch。两次导航结果均未被
接受为 evidence，原日志移至 `artifacts/v2/calibration/v2_04e/excluded_interface_attempts/`；
R1 amendment 只修 evaluator 恢复原 compiled split 后再做 hash 校验，候选、seed、预算和门槛
均未变。

## V2-04F fresh held-out 结果

| Method | Success | Collision | 全部场景最小净空 | 总导航时间 | 实际 Anchor |
|---|---:|---:|---:|---:|---|
| Fixed TEB | 10/10 | 0 | 0.254 m | 203.0 s | n/a |
| Balanced Anchor | 10/10 | 0 | 0.000 m | 293.4 s | Balanced |
| Rule Multi-Anchor | 10/10 | 0 | 0.429 m | 268.4 s | Balanced/Cruise/Static/Corridor/Maneuver |

Rule 的 held-out 机制证据：

- 两个 Cruise episode 的 Static fraction 均为 0，Cruise 被 Static 吸收问题通过；
- 两个 Maneuver episode 的 Maneuver fraction 为 22.16%/18.50%，实际激活
  `anchor_maneuver_forward`，Maneuver 不触发问题通过；
- 五种 Anchor 全部实际激活；
- 10 个 Rule switch counts 为 `[3,4,3,1,1,2,1,3,1,1]`；一个 Cruise seed 为 4，超过
  预注册最大 3，因此 held-out chatter mechanism gate 失败。

安全/证据质量阻塞：

- Balanced dynamic-s4807 虽然 action result 为 success 且没有 contact collision，但统一
  LaserScan footprint clearance 最小值为 0.000 m，低于 0.25 m hard gate；
- 6 个跨方法 Dynamic episode 的 `TRACKER_INVALID=0`，但 `OBSERVED_CONFLICT=0/6`，低于
  80% TTC 覆盖门槛；这批 validation 不能用于重新调整动态时序。

描述性配对结果（hard gate 失败，禁止作正式性能改善声明）：

| Family | Rule time vs Fixed | Rule time vs Balanced | Rule path vs Fixed | Rule clearance vs Fixed |
|---|---:|---:|---:|---:|
| Cruise | +3.9% | -20.4% | -0.03% | +0.000 m |
| Dynamic | +4.0% | -25.5% | -0.01% | -0.789 m |
| Static Dense | +73.3% | +12.7% | +0.59% | +0.118 m |
| Corridor | +60.7% | +26.0% | +0.04% | -0.001 m |
| Maneuver | +51.2% | -10.8% | -7.17% | +0.182 m |

因此准确表述是：**新监督器在 held-out seeds 上修复了 Cruise/Static 混淆并实际触发 Maneuver，
同时保持 10/10 成功；但净空、抖动、TTC 覆盖和相对 Fixed 的时间效率 hard gate 未通过，整体
性能有效性没有证明。**

## 机器入口与哈希

- V2-04F 合同：`config/thesis_experiments/v2/v2_04f_fresh_paired_validation_contract.yaml`；
- 预注册：`experiments/manifests/v2/validation/v2_04f_preregistration.yaml`；
- fresh validation scenes：`artifacts/v2/validation/v2_04f/v2_04f_paired_validation_scenes.yaml`；
- progress：`artifacts/v2/validation/v2_04f/v2_04f_progress.yaml`，SHA256
  `ea88d132e3f39549d421337e2bb1d62f37d540ac1c7efb331ff130cc730dff09`；
- assessment：`artifacts/v2/validation/v2_04f/v2_04f_paired_assessment.yaml`，SHA256
  `44aa92906fa177108ca03768a2f537f07c9c45725c25c8b5b581fa2c7c1d6694`；
- 人读报告：`artifacts/v2/validation/v2_04f/V2_04F_PAIRED_VALIDATION_REPORT.md`，SHA256
  `1efd88d5b61c68d890d1eeeddd7e978dc4cc9e2ff890363919a2ed477a671c38`。

## 不可越过的边界

- V2-04D 和 V2-04F validation 数据都不得用于阈值选择、候选排名或参数回写；冻结监督器和
  Anchor Bank 不因 validation 结果修改。
- V2-05 不授权；SAC/任何学习不授权；实车闭环和实车参数写入不授权。
- 后续若继续，必须另建新的 calibration-only 合同和新 seeds，在 calibration 中独立解决
  Dynamic 可观察交互、净空风险和效率问题，再冻结后使用第三组全新 held-out validation；
  不得把 4801--4810 转成 calibration。
- 当前最强的正面结论仅是成功率不退化和两项模式机制修复，不得写成总体性能提升。

