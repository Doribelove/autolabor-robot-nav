# V2-04G-P2-D1 纯离线 trace attribution 交接

日期：2026-07-22

## 1. 阶段结论

P2-D1 已完成，机器状态为：

```text
offline_trace_attribution_physical_room_pass
offline_trace_attribution_independent_review_pass
```

在 P1 的 30 个 `time_separated_semantic_clear` paired blocks（seeds 5261--5290）上，保留每条
circle-contact trace 从 goal dispatch 到首次 `anchor_cruise` 的真实前缀、原始二维路径、终点速度
约束和 P1 的 0.3 s terminal accounting，只把首次 cruise 后的同一路径按当前纵向限制
`max_vel_x=1.2 m/s`、`acc_lim_x=0.8 m/s²` 做 bang-bang 最短时间重计。所得 aggregate
same-path envelope 为：

| quantity | value |
|---|---:|
| P1 legacy geometric restricted time | 19.607582 s |
| P1 circle-contact geometric restricted time | 18.869873 s |
| P1 已观察下降 | 3.762364% |
| frozen-prefix post-cruise envelope time | 17.251842 s |
| envelope 相对 legacy 的空间 | **12.014433%** |
| paired bootstrap 单侧 95% lower | **11.264935%** |
| point / lower 超过 10% 的 margin | 2.014433 / 1.264935 percentage points |

因此，按预先冻结的 aggregate 判定规则，8%--10% 目标位于既有 trace 的同路径纵向物理包络内。
这完成的是“存在物理空间”的必要设计门，不是已经实现了 8%--10% 性能收益。

本阶段完全离线：没有启动 ROS、Gazebo、`move_base` 或训练，没有创建 release、attempt、journal
或 consumption，没有消耗 seed/evidence/training budget，也没有修改 P1/I5 证据、运行时 profile、
在线 TEB 参数或真实车辆。

终态保持：

```text
formal_result=false
runtime_ready=false
execution_ready=false
execution_authorized=false
evidence_budget_authorized=0
training_budget_steps=0
```

## 2. 证据与完整性闭包

分析合同：

```text
config/thesis_experiments/v2/
  v2_04g_r6_i1_p2_d1_offline_trace_attribution_contract.yaml
SHA-256 82297df5c26538e09f21c5c078dd1d8eb64fe431236d9932d5937deca6b5b31b
```

主要机器报告：

```text
artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/
  v2_04g_p2_d1_trace_attribution_report.yaml
SHA-256 957c1942364781d1239eeb1e86a50e8afde72aba617810f17d72c08f315f7793
```

逐 block 表：

```text
artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/
  v2_04g_p2_d1_trace_attribution_blocks.json
SHA-256 3b4bf51419eca7d1db4ca5b1c0d57e2a2936ff18863b026378c4e62653f6217d
```

独立 reviewer：

```text
src/tools/thesis_experiment/scripts/
  v2_04g_r6_i1_p2_d1_offline_reviewer.py
SHA-256 f91a258888a19fbff3cac26f5846000b597c9df02794efecfd49f214eb1f2bd2

artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/
  v2_04g_p2_d1_offline_review.yaml
SHA-256 66603dd9f64fb49f7f2a673c1b05e1171c95a7b4bba3b3d3aa1176cfb5a80342
```

reviewer 独立完成：

- 109 个 source closure path 全量重哈希；
- 30 个 treatment journals、30 个 raw trace 和 30 个 raw evaluation 的 journal SHA 复核；
- 30 条 post-cruise kinematic time 的独立公式复算；
- block table 与报告的确定性逐字节重建；
- 8% 和 10% 两个 target-recovery 解以及 cap-exposure 解的反代复核；
- P1 阴性四门结论、零执行授权和零预算边界复核。

没有 missing、retry、replacement、sample exclusion 或 integrity failure。

本阶段 10 项机器/文档产物另由下列 closure manifest 固定；只读 `sha256sum -c` 应为
10/10 成功：

```text
artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/
  p2_d1_offline_closure_freeze.sha256
```

## 3. trace attribution

30 条 treatment trace 均为 `anchor_balanced -> anchor_cruise` 一次切换、
`NO_CONFLICT_IN_HORIZON`、零 finite TTC 和全程 `NONE` overlay。均值归因为：

| observed component | mean |
|---|---:|
| goal dispatch 到首个运动命令 | 0.2000 s |
| goal dispatch 到首次 cruise | 1.1733 s |
| 首次 cruise 实际速度 | 0.6413 m/s |
| 到首次 goal-reached 的路径 | 18.2052 m |
| 首次 cruise 后路径 | 17.8379 m |
| cruise dwell | 17.4000 s |
| 实际速度达到当前 cap 95% 的 dwell | 9.2033 s（observation 的 49.64%） |
| 命令速度达到当前 cap 95% 的 dwell | 10.6733 s（observation 的 57.54%） |
| 当前包络下 post-cruise 可回收时间 | 1.6215 s |

按路径进度拆分，当前轨迹相对 rest-to-rest 同路径包络的平均 slack 为：

| phase | mean slack |
|---|---:|
| 0%--10% path | 0.1474 s |
| 10%--90% path | 1.1863 s |
| 90%--100% path | 0.3687 s |

主要空间位于 cruise 中段，其次是终端减速；它不是主要由 1.17 s 的 mode-entry 前缀造成。为保持
保守性，primary envelope 没有重计这段前缀。aggregate envelope 达到 8%/10% 分别需要回收
post-cruise 可回收时间的 51.40% 和 75.62%。单 block 中 29/30 达到 8% envelope，24/30 达到
10%；这里通过的是与 P1 estimand 一致的 30-block aggregate 门，不是“每个 seed 都至少改善
10%”。

## 4. 对候选因素的约束

当前 `anchor_cruise.max_vel_x=1.2` 和 `acc_lim_x=0.8` 已分别等于现有 bank 上界。因此：

1. 不能声称“在现有 bank 内继续提高这两个值”；现有范围没有上调空间。
2. 仅按实际 cap-bound dwell 做一阶 exposure scaling，达到 aggregate 8%/10% 对应的 speed
   multiplier 为 1.0990/1.1529，即约 `1.319/1.383 m/s`。这些值在现有 `1.2 m/s` bank 外，
   只是设计估算，不是合格候选，也没有更高速度的安全证据。
3. 历史单 seed、35 m straight calibration 的描述性单坐标检查中，`max_vel_x` 从 1.09 到
   1.20 m/s 时完成时间从 34.8 到 31.8 s，方向性下降 8.62%，路径变化 -0.020 m、minimum
   clearance 变化 +0.009 m；这支持速度因素具有 target-scale 响应，但不能外推到新上界或 P1
   分布。
4. 同一历史检查中，`weight_optimaltime` 从 1.03 到 1.37 的时间变化只有 1.25%，所以当前没有
   证据把单独提高 `weight_optimaltime` 当成 8%--10% 因素。

P2-D1 因而只保留 `semantic_clear_post_cruise_longitudinal_release` 这一因素假设类别，没有冻结
具体 runtime field 或数值。物理空间也可以来自在当前 cap 下提高纵向包络利用率；报告不把理想
轨迹偷换成某个参数的因果效果。

## 5. 可宣称与不可宣称

允许的准确表述为：

> 对 P1 已冻结的 30 个 semantic-clear paired blocks 进行纯离线同路径归因后，保留真实 pre-cruise 前缀的纵向运动学包络相对 paired legacy 的 aggregate 时间空间为 12.014%，单侧重采样下界为 11.265%；因此 8%--10% 是后续单因素设计的物理可达目标，但尚未证明任何具体参数能实现该收益。

不得宣称：

- P2 已经证明性能提升或获得 8%--10% 实测收益；
- 某个已命名 runtime 参数具有因果收益；
- `1.319/1.383 m/s` 已安全、合格或可执行；
- P1 的 3.762% 阴性四门结论被升级；
- 结果泛化到 conflict roles、其他 geometry、真实车辆或部署。

## 6. 下一安全入口

下一阶段只能是新的 **P2-D2 纯离线 single-factor design closure**，不得直接运行模拟。它必须：

1. 在“当前 cap 下提高 envelope utilization”与“扩展 semantic-clear 专用 speed ceiling”之间只选
   一个可执行 runtime field；不得同时改 `max_vel_x`、`acc_lim_x`、optimizer weight、空间参数
   或 mode threshold。
2. 若选择超过 1.2 m/s，先离线建立 registry/bank 边界、Ackermann yaw coupling、终端减速、
   资源与安全 margin；`1.319/1.383 m/s` 只能作为审查起点，不能直接冻结。
3. 保持 circle-contact semantics、overlay、路径/场景派生规则、评价器、安全非劣门与训练预算 0；
   不得用 `weight_optimaltime` 的 1.25% 描述性变化包装成 target-scale 证据。
4. 使用新的 preregistration、fresh seeds、exact schedule、path+SHA closure 和独立 reviewer。
5. D2 通过仍不等于执行授权；release、budget 和 simulation authorization 必须在后续阶段分别建立。

I5/P1 继续禁止 rerun/resume；seeds 5161--5163 和 5201--5290、历史 budget 均不得复用。

## 7. 只读复核

```bash
cd /home/robot/robot_ws_base_rl

PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p2_d1_trace_attribution.py \
  --workspace /home/robot/robot_ws_base_rl --check-only

PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p2_d1_offline_reviewer.py \
  --workspace /home/robot/robot_ws_base_rl --check-only

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/\
test_v2_04g_r6_i1_p2_d1.py
```

预期依次返回：

```text
offline_trace_attribution_physical_room_pass
offline_trace_attribution_independent_review_pass
5 passed
```
