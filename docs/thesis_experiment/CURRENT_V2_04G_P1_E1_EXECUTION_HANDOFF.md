# V2-04G-P1-E1 Execution Handoff

更新时间：2026-07-21

## 终态结论

`V2-04G-P1-E1` 已按唯一 release 完成 exact 270-row 仿真执行和统一 raw replay：

```text
status=exact_270_row_execution_and_raw_replay_complete
authorized=270
consumed=270
completed=270
forfeited=0
terminal_integrity_failures=0
formal_study_complete=true
formal_result=true
performance_improvement_demonstrated=false
claim_authorized=false
runtime_ready=false
training_budget_steps=0
real_vehicle_used=false
I5 rerun/resume=false
```

四门结果为：

| gate | result |
| --- | --- |
| integrity | pass |
| safety and task noninferiority | pass |
| efficiency superiority | **fail** |
| mechanism reduction | pass |

因此本研究是完整、有效、阴性的正式研究。不得宣称“性能提升”；也不得因为效率门失败而删去、
替换或重跑任何样本。

## 唯一执行身份与预算

- release SHA：`05a6efe21a4c110be667b0873d8ac1b26ac692feefea8932f40a506ca9b9027b`
- schedule SHA：`6c43cced674ae475dcf972fcdcca1872639e3b70fd0dc3156bc38c3b738e79fd`
- schedule episode digest：`e29530f28449a9b1bcbbb19090f1a9a3a4584e6fc4f9e1cc1f251cdf6f56f38f`
- fresh execution seeds：5201--5290；90 个 seed、90 个连续 paired blocks
- methods：`r6_semantics_legacy_control`、`r6_semantics_circle_contact`、
  `fixed_teb_reference`，各 90 episodes
- roles：`single_true_conflict`、`multi_true_conflict`、
  `time_separated_semantic_clear`，各 90 episodes
- 270 个 identity 均为 `attempt=1`，没有 retry、resume、replacement 或追加 seed
- 执行窗口：2026-07-21T12:17:15.292178Z 至 2026-07-21T14:52:32.676593Z

该 release 的 270-unit budget 已全部消耗。release 中的授权是已履行的历史事实，不是后续执行
许可；不得再次调用 runner、复用 5201--5290、恢复 attempt，或以 fixed reference 等名义追加行。

## 统一 raw replay 与完整性

终态 adapter 从 journal 绑定的 raw files 重算每行 success、collision、restricted completion
time、clearance、TTC status 和 timestamped overlay dwell，不信任在线 summary。

- 270 consumption receipts、270 journals、270 attempt roots
- 270/270 terminal status 为 `completed`
- 270 个 schedule identity 与 normalized matrix 逐行严格相等
- 1,890 个 raw resources（每 episode 7 个）全部重新哈希
- normalized matrix canonical SHA：
  `f3868c821a697d319cce703a2dbf9030c32efd2bc9ed0ddf574a13194710a1ca`
- raw replay canonical-object SHA：
  `47ad86639a056bd4906ff7bf8de708cfba491157da6812e3d6911584caa2a8c5`
- 终态后只读内存重放与 normalized matrix、raw replay report 和 four-gate assessment 三个
  落盘文件逐字节一致
- release 终态重验通过：456 个 workspace-local 和 301 个 external path+SHA bindings 全部匹配
- I5 evidence freeze 13/13 通过；I5 未重跑、未恢复、未修改
- 终态无 ROS master、roslaunch、Gazebo、move_base 或 P1 child process 残留，11311/11345
  无监听

`p1_e1_terminal_evidence_freeze.sha256` 是执行结束后建立的只读交接清单；它和本文件都不是
release 的运行时依赖，不会反向改写已冻结 release。

## 统计结果

确认性比较仅为 circle-contact treatment 对 legacy control；`fixed_teb_reference` 仍是 contextual
reference，不进入四门合取结论。

### Safety and task noninferiority：通过

- legacy 与 circle-contact 都是 90/90 success、0 collision、0 algorithmic emergency；success、
  collision 和 emergency 的 harmful discordance 都为 0/90，单侧 exact upper 为 0.032738。
- 60 个 true-conflict pairs 中 control/treatment expected-TTC-status match fraction 均为 1.0。
- circle-contact minus legacy 的平均 minimum footprint clearance 为 -0.029086 m；分层 paired
  bootstrap 单侧 95% lower 为 -0.042891 m，高于冻结 -0.05 m 非劣界。
- 三个 role 的 clearance mean difference 分别为 -0.010759、-0.032788、-0.043713 m，均高于
  -0.05 m role-level 门。

这些结果只支持冻结三模板 fresh-seed 仿真分布内的安全/任务非劣，不支持真实机器人安全、部署
安全或全场景泛化。

### Efficiency superiority：失败

在 30 个 semantic-clear confirmatory pairs 上：

- geometric restricted-completion-time reduction：0.037624（3.762%）
- 单侧 95% lower：0.027904（2.790%）
- raw/fixed-sequence adjusted p：`9.99990000099999e-06`
- 30 对中 circle-contact 更快 27 对、更慢 3 对；平均 navigation-time difference 为
  -0.743333 s

结果方向和统计显著性为正，但点估计低于预注册 practical-effect threshold 0.05。因此必须判定
efficiency gate fail，不能把显著性单独改写为“性能提升”。允许如实报告“观察到 3.762% 的方向性
时间下降，但未达到预注册 5% 最小效应量”。

### Mechanism reduction：通过

在同一 30 个 semantic-clear pairs 上，legacy 的 mean non-NONE overlay occupancy fraction 为
0.176721，circle-contact 为 0；relative reduction=1.0，paired absolute-decrease 单侧 95% lower
为 0.171762，adjusted p=`9.99990000099999e-06`。

这支持限定仿真分布上的机制性结论：circle-contact 消除了 semantic-clear 场景中的误动态 overlay
占用。它不等价于 whole-system 性能优越。

## 论文声明边界

允许写入论文：

1. P1 是预注册、fresh-seed、顺序平衡、270-episode 的正式仿真研究，并已完整执行。
2. 在冻结分布上，circle-contact 相对 legacy 通过安全/任务非劣和机制下降门。
3. restricted completion time 的估计下降为 3.762%，但未达到预注册 5% 实际效应阈值；P1
   因而没有证明性能提升。

禁止声明：whole-system superiority、learning gain、跨 family/geometry/timing 泛化、真实机器人
性能或安全、deployment readiness，以及任何无“未达到 5% 门”限定的性能提升表述。

## 核心证据

- stage report：
  `artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/p1_e1_stage_report.yaml`
- normalized terminal matrix：
  `artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/normalized_terminal_matrix.json`
- raw replay report：
  `artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/raw_replay_report.yaml`
- four-gate assessment：
  `artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/p1_e1_four_gate_assessment.yaml`
- terminal SHA freeze：
  `artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/p1_e1_terminal_evidence_freeze.sha256`

对应 file SHA 依次为：

```text
stage report      1c625037365eea69b9363c61400e709195f9647211308b81100b395f96805853
normalized matrix 07ee925756405029084ccf8a48ba02b60b38b0760171a71f0cbde6f08f678466
raw replay report 91a213e51d043938637be71faafc89169a61d61527bff22ab49c4c8ececc60c3
four-gate result  89147e95f19c461926e302982ab4db3772f1847fa0ca273e05b0df73fe2cd6df
```

## 下一安全入口

下一阶段只能是纯离线 result interpretation / paper closure：把阴性四门结论写入系统设计、
完整性方法和 P1 结果章节，并冻结可宣称/不可宣称文本。不得为跨过 5% 门而重跑、筛 seed、追加
样本或扩训练预算。

任何全新实验都必须重新提出独立科学问题、fresh seeds、独立 preregistration/review/release/budget，
并再次取得明确执行授权；本 handoff 不提供该授权。
