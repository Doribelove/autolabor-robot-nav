# FAM-TEB 论文正文草稿

日期：2026-08-08

当前论文写作入口为
`artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/`。其中
`FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf` 是 41 页 A4 的 V3+V6 真正整合稿，另含
自包含数据、11 张可编辑图、5 个三线表、combined claims、实车空白章节和另一台电脑 Codex
学校模板更新提示词。此前 V6-05 的 69 页文件只是既有 PDF 后追加 20 页附录，现作为历史导出
保留。引用结果前必须阅读 `CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md` 和
`CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md` 并保留证据边界。

本目录承载可直接并入论文的中文正文，而不是实验运行手册。当前版本包括：

- `CH03_FAM_TEB_SYSTEM_DESIGN.md`：系统目标、分层架构、TTC 语义、模式选择、可行解码与 P1 后的证据边界；
- `CH04_EXPERIMENT_INTEGRITY_METHOD.md`：预注册、哈希闭包、预算原子性、统一 raw replay、统计四门和终态 claim gate；
- `CH05_I5_INTEGRATION_RESULTS.md`：I5 六单元新鲜种子语义/执行集成结果；
- `CH06_P1_FORMAL_PERFORMANCE_RESULTS.md`：P1 的 90-block / 270-episode 正式配对结果；
- `CH07_V3_SCOPE_FORMAL_RESULTS.md`：V3-SCOPE 的空间候选设计、历史 comparator 研究、
  actual-TEB 公平基线、30-block / 60-episode 正式阳性结果，以及 V3-14 多演员支持度
  扩展的完整阴性结果和 V3-16 响应门控单因素实验；
- `CH08_V3_FOUR_METHOD_REPLICATION_AND_ABLATION.md`：四方法 30-seed/120-episode 新鲜复现、
  主基线/固定对比/单因素消融、失效计入统计、分层结果与历史阳性未复现解释；
- `CH09_PCT_SCOPE_RL_INTEGRATED.tex`：V3 到 V6 的架构关系、Asymmetric PPO、PCT、V5 阴性、
  V6 正式阳性、消融与 Gazebo 迁移阴性；
- `CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex`：只含协议和空表的实车实验占位；
- `CURRENT_THESIS_INTEGRATION_GUIDE.md`：把另一台电脑 V3 学校模板初稿更新到 V6 的逐章说明；
- `CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md`：可直接发送给另一台电脑 Codex 的论文更新提示词；
- `V3_PAPER_SUBMISSION_PACKAGE.md`：建议题目、摘要、贡献、核心图表和投稿前缺口；
- `CLAIMS_EVIDENCE_MATRIX.md`：论文陈述、直接证据、当前资格与禁止外推的对应表。

证据层级必须分开：

1. 仓库代码和组件测试支持系统机制已经实现；
2. I5 支持 fresh simulation semantic/execution integration；
3. P1 是完整执行的正式性能研究，但四门中效率门失败；
4. post-P1 三轮有限搜索以 4 consumed + 2196 retired 关闭全部 2200-unit 容量，没有形成完整
   calibration 或新 held-out test，完整性门失败；
5. 因此证据只支持安全/任务非劣、机制下降和 P1 3.762% 的限定描述性时间估计，不支持
   “明显性能提升”；
6. 没有证据支持 winner、真实车辆、跨分布泛化、形式化安全或 deployment readiness。
7. 历史 V3-09-F1 在 30 个 fresh 参数化反应式 single-actor crossing blocks 上，相对
   fixed-v1.20 shielded 直线策略得到 22.3844% 点效应和 21.8235% 单侧 lower；
8. 最新 V3-11-F1 在三个冻结 single-reactive-actor Gazebo 模板上，相对 actual
   TebLocalPlannerROS 0.8.4 得到 12.9288% 点效应和 12.7253% 单侧 97.5% lower；
   两项阳性结果均不改变 V2 的阴性结论；
9. V3-14-S1 的 5 单元预检和 40 单元证据矩阵均完整执行，动作身份绑定 45/45 通过，
   但共同前缀 actor 坐标/速度差分别达到 0.06607 m/0.07208 m/s，超过冻结的
   0.05 m/0.05 m/s 门。因此多人预测器拟合和后续 actual-TEB 多人评测关闭。
10. V3-16-S1 只把响应启用时刻改为 12.55 s，并用 fresh 45 单元复核；actor 速度差降至
    0.008597 m/s（下降 88.07%），但坐标差仍为 0.067472 m，仅 5/8 区块通过坐标门。
    因此只形成速度同步机制结论，整体多人支持仍未证明。

当前可直接阅读的终态论文为 32 页 A4 PDF：
`artifacts/v3/paper/v3_22_p2_thesis_export/FAM_TEB_V3_THESIS_DRAFT.pdf`。

PDF SHA-256：`22f4e423a4211590c2d4936c8be4a7f5d6963da83083e0dd8f86b0197af58b7d`；
独立 paper review SHA-256：`adb4317773484601dd3a1b2006098051521c611fdaa7073138991112a7a3d7a5`。
V3-17 的 25 页 PDF 作为历史版本保留。

V3-21 新鲜四方法表征的终态是：

```text
episodes/paired_blocks=120/30
v3_success/actual_teb_success=28/30,30/30
v3_time/actual_teb_time=7.785s/7.770s
v3_vs_actual_teb=-0.193%, two-sided 95% CI [-19.867%,12.982%]
v3_vs_no_candidate_conditioning=49.454%, 95% CI [37.001%,58.800%]
formal_result=false
characterization_result_available=true
```

该结果不删除 V3-11 的历史正式阳性，但不支持“稳定或普遍优于 Actual TEB”的更强陈述。

## V3-SCOPE 终态

```text
authorized/consumed/completed=60/60/60
retry=0
formal_result=true
performance_improvement_demonstrated=true
comparison=v3_spatial_safe_vs_actual_TebLocalPlannerROS_0_8_4
point_relative_reduction=12.9288%
one_sided_97_5pct_lower=12.7253%
goal_success=100%/100%
contact_state_count=0
scene_policy_strata_positive=6/6
shared_oracle_actor_track=true
actual_teb_comparison=true
runtime_ready=false
real_vehicle_ready=false
```

V3-12-P1 的 paper evidence manifest SHA 为
`319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`。

## P1 终态

```text
authorized/consumed/completed=270/270/270
forfeited=0
integrity=true
safety_and_task_noninferiority=true
efficiency_superiority=false
mechanism_reduction=true
formal_result=true
performance_improvement_demonstrated=false
claim_authorized=false
runtime_ready=false
```

效率门失败的精确原因是：geometric restricted-completion-time reduction 为 3.762%，单侧 95%
lower 为 2.790%，adjusted `p≈1e-5`，但点估计未达到预注册的 5% minimum effect。不得用显著性
绕过该实际效应门。

## post-P1 搜索终态

```text
global_capacity=2200
consumed=4
retired=2196
remaining=0
training_steps_consumed=0
new_held_out_test_accessed=false
obvious_performance_improvement_demonstrated=false
candidate_winner_frozen=false
```

最后一次 P4-I1 fresh identity 在 Gazebo 启动前因 `m2_gazebo` package resolution 失败，
没有产生参数事务或 raw trace。五个 promoted Bank 的 strict offline parser pass 不能替代
live 性能证据。终态结论与独立复核分别为：

- conclusion SHA：`8ce6736dd7ea1f13d6c272f0420f4588d1e0c9e25949b7b1eabf5bd028b3eeb7`
- review SHA：`e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8`

## 主要冻结证据

- I5 execution release：SHA-256 `9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43`
- I5 execution report：SHA-256 `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599`
- I6 interpretation review：SHA-256 `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68`
- P1 execution release：SHA-256 `05a6efe21a4c110be667b0873d8ac1b26ac692feefea8932f40a506ca9b9027b`
- P1 normalized matrix：SHA-256 `07ee925756405029084ccf8a48ba02b60b38b0760171a71f0cbde6f08f678466`
- P1 raw replay report：SHA-256 `91a213e51d043938637be71faafc89169a61d61527bff22ab49c4c8ececc60c3`
- P1 four-gate assessment：SHA-256 `89147e95f19c461926e302982ab4db3772f1847fa0ca273e05b0df73fe2cd6df`
- P1 stage report：SHA-256 `1c625037365eea69b9363c61400e709195f9647211308b81100b395f96805853`
- P4-F1 terminal conclusion：SHA-256 `8ce6736dd7ea1f13d6c272f0420f4588d1e0c9e25949b7b1eabf5bd028b3eeb7`
- P4-F1 independent review：SHA-256 `e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8`

## 使用约定

- “系统实现”只指仓库内已有、能由代码和测试定位的模块；设计意图不写成实验事实。
- I5 只证明集成正确性和可复核性，不承担性能推断。
- P1 的阳性性能 claim 已正式失败，不再是 pending；不能重跑、挑 seed、追加样本或事后改门。
- post-P1 性能搜索也已耗尽全局预算；不能修复后重跑 P4、增加 round、回收 retired budget 或
  访问新 held-out test。
- P1 population 是三个冻结动态模板在 seeds 5201--5290 下的随机实现，不是 90 个独立几何场景。
- fixed TEB 只作 contextual reference，不能据描述性均值选 winner。
- “性能提升”“更安全”“可部署”“泛化到真实车辆”等短语必须按 claims matrix 审查。
- 最终转为 LaTeX 时应保留证据哈希、效应阈值、阴性结论和限定语。
