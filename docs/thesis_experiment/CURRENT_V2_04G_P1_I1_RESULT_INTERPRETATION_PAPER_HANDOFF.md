# V2-04G-P1-I1 纯离线结果解释与论文闭环交接

日期：2026-07-21

## 1. 阶段结论

P1-I1 已完成，机器状态为：

```text
offline_result_interpretation_paper_closure_pass
```

本阶段只读取并重放既有 P1/I5 证据，冻结结果解释和论文文本。它没有启动 ROS、Gazebo、
`move_base` 或训练，没有创建 execution release、attempt、consumption receipt 或 journal，没有
消耗 seed/evidence/training budget，也没有连接真实车辆或写入在线 TEB 参数。

终态权限为：

```text
offline_only=true
formal_result=true
runtime_ready=false
execution_ready=false
execution_authorized=false
evidence_budget_authorized=0
training_started=false
real_vehicle_used=false
```

`formal_result=true` 表示 P1 正式研究完整完成；它不等于阳性性能结论。

## 2. 冻结结果解释

P1 四门为：

| gate | result |
|---|---|
| integrity | pass |
| safety and task noninferiority | pass |
| efficiency superiority | **fail** |
| mechanism reduction | pass |

效率点估计为 3.762%，单侧 95% lower 为 2.790%，adjusted `p≈1e-5`。方向、区间和显著性均有利，
但点估计低于预注册的 5% minimum effect，因此效率门与四门合取失败：

```text
performance_improvement_demonstrated=false
claim_authorized=false
```

机制门支持 semantic-clear overlay 从 legacy mean fraction 0.176721 降至 circle-contact 0；安全/
任务非劣门支持两种语义 90/90 success、0 collision，并满足 clearance 与 true-conflict 语义 margin。
这些子结论不能合成为“性能提升”。

## 3. 合同与确定性复核

纯离线合同：

```text
config/thesis_experiments/v2/
  v2_04g_r6_i1_p1_i1_offline_result_interpretation_paper_contract.yaml
SHA-256 559ef0994f7848256237c94871c94af0db5d86a414b2af70c4a9f867e888aec9
```

reviewer：

```text
src/tools/thesis_experiment/scripts/
  v2_04g_r6_i1_p1_i1_result_interpretation_reviewer.py
SHA-256 2c75e319600d92919e42d02f68a2dc817f96246803c5acfb918f2256329dcf33
```

review report：

```text
artifacts/v2/integration/v2_04g_r6_i1/p1_i1_interpretation/
  v2_04g_p1_i1_result_interpretation_review.yaml
SHA-256 fb5637fbdee9cad20ac04918a861061e2a006c0c9569af7fb3ee596d05529e2e
```

reviewer 确定性验证了：

- P1 terminal freeze 10/10、I5 freeze 13/13；
- P1 release 的 456 个 workspace-local 与 301 个 external bindings；
- exact schedule / normalized matrix 的 270 行、90 paired blocks 和 90 fresh seeds；
- 270 journals 和 1,890 raw resources；
- normalized matrix、raw replay report 与 four-gate assessment 的内存重放逐字节相等；
- 6 份论文文件与合同 SHA 相等；
- `formal_result=true`、efficiency fail 和 `claim_authorized=false` 没有被混淆。

## 4. 论文正文

已冻结的正文包括：

1. `CH03_FAM_TEB_SYSTEM_DESIGN.md`：把待检验假设改为 P1 已完成且效率门失败的设计结论；
2. `CH04_EXPERIMENT_INTEGRITY_METHOD.md`：加入 exact consumption boundary、七资源 raw replay、
   四门统计定义和 P1 终态；
3. `CH05_I5_INTEGRATION_RESULTS.md`：保持 I5 集成边界，并补入后续 P1 正式阴性结果；
4. `CH06_P1_FORMAL_PERFORMANCE_RESULTS.md`：完整记录 90-block / 270-episode 设计、完整性、统计、
   fixed TEB 描述性结果、限制和结论；
5. `CLAIMS_EVIDENCE_MATRIX.md`：将原 pending 性能项改为 `not demonstrated`，并冻结 allowed /
   forbidden claims；
6. `README.md`：更新论文目录、证据层级和 P1 终态。

paper closure freeze：

```text
artifacts/v2/integration/v2_04g_r6_i1/p1_i1_interpretation/
  p1_i1_paper_closure_freeze.sha256
SHA-256 71d22a8e80b0ad63e610f873a3b6813828dc6c88e849ad92ab40457353aa2641
```

清单固定合同、reviewer、review report、6 份论文文件和 P1 terminal evidence freeze，共 10 项。

## 5. 论文可宣称与不可宣称

必须使用的核心表述为：

> 在预注册的三个冻结动态模板 fresh-seed 仿真分布上，circle-contact 语义通过了安全/任务非劣与机制下降门，并使 semantic-clear 配对的受限完成时间估计下降 3.762%；然而该点估计未达到预注册的 5% 最小效应量，因此本研究未证明性能提升。

允许拆分报告：P1 完整完成；安全/任务非劣通过；mechanism reduction 通过；方向性 restricted
completion time reduction 为 3.762%；阳性性能 claim 未通过。

禁止：navigation/whole-system performance improvement、safety superiority、跨 family/geometry/
timing 泛化、learning gain、真实车辆性能或安全、形式化安全、winner/best profile、deployment
readiness。

fixed TEB 只作 contextual reference；三个 role 是三个冻结模板的 stochastic-seed 分布，不是 90 个
独立几何场景。

## 6. 后续边界

P1 的唯一 270-unit release 已耗尽且不可再次执行。本 closure 不授权任何新模拟、训练、实车或
在线参数写入。不得为跨过 5% 门而重跑、扩样本、筛 seed、修改阈值或删除不利样本。

下一安全工作仅包括不改变上述冻结结论的论文排版、图表生成、语言润色、LaTeX 转换和答辩材料。
如果未来提出不同科学问题，必须使用新的 preregistration、fresh seeds、独立 review/release/budget
和新的明确授权；不得复用 P1 或 I5 身份。

## 7. 只读复核

```bash
cd /home/robot/robot_ws_base_rl
source devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_i1_result_interpretation_reviewer.py \
  --workspace /home/robot/robot_ws_base_rl --check-only

sha256sum -c \
  artifacts/v2/integration/v2_04g_r6_i1/p1_i1_interpretation/p1_i1_paper_closure_freeze.sha256
```

预期分别返回：

```text
offline_result_interpretation_paper_closure_pass
10/10 成功
```
