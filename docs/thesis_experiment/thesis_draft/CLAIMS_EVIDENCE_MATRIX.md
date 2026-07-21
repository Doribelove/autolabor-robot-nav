# 论文 Claims–Evidence Matrix

日期：2026-07-21

状态定义：

- **qualified**：现有冻结证据足以支持限定陈述；
- **pending**：已有预注册设计，但没有结果；
- **forbidden**：当前证据明确不足，正文不得写成肯定结论。

| 编号 | 论文陈述 | 状态 | 直接证据 | 必须保留的限定语 |
|---|---|---|---|---|
| C01 | 系统使用有限、类型化 Anchor Bank，而非无界直接参数输出 | qualified | V2-04 软件、配置与组件测试 | Anchor Bank 是未校准模拟候选 |
| C02 | 因子化模式可组合 base 与 scene overlay | qualified | 模式管理器实现与 shadow 证据 | 不等于已找到最优模式 |
| C03 | feasible decoder 检查类型、范围与运动学耦合 | qualified | decoder 源码、合同和测试 | 不覆盖全部真实动力学 |
| C04 | 平滑以 previous-executed 而非 commanded 为基准 | qualified | manager/decoder 实现和回归测试 | 不声称性能优越 |
| C05 | circle-contact TTC 以相对圆接触为冲突事件 | qualified | TTC 实现、组件测试和 I5 语义证据 | 使用短时运动与圆足迹近似 |
| C06 | 参数事务记录 commanded/feasible/safe/executed 链 | qualified | transaction 源码、journal 和执行证据 | 真实车辆在线写入未授权 |
| C07 | I5 使用全新 seeds 5161--5163 完成 6/6 单元 | qualified | execution report SHA `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599` | 纯模拟、六单元集成验证 |
| C08 | I5 没有 retry、resume、terminal failure 或 forfeiture | qualified | stage report 与 deterministic assessment | 只针对 I5 身份 |
| C09 | I5 全部 expected/observed TTC 状态一致 | qualified | 6 journals + 36 raw bindings + execution report | 语义/执行集成结论 |
| C10 | semantic-clear 配对中 legacy non-`NONE` 为 18，circle-contact 为 0 | qualified | I5 sequence 5/6 raw traces 与重放报告 | 描述性单 block 观察，不是效应估计 |
| C11 | I5 关键证据可由确定性 assessor 重放 | qualified | execution report、I6 review SHA `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68` | 可复核性不等于科学假设为真 |
| C12 | 历史失败和失败 release 被原样保留 | qualified | freeze manifest SHA `40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935` | 修复使用新的阶段身份 |
| C13 | circle-contact 提升模拟导航性能 | pending | future design SHA `a5b74aa99cb63785aa3993ba7cae40974baa3ab9b6aace71ee9cb815e08d379c` | 只有未来四类门全部通过才可写 |
| C14 | 性能改善至少 5% 且统计显著 | pending | 90-block / 270-episode 预注册设计 | 当前没有执行结果 |
| C15 | semantic-clear 错误触发至少降低 30% | pending | 未来机制 gate | I5 的 18 对 0 不能代替多种子估计 |
| C16 | 性能改善不牺牲成功、碰撞或间距 | pending | 未来非劣 gate | 必须逐项通过预注册 margin |
| C17 | 已证明跨未见分布泛化 | forbidden | 无 | 需要独立 held-out 分布研究 |
| C18 | 已证明真实车辆性能提升 | forbidden | 无 | 没有实车授权或实车数据 |
| C19 | 已证明形式化安全或无碰撞保证 | forbidden | 无 | 当前仅有约束与经验评估 |
| C20 | 已校准 winner 或可部署阈值 | forbidden | I5 明确 `winner_ranked_or_frozen=false`, `runtime_ready=false` | 不得改写状态 |
| C21 | 新增 SAC 训练带来提升 | forbidden | future design 固定 additional training steps = 0 | 本轮不扩训练预算 |

## 正文审查规则

1. 所有 “qualified” 陈述必须同时带上该行限定语。
2. 所有 “pending” 陈述只能写为假设、设计或待检验问题。
3. 所有 “forbidden” 陈述不得出现在摘要、结论、图题或宣传性文字中。
4. `simulation_integration_validation_pass` 不得改写为 `performance_pass`。
5. I5 的高频语义样本不得作为独立 $n$；I5 的独立配对 block 数是 3。
6. 未来性能实验若失败，矩阵中的 C13--C16 改为“not demonstrated”或具体权衡，不得通过追加样本升级。

## 性能结论升级条件

只有未来预注册实验同时满足以下条件，C13 才能从 pending 升级为 qualified：

- release 和执行得到新的明确授权；
- 90 个预注册 block 按停止规则完整记账；
- 完整性报告无失败；
- 成功、碰撞、间距和真实冲突保持通过非劣门；
- 受限完成时间改善点估计至少 5%，且确认性区间/检验通过；
- semantic-clear 错误触发减少至少 30%，且机制区间/检验通过；
- 结论限定为预注册的新鲜模拟动态交互分布。

上述条件是 claim gate，不是保证结果成功的调参目标。
