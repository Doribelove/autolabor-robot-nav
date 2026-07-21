# FAM-TEB 论文正文草稿

日期：2026-07-21

本目录开始承载可直接并入论文的中文正文，而不是实验运行手册。当前版本包括：

- `CH03_FAM_TEB_SYSTEM_DESIGN.md`：系统目标、分层架构、TTC 语义、模式选择、可行解码与安全边界；
- `CH04_EXPERIMENT_INTEGRITY_METHOD.md`：预注册、哈希闭包、事务日志、失败保全、确定性重放与 claim gate；
- `CH05_I5_INTEGRATION_RESULTS.md`：I5 的六单元新鲜种子集成结果及其严格解释；
- `CLAIMS_EVIDENCE_MATRIX.md`：论文陈述、证据来源、当前资格与禁止外推的对应表。

这些章节把三个层级明确分开：

1. 已实现并经组件或集成证据验证的系统机制；
2. I5 已证明的模拟语义/执行集成性质；
3. 尚待最后一轮全新 multi-seed 配对实验检验的性能假设。

当前正文不得写成“已经证明性能提升”。I5 不是性能实验；其冻结证据只支持集成正确性和执行完整性。性能结论必须等待预注册的未来实验完成，并同时通过完整性、安全非劣、效率和机制四类门槛。

## 主要冻结证据

- I5 execution release：SHA-256 `9cef80f5c4eaf562719a71bb11fadd2cded7208d2ade07a22b09d7b6058b3d43`
- I5 execution report：SHA-256 `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599`
- I6 interpretation review：SHA-256 `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68`
- I5 critical evidence manifest：SHA-256 `40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935`
- future performance design：SHA-256 `a5b74aa99cb63785aa3993ba7cae40974baa3ab9b6aace71ee9cb815e08d379c`

## 使用约定

- 文中“系统实现”仅指仓库内已有、能由代码和测试定位的模块；未来设计不写成既成事实。
- 文中“I5”仅指冻结的六单元模拟集成验证；不得重跑、续跑或追加样本。
- 文中“性能提升”仅作为待检验假设出现，除非未来结果满足预注册 claim gate。
- 任何真实车辆结论、部署阈值、形式化安全或跨域泛化结论均不在当前证据范围内。
- 最终排版时可将这些 Markdown 章节转为 LaTeX，但证据哈希和限定语应保留。
