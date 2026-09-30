# 与旧 V6-06 交接包的关系

`upstream_v6_06_historical/` 是 2026-08-08 的完整历史论文包，内容和散列保持不变。它仍是 V3、
V5、V6 fast-population、历史 Gazebo preflight 和 41 页整合稿的权威来源。

唯一需要更新的时间性陈述是：旧包写作时“V6/PCT 没有 Gazebo 正式矩阵”；2026-08-10 后续研究
已经完成 Gazebo 正式 R1/R2。因此新论文必须新增后续研究，同时保留旧 preflight 失败。新 formal
并未建立优势，所以不得把更新写成成功迁移。`claims_contract.yaml` 对 Gazebo 相关措辞拥有最高
优先级；旧包其他冻结结果不因新研究而失效。
