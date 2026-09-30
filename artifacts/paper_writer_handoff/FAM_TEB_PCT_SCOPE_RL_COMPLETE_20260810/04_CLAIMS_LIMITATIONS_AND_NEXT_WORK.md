# 声明边界、有效性威胁与下一步

## 可以写

- PCT 蒸馏在冻结 fast sensor-level population 上相对解析先验有 8.402444% 限定阳性。
- 后续 Gazebo 研究完成两轮公平、独立、fresh-seed 正式比较，但未达到优于 TEB 或 DWA 的
  预注册综合门。
- PCT 在两轮中的有效目标率高于 TEB/DWA；这与时间 superiority 是不同 estimand。
- R2 对 DWA 的区间为正，但 7.7754% 未达到 8% 实效门，故结论仍为阴性。

## 不能写

不能声称一般 TEB/DWA 优势、形式安全、感知/OOD 或多人泛化、部署、实车、GRU history benefit，
也不能把 V3 Gazebo 阳性改写成 PCT/V6 结果。不得合并 R1/R2 重新估计，不得隐藏传统规划器或
PCT 的负保守净空失败。

## 有效性威胁

新 formal 总体仍是六个参数化模板与 simulator seeds，不是独立几何分布；actor state 只用于
事后评价，但模拟传感器与真实激光仍有 domain gap。TEB 比较在 R2 pilot 功效推算中即使 200
blocks 也略低于 80% 目标功效，因此阴性不是等效证明。保守净空是冻结评价合同，不等于物理
contact，也不等于形式化可达集证明。

## 可证伪后续

只在全新 development identities 上测试一个“安全候选切换滞回/停车恢复释放”因素，事先要求
动态均时、stop、switch 同时改善，contact=0、mask=0 且净空不低于 control；不降低任何安全门。
LaserScan 时间戳 TF-buffer/warmup 健壮性应作为独立非行为因素测试。若未来再做正式研究，必须
重新冻结、功效分析、预注册并使用全新 formal seeds；197001–197160 和 198001–198200 永久只读。
