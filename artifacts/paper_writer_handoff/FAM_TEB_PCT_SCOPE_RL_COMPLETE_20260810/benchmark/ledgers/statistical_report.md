# 统计报告

状态：P01 历史快速总体科学复现通过；P02 Gazebo 迁移准入通过；P03 静态 development
组件门通过，其余核心场景与四方法公平调优未完成；正式比较尚未开始。

冻结 V6-02-F1 已在新目录独立重算：80 paired blocks、320 episodes，PCT/解析先验
60/80 对 49/80 goals，失效计入均时
13.115625/14.318750 s，相对降低 8.402444%，双侧 97.5% CI
[3.484003%, 13.713459%]。PCT 与冻结 V5 的次要相对降低为 3.161052%，双侧 95% CI
[0.493580%, 6.576508%]。四方法接触和 mask violation 均为 0。

复现等同性：320 行的全部科学字段精确一致；formal assessment 排除
`mean_inference_latency_ms`、`p99_inference_latency_ms` 和
`p99_episode_inference_latency_ms` 三项墙钟字段后精确一致；bootstrap distribution CSV
逐字节一致。墙钟字段依赖当前 CPU/调度，不是冻结科学结论。

范围限制：以上只支持四类冻结传感器级快速总体，不是 Gazebo 结果，也不包含 Actual TEB 或
DWA。本研究当前没有新的 Gazebo 正式比较、效应估计或优势声明。

## P02 development 准入描述

当前候选/运行版本的六个预声明 live gate 单元全部通过（static、single crossing、partially
occluded crossing 各含 analytic shadow 与 PCT takeover）。六个单元均 goal reached，contact=0，
mask violation=0；最低保守净空为 0.051338 m，episode 内最大 P99 推理时延为 6.878368 ms，低于
20 ms 准入门限。这些单位用于链路准入，不是随机样本，不计算方法效应或置信区间。

P02 全开发账本保留 39 个身份，其中 `goal` 13、`conservative_clearance_violation` 19、
`timeout_or_nonarrival` 6、`infrastructure_failure` 1。失败计数包含被修复前的单因素诊断与传统
规划器调参尝试；它们未被删除，也不会混入后续 pilot/formal 效应估计。

传统规划器只完成链路 smoke 和初步参数探索，尚未完成公平调优。动态场景出现零 contact 但
负的预声明保守净空，因此当前既不能声称 PCT 优于 Actual TEB/DWA，也不能计算正式完成率、
时间降低或置信区间。下一次统计更新必须来自顺序平衡的同 seed development blocks。

## P03 静态修复的 development 描述

静态修复序列保留 34 个互斥 development 身份：1 个 goal、24 个 timeout/nonarrival、8 个
execution-integrity failure、1 个 infrastructure failure。最终 seed196074 为 goal，限制时间
20.45 s，最小保守净空 0.224907 m，contact=0。该序列是自适应开发诊断，不能估计泛化率、
方法效应或置信区间；失败不可删除，最终门也不得进入 pilot/formal 统计。

## P04 六块 development 描述

同 seed 六场景自适应开发矩阵的失效计入均时为 PCT 15.6917 s（6/6）、解析先验 15.9083 s（6/6）、
TEB-v0 13.2833 s（4/6）和 DWA-v0 16.1167 s（3/6）。PCT 相对 DWA 的点改善约 2.64%，低于
预定 8% 实效门；相对 TEB 为负。该矩阵参与过调参，不能计算或报告正式置信区间，也不能支持优势声明。

离线互斥开发验证共 1,200 episodes。新模型 400/400、平均 16.710625 s；解析先验 400/400、
17.190000 s；历史模型 392/400、18.760000 s。三者 contact 与 mask violation 均为 0。

## P05 最终 development 冻结描述

六个配对开发块的失败计入均时为 PCT 14.9500 s（6/6）、解析先验 17.0750 s（4/6）、
Actual TEB 13.3000 s（4/6）、DWA 16.1083 s（3/6）。PCT 相对 DWA 的点改善为 7.1903%，
相对 TEB 为 -12.4060%。该矩阵参与了开发决策，禁止计算正式显著性或用作论文优势证据。

足迹对齐后的 1,600 回合离线验证中，解析/R1/R2/R3 均为 400/400 且 0 contact、0 mask violation；
均时分别 17.1700/16.7863/17.1456/17.1019 s。R1 按失败计入均时预先选择。下一阶段 pilot 仅用于
方差、功效和系统可行性估计，不允许调参。

## P06 pilot 与功效分析

预注册 pilot 共 20 个配对块、80 回合。失败计入均时为 PCT 15.1550 s、解析 16.7850 s、
Actual TEB 14.8525 s、DWA 16.0775 s；PCT 相对 DWA 的点改善 5.7378%，相对 TEB 为 -2.0367%。
pilot 仅估计方差，不构成优势检验。

按配对差标准差、8% 目标效应、双主比较 97.5% 临界值与 80% 功效，DWA 需 153 blocks，TEB
需 253 blocks。正式 R1 预注册 160 blocks；其 DWA 近似功效约 82%，TEB 比较因 200-block
上限而功效不足，必须作为限制报告。

## P07 formal R1 预注册

正式 R1 为 160 个 paired seed-scene blocks、640 回合。两项锁定主比较分别为 PCT 对 Actual TEB
和 PCT 对 DWA；每项使用 200,000 次配对块 bootstrap 的双侧 97.5% 区间。阳性需同时满足相对
失败计入时间改善至少 8%、区间下界严格大于 0、完成率差不低于 -5pp、contact 不更高且 PCT
mask violation 为 0。所有失败统一计 22 s。

## P08 formal R1 主结果

PCT/TEB/DWA 的失败计入均时分别为 15.0634/14.9225/16.0881 s。PCT 相对 TEB
为 -0.9442%（双侧 97.5% CI [-8.3569%, 5.6359%），相对 DWA 为
6.3693%（[1.0871%, 11.1411%）。阳性比较：none。
所有 160 个配对块、失败和安全门均进入统计。

## P09 R2 development 单因素消融

预注册的 12 个 development 配对块显示，0.6 m/s 候选相对 R1 control 的失败计入均时从
15.5917 s 降至 14.3917 s，绝对改善
1.2000 s；静态均值改善 5.6250 s，动态均值未退化。
包含首个预语义失败的主分析与删除整对 B001 的敏感性分析方向一致；由于这是开发数据，不计算正式置信区间，也不构成优于 TEB/DWA 的证据。

## P10 R2 pilot 与功效

R2 pilot 的 PCT/TEB/DWA 失效计入均时为 14.2750/
14.5376/16.0700 s；PCT 相对 DWA 点改善
11.1699%，相对 TEB 1.8064%。这些数值仅用于功效；正式 R2 固定为 200 blocks，指标、门限和方法均不改变。

## P11 formal R2 预注册

正式 R2 为 200 paired blocks、800 episodes。两项锁定主比较、估计量、8% 实效门、97.5% 配对
bootstrap、完成率 -5pp 非劣门及安全门与 R1 相同；bootstrap seeds 独立为 20261991/20261992。
所有失败统一计 22 s，正式结果不得用于继续调参。

## P12 formal R2 主结果

PCT/TEB/DWA 的失败计入均时为 14.8413/
14.4470/16.0925 s。PCT 对 TEB 的相对改善为
-2.7288%（97.5% CI [-9.8083%, 3.5914%），
对 DWA 为 7.7754%（[2.5521%, 12.6148%）。阳性比较：none。

## P13 论文回填

主结果、区间、分层和失败清单已确定性导出至 `THESIS_RESULT_TRANSFER.md` 及 `final_transfer/`。
R2 中 PCT 相对 DWA 在 partially-occluded/static-avoidance 快 8.3061/7.0338 s，但在
head-on-yield/single-crossing 慢 5.2666/4.6618 s。后两层的平均 certified-stop 数为
24.97/15.53，single-crossing 候选切换为 14.00；sequential-interaction 有 11 次超时。
这些是封存后的机制一致性归因，不是可在正式种子上调参的因果证明。

## P14 论文系统统一结果入口

研究级机器总表现包含 V3-11/V3-21、V4、V5、V6 fast population、历史 V6 Gazebo preflight 和
新 Gazebo R1/R2。当前 Gazebo 终局为两轮 360 paired blocks / 1,440 episodes，无阳性比较；R2
对 DWA 7.7754% 的正区间不能覆盖冻结 8% 点门。交接包只重组已封存结果，不产生新估计量。
