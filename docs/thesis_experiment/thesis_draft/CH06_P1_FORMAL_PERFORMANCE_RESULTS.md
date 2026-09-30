# 第 6 章 P1 多种子配对性能实验结果

## 6.1 研究问题与结论先行

P1 检验的问题是：相对于 `r6_semantics_legacy_control`，`r6_semantics_circle_contact`
能否在三个冻结动态交互模板的全新随机种子分布上，减少 semantic-clear 条件中的错误动态
overlay，同时保持成功、碰撞、最小间距和真实冲突语义非劣，并达到预注册的受限完成时间改善。

P1 是完整执行的正式研究，但结果不是阳性性能结论。270 个预注册 episode 全部完成，完整性、
安全/任务非劣和机制门通过；受限完成时间的估计下降为 3.762%，方向、单侧区间和检验均有利，
但点估计没有达到预注册的 5% 最小效应量。效率门因此失败，四门合取为假：

```text
formal_study_complete=true
formal_result=true
performance_improvement_demonstrated=false
claim_authorized=false
runtime_ready=false
```

这里 `formal_result=true` 表示正式研究完整地产生并冻结了结果，不表示证明了性能提升。

## 6.2 预注册设计

### 6.2.1 确认性比较与背景参照

主确认性比较为：

```text
treatment = r6_semantics_circle_contact
control   = r6_semantics_legacy_control
```

`fixed_teb_reference` 在每个 block 中同步运行，但只作 contextual reference，不进入主配对检验、
四门合取或 winner 排名。三种方法使用相同的 scene-seed realization；差异只来自冻结执行模式。
实验不训练、不选择 checkpoint，也不增加任何训练步数。

### 6.2.2 场景角色与种子

P1 使用三个从 I5 模板冻结的 evaluator-only role：

| role | fresh seeds | blocks | expected TTC status |
|---|---:|---:|---|
| `single_true_conflict` | 5201--5230 | 30 | `OBSERVED_CONFLICT` |
| `multi_true_conflict` | 5231--5260 | 30 | `OBSERVED_CONFLICT` |
| `time_separated_semantic_clear` | 5261--5290 | 30 | `NO_CONFLICT_IN_HORIZON` |

因此统计独立单位是 90 个 scene-seed block，而不是 270 个方法行，也不是 episode 内的高频 TTC
样本。5201--5290 改变的是冻结模板下的 simulator stochastic realization；它们不是 90 个独立
几何设计。5291--5294 只服务于 manifest 编译完整性，不进入证据 schedule。

### 6.2.3 顺序平衡 schedule

每个 block 连续运行 legacy、circle-contact 和 fixed TEB 三种方法。30 个 wave 对三个 role 和六种
方法排列作确定性平衡：每个 role 内每种方法在位置 1/2/3 各出现 10 次，每种 permutation 各
出现 5 次。最终 schedule 包含 90 blocks、270 rows，其中 180 行属于确认性 treatment/control
比较，90 行属于背景参照。

关键冻结身份为：

- schedule SHA-256：`6c43cced674ae475dcf972fcdcca1872639e3b70fd0dc3156bc38c3b738e79fd`
- schedule episode digest：`e29530f28449a9b1bcbbb19090f1a9a3a4584e6fc4f9e1cc1f251cdf6f56f38f`
- execution release SHA-256：`05a6efe21a4c110be667b0873d8ac1b26ac692feefea8932f40a506ca9b9027b`
- attempt policy：每个 identity 仅 `attempt=1`
- authorized evidence budget：270
- retry / resume / replacement：禁止

## 6.3 终点与四门判定

### 6.3.1 受限完成时间

clean success 使用实测 navigation time；collision、timeout、planner failure 和其他算法失败保留并
赋冻结的 80 s 上限。对 30 个 semantic-clear pairs，令

\[
d_b=\log\frac{T_{b,\mathrm{legacy}}}{T_{b,\mathrm{circle}}},
\qquad
\Delta_T=1-\exp(-\bar d).
\]

效率门要求三项同时成立：\(\Delta_T\ge0.05\)，按 role 分层的 paired bootstrap 单侧 95% lower
大于 0，fixed-sequence adjusted \(p\le0.05\)。5% 是预注册实际效应阈值，不因样本结果而修改。

### 6.3.2 安全与任务非劣

成功、碰撞和 algorithmic emergency 使用 treatment harmful discordance rate 的单侧 exact
Clopper--Pearson 上界，并以 5 percentage points 为冻结界。minimum footprint clearance 使用 90
对 role-stratified paired bootstrap，要求 pooled 单侧 lower 以及三个 role 的 point estimate 均不低于
-0.05 m。60 个 true-conflict pairs 还要求 treatment expected-status match fraction 至少 0.95 且
不低于 control。

### 6.3.3 机制门

机制量为从首次 goal dispatch 到 terminal 的 timestamped non-`NONE` overlay time fraction。interval
必须无缝覆盖观察窗口，不能从消息条数近似时长。30 个 semantic-clear pairs 上要求 relative
reduction 至少 30%，paired absolute decrease 的单侧 lower 大于 0，并通过 fixed-sequence adjusted
检验。若 control mean 为 0，则 relative reduction 未定义并直接失败。

### 6.3.4 合取声明规则

只有 integrity、safety/task noninferiority、efficiency superiority 和 mechanism reduction 四门全部
通过，才允许写“在预注册的冻结模拟分布上性能提升”。任何单门失败都必须令
`performance_improvement_demonstrated=false`，即使其他终点方向有利或统计显著。

## 6.4 执行与证据完整性

唯一执行于 2026-07-21T12:17:15.292178Z 开始，于 2026-07-21T14:52:32.676593Z 完成：

| item | terminal value |
|---|---:|
| authorized / consumed / completed | 270 / 270 / 270 |
| forfeited | 0 |
| terminal integrity failures | 0 |
| attempts other than 1 | 0 |
| retries / resumes / replacements | 0 / 0 / 0 |
| training steps | 0 |
| real-vehicle episodes | 0 |
| I5 rerun or resume | false |

270 个 consumption receipt 的边界均为 `base_roslaunch_spawn_requested`；270 个 terminal journal 和
270 个 attempt root 均存在，schedule identity 与 normalized matrix 逐行一致。所有行的 terminal
status 都是 `completed`。

统一 raw replay adapter 对每个 episode 的 activation、evaluation、trace、clearance、overlay、
teardown receipt 和 process log 七项资源重新打开和哈希，共验证 1,890 项 raw binding。它从 raw
内容重算 success、collision、time、clearance、TTC status 和 overlay dwell，而不信任在线 summary。
终态后再次执行只读内存重放，得到的 normalized matrix、raw replay report 和 four-gate assessment
与落盘文件逐字节一致。

release 终态重验还验证了 456 个 workspace-local 和 301 个 external path+SHA binding。I5 的
13-resource freeze 清单保持 13/13 通过，最后没有 ROS/Gazebo/navigation 子进程或 11311/11345
监听残留。

核心 file SHA-256 为：

| evidence | SHA-256 |
|---|---|
| normalized terminal matrix | `07ee925756405029084ccf8a48ba02b60b38b0760171a71f0cbde6f08f678466` |
| raw replay report | `91a213e51d043938637be71faafc89169a61d61527bff22ab49c4c8ececc60c3` |
| four-gate assessment | `89147e95f19c461926e302982ab4db3772f1847fa0ca273e05b0df73fe2cd6df` |
| stage report | `1c625037365eea69b9363c61400e709195f9647211308b81100b395f96805853` |

## 6.5 结果

### 6.5.1 四门总览

| gate | result | decisive observation |
|---|---|---|
| integrity | pass | 270/270，1,890 raw bindings，0 integrity failure |
| safety and task noninferiority | pass | 0 harmful discordance；clearance lower 高于 -0.05 m |
| efficiency superiority | **fail** | 3.762% 小于预注册 5% minimum effect |
| mechanism reduction | pass | semantic-clear overlay relative reduction=100% |

### 6.5.2 安全与任务非劣结果

legacy 和 circle-contact 均为 90/90 success、0 collision、0 algorithmic emergency。三项 harmful
discordance 均为 0/90，其单侧 exact upper 为 0.032738；success 的保守 risk-difference lower 为
-0.032738，collision 与 emergency 的保守 upper 为 0.032738，均在 0.05 margin 内。

circle-contact minus legacy 的 minimum footprint clearance 平均差为 -0.029086 m，分层 paired
bootstrap 单侧 95% lower 为 -0.042891 m。三个 role 的平均差为：

| role | circle-contact minus legacy clearance (m) |
|---|---:|
| single true conflict | -0.010759 |
| multi true conflict | -0.032788 |
| semantic clear | -0.043713 |

pooled lower 与三个 role 点估计均高于冻结的 -0.05 m 下界。60 个 true-conflict pairs 中，legacy
和 circle-contact 的 expected-TTC-status match fraction 都是 1.0。因此安全/任务非劣门通过。
“通过”表示在该冻结模拟分布与 margin 下未观察到超界损害，不表示 circle-contact 更安全或提供
形式化无碰撞保证。

### 6.5.3 效率结果

30 个 semantic-clear pairs 的结果为：

| statistic | value |
|---|---:|
| legacy geometric restricted time | 19.6076 s |
| circle-contact geometric restricted time | 18.8699 s |
| geometric reduction | 0.037624 (3.762%) |
| one-sided 95% lower | 0.027904 (2.790%) |
| raw p | `9.99990000099999e-06` |
| fixed-sequence adjusted p | `9.99990000099999e-06` |
| circle-contact faster / slower | 27 / 3 pairs |
| arithmetic mean time difference | -0.743333 s |

方向、置信下界和显著性均支持时间下降，但预注册规则还要求点估计至少为 0.05。实际 0.037624
比阈值低 0.012376，即约 1.238 percentage points。因此 efficiency superiority 必须判为 fail。
不能把很小的 $p$ 值单独改写为性能提升，也不能在结果出现后删除 5% practical threshold。

### 6.5.4 机制结果

semantic-clear 条件下，legacy 的 mean non-`NONE` overlay occupancy fraction 为 0.176721，
circle-contact 为 0。relative reduction 为 1.0，paired absolute decrease 的单侧 95% lower 为
0.171762，raw 和 adjusted $p$ 均为 `9.99990000099999e-06`。机制门通过。

该结果支持一条限定机制陈述：circle-contact 在冻结 semantic-clear 模板的全新随机种子上消除了
错误动态 overlay 占用。它不等价于整体导航性能优越；效率门的失败正说明机制改善与预注册的
最小系统效应之间不能画等号。

### 6.5.5 fixed TEB 的描述性结果

以下均值只用于交代背景参照，不进入确认性推断：

| role | metric | legacy | circle-contact | fixed TEB |
|---|---|---:|---:|---:|
| single conflict | navigation time (s) | 20.9567 | 20.9567 | 17.4767 |
| single conflict | minimum clearance (m) | 0.8016 | 0.7908 | 1.0116 |
| multi conflict | navigation time (s) | 21.7200 | 22.3000 | 17.6833 |
| multi conflict | minimum clearance (m) | 0.8193 | 0.7865 | 1.0235 |
| semantic clear | navigation time (s) | 19.6167 | 18.8733 | 26.8133 |
| semantic clear | minimum clearance (m) | 2.7180 | 2.6743 | 0.0991 |

这些场景依赖的反向变化不用于选择 winner，也不改变 circle-contact 与 legacy 的预注册主比较。

## 6.6 结果解释与论文声明

P1 支持以下限定结论：

1. exact 270-row runner、统一 raw replay 和四门 assessor 在全部授权身份上形成了完整可复核链；
2. 在三个冻结动态模板的 seeds 5201--5290 模拟分布上，circle-contact 相对 legacy 通过了
   安全/任务非劣门；
3. 在 semantic-clear role 上，circle-contact 的错误动态 overlay 占用显著并实质性下降；
4. 受限完成时间呈 3.762% 的方向性下降，但未达到预注册 5% 最小效应量；
5. 因此 P1 没有证明性能提升。

建议正文表述为：

> 在预注册的三个冻结动态模板 fresh-seed 仿真分布上，circle-contact 语义通过了安全/任务非劣与机制下降门，并使 semantic-clear 配对的受限完成时间估计下降 3.762%；然而该点估计未达到预注册的 5% 最小效应量，因此本研究未证明性能提升。

不得将其缩写为“circle-contact 显著提升性能”。也不得宣称 whole-system superiority、跨 family 或
几何/时序泛化、learning gain、真实车辆性能/安全、形式化安全、winner 或 deployment readiness。

## 6.7 有效性限制

- 三个 role 来自三个冻结模板；30 个 seeds/role 表征模板内随机性，不覆盖更广几何和行为分布；
- 所有证据来自模拟器，不能直接外推到感知噪声、执行器延迟和参与者行为不同的真实环境；
- circle-contact 仍依赖短时相对运动、圆足迹和冻结 horizon；模型外加速度与遮挡未被证明覆盖；
- 5% practical threshold 是设计选择；P1 能说明结果未达到该预注册标准，不能证明 3.762% 在
  所有应用中都没有价值；
- fixed TEB 是背景参照，未被设计为与两个语义方法进行多重确认性比较；
- 完整性复核降低证据漂移与选择偏差，但不能证明 simulator 或统计模型本身完全正确。

## 6.8 本章小结

P1 在 fresh seeds 5201--5290 上完成了 90 个三方法 paired blocks 和 270 个 episode，没有重试、
替换、forfeiture 或完整性失败。安全/任务非劣与机制门通过，说明 circle-contact 在冻结模拟分布
内保留了预注册安全/任务边界并消除了 semantic-clear 错误 overlay。受限完成时间下降估计为
3.762%，但未达到冻结的 5% 最小效应量，故效率门与最终性能 claim 失败。该正式阴性结论与全部
原始证据一并保留，不再通过追加实验或事后改阈值升级。

## 6.9 post-P1 有限搜索与最终结论

P1 后续曾以独立合同冻结更严格的明显提升目标：相对 calibration-only 选出的最强公平固定 TEB
基线，受限完成时间点估计至少降低 10%，单侧下界至少降低 5%，并同时通过 success、collision、
emergency、clearance、projection 与 integrity 门。TEB-Default 只作为透明弱参考，不能取代
TEB-Tuned-V1 与独立公平调优 fixed TEB 中的最强安全者。

该 post-P1 计划限定为三个单因素轮次、2200 个新仿真证据单元和 120000 个训练步上限。实际执行
总账如下：

| round | capacity | consumed | retired | terminal boundary |
|---|---:|---:|---:|---|
| 1 | 792 | 2 | 790 | runtime/evidence profile identity mismatch |
| 2 | 704 | 1 | 703 | offline candidate-bank schema rejected before transaction |
| 3 | 704 | 1 | 703 | `m2_gazebo` package resolution failed before Gazebo startup |
| **total** | **2200** | **4** | **2196** | **remaining=0** |

训练步实际消耗为 0；三轮均未进入完整 calibration，更未访问新 held-out test。Round 3 前，
五个 candidate Bank 仅通过了严格离线 parser；最后一次 live identity 在 roslaunch package
resolution 阶段终止，Gazebo、move_base、参数事务和 raw trace 均未启动。因此不能用该链选择
candidate、最强 baseline 或 winner，也不能估计 post-P1 性能效应。

历史 P1 的 point=`3.7624%` 和 lower=`2.7904%` 均低于新冻结的 `10% / 5%` 门，post-P1
又没有产生可替代它的完整 test matrix，且 integrity 门失败。全局预算按合同耗尽并关闭后，
论文的最终表述为：

> 历史 P1 正式配对试验的受限完成时间点估计降低 3.7624%，单侧 95% 下界为
> 2.7904%，均未达到本轮预注册的 10%/5% 明显提升门；后续有限预算搜索因执行完整性失败
> 未形成完整校准或 held-out test 矩阵。全局 2200-unit 预算现已耗尽并关闭，因此本研究未证明
> FAM-TEB 相对最强公平固定 TEB 基线有明显性能提升。

终态机器结论 SHA 为
`8ce6736dd7ea1f13d6c272f0420f4588d1e0c9e25949b7b1eabf5bd028b3eeb7`，独立复核 SHA 为
`e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8`。不得重跑失败身份、
修复环境后复用 seed、增加 round、回收 retired budget、降低门槛或将离线 parser pass 写成
性能/安全阳性证据。
