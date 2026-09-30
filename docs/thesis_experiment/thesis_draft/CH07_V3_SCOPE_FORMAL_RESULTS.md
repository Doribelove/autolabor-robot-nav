# 第 7 章 V3-SCOPE：空间候选、安全验证与正式仿真结果

## 7.1 研究动机

V1 的 Residual SAC 暴露了高维 TEB 参数动作与实际执行之间的投影错位；V2 的 P1 正式研究
又表明，仅修改 TTC 语义虽能改善机制指标，但受限完成时间只改善 3.7624%，未达到 5% 的
预注册最小效应量。V3-SCOPE 因此不再把高维 TEB 参数作为主要学习动作，而把决策对象改为
一组预先生成、满足 Ackermann 约束的候选轨迹。候选在排序前经过候选条件交互预测、校准
占据管和硬安全验证；只有被验证器接受的轨迹才可进入解析排序器。

本章回答一个限定而可检验的问题：

> 在参数化反应式单行人横穿仿真中，冻结的 V3 空间安全候选规划器能否相对冻结的
> fixed-v1.20 shielded 直线策略，减少受限目标完成时间，同时保持目标成功、净空、
> certified stop、精确重放和规划延迟门？

该问题对应历史 V3-09-F1 主研究；其确认性比较器不是 TEB 实例。为补足这一外部效度
缺口，后续 V3-11 另立 fresh seeds、actual `TebLocalPlannerROS` 公平执行合同与
多场景正式矩阵，结果见 7.11。两个研究的 seed、release、统计总体和声明边界相互独立。

## 7.2 系统设计

V3 的执行链分为五层：

1. 从共同前缀场景快照生成 Ackermann 可行的空间候选；
2. 对每个 ego candidate 预测 actor 的条件响应；
3. 用冻结的 conformal 半径构造保守可达占据；
4. 检查曲率、净空、制动和下一承诺域，并保留 certified-stop 备份；
5. 解析排序器只在安全候选集合内选择，输出仍受现有硬件命令安全链约束。

候选动作不是被安全层静默投影的参数向量。每次拒绝都带有明确 rejection code，每次接受、
选择与执行都绑定 scene snapshot、candidate-set digest 和 decision digest。V3-04 的离线
learning-value audit 发现，已有 closed-loop 日志对未选动作没有正 propensity，且解析排序器
在观察到的即时进度目标上没有 regret。因此学习门按预注册规则关闭，训练步数为 0；正式
评测使用解析规划器，不将其包装为 RL 结果。

## 7.3 证据形成过程

### 7.3.1 反应式 actor 与预测

V3-01 建立 cautious、assertive 和 hesitant 三种 evaluator-only actor policy。隐藏类型只能
用于分层评估，不得进入 runtime planner。V3-02 逐步修复共同前缀与动作可行性，最终得到
满足加速度约束、包含 certified stop 的候选支持。

空间分支 V3-07 的 B0-R1 使用 50 个 train/cal episode 冻结 predictor，模型 SHA 为
`38d8c0fa7c7a696fb9ffdd5c3ce399d760d52fc42a0c6041525c5d70f7445fd4`，
conformal 半径为 0.496740409 m。B1 在 20 个 fresh held-out episode 上获得 100% global
coverage 和 100% per-head coverage，最大位置误差 0.24733 m，最小保守净空 0.5538 m。
该结果只适用于冻结的空间横穿场景族。

### 7.3.2 空间安全规划器

V3-08-A0 在 520 个离线 fixture 上验证空间候选与硬安全过滤，P99 规划延迟为 5.247 ms。
A1 首次在线集成暴露了一个证据序列化错误：被拒候选的 `-inf` 诊断不能进入 canonical
JSON。A1-R1 只将非有限的拒绝诊断归一为有限哨兵值，随后得到 17/18 目标成功的有效阴性
结果。A2 证明失败 episode 只差 0.5 s 即到达目标；A3 只把时长从 12.0 s 改为 12.5 s，并
修复 receipt stage identity，在 fresh seeds 6691--6696 上完成 18/18。

### 7.3.3 配对资格门

V3-09-Q1 发现共享 0.5 s 重规划周期对冻结的 0.25 s baseline commitment 不公平，且 baseline
certified stop 在制动余量门处被拒。该阶段在 1 个完整单元后永久停止。Q1-R1 只把双方共享
重规划周期改为 0.25 s，在 fresh seeds 6721--6726 上完成 36/36，得到 22.63% 描述性改善并
通过资格门。资格数据没有进入正式推断。

## 7.4 正式实验设计

V3-09-F1 在读取任何正式 outcome 前冻结：

- 30 个 fresh seed blocks：6801--6830；
- 每 block 三种 actor policy，两个 paired method，共 180 episodes；
- 每个身份只允许 attempt 1，禁止 retry、resume 与 replacement；
- 主要比较：`v3_spatial_safe_vs_fixed_v120_shielded`；
- 统计单位：seed block，而非 episode、decision 或仿真帧；
- 主要量：block 内三策略平均受限目标时间的配对相对下降；
- 20,000 次 block bootstrap，固定 seed 30909；
- 点估计门：至少 10%；
- 单侧 95% lower 门：至少 5%；
- positive block fraction 门：至少 80%；
- V3 目标成功率至少 95%，且不低于 comparator；
- conservative-clearance violation 为 0；
- accepted selection、certified stop、精确重放和 stage identity 均为 100%；
- planner latency P99 不超过 20 ms；
- 任一门失败即形成有效阴性正式结果。

受限目标时间对未到达目标的 episode 赋值 12.5 s。所有几何、actor policy、方法与执行顺序
由 exact schedule 固定。

## 7.5 正式结果

180/180 个 episode 均在 attempt 1 完成，没有 retry、resume、replacement 或 forfeiture。
两种方法均为 90/90 目标成功。

| 指标 | V3 spatial-safe | fixed-v1.20 shielded |
| --- | ---: | ---: |
| episode 数 | 90 | 90 |
| 目标成功率 | 100% | 100% |
| 受限目标时间均值 | 8.8611 s | 11.4167 s |
| 受限目标时间中位数 | 7.50 s | 11.25 s |

30 个 block 的配对相对下降点估计为 22.3844%，单侧 95% bootstrap lower 为 21.8235%。
两者分别超过 10% 与 5% 的冻结门；30/30 block 的聚合差均为正。因而
`formal_result=true` 且 `performance_improvement_demonstrated=true`。

分 actor policy 的均值为：

| actor policy | V3 spatial-safe | fixed-v1.20 shielded | V3 相对方向 |
| --- | ---: | ---: | --- |
| cautious | 7.5000 s | 11.7667 s | 更快 |
| assertive | 11.5833 s | 10.8917 s | 更慢 |
| hesitant | 7.5000 s | 11.5917 s | 更快 |

该分层结果说明，V3 的总体收益来自对 cautious/hesitant 交互的空间绕行或更早安全推进；
在 assertive actor 下，保守空间验证会产生约 0.692 s 的平均代价。正式结论针对三类 actor
等权构成的预注册总体，并不声称每个 policy stratum 都优越。

## 7.6 安全、完整性与计算代价

5,320 个 planner decisions 全部完成确定性重放。selected candidate accepted、
certified-stop available、exact-core replay 和 receipt-stage identity 均为 100%。最小保守
净空为 0.165663302 m，保守净空违规为 0，非有限统计量为 0。

整体 planner latency P99 为 18.087844 ms；V3 spatial-safe 与 comparator 分别为
15.629119 ms 和 18.548193 ms，均处于冻结的 20 ms 门内。这里的“安全门通过”是对冻结
仿真合同的经验完整性陈述，不是形式化无碰撞证明。

正式 assessment SHA 为
`d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b`，
独立 result review SHA 为
`206732c86b536e7f9a520c58dac6d386ff6752595bdc767d4957e4804e5f8d5a`。
冷启动复算与 persisted assessment 字节一致。

## 7.7 与 V2 和早期 V3 的关系

V2 的正式阴性结果保持不变：P1 只得到 3.7624% 点估计和 2.7904% lower，未达到其 5%
最小效应门。V3-05 的早期纵向候选正式研究也保持阴性：1.2007% 点估计、0.9647% lower，
且 V3 目标成功率只有 93.33%。V3-09-F1 的阳性结果来自新的空间候选、空间预测与安全
验证链，以及后续 fresh formal population；它不能用于把 V2 或 V3-05 的历史阴性结果
改写为阳性。

V3-04 没有授权学习。因此本章证明的是解析空间安全规划器的限定收益，不是 SAC、bandit、
IQL 或 CQL 的学习收益。

## 7.8 有效性威胁

第一，确认性总体只有一个参数化反应式单 actor crossing family。30 个 blocks 是不同仿真
随机实现与参数组合，不代表 30 个独立场景家族。第二，actor policy 是工程构造的
cautious/assertive/hesitant 模型，不能代替真实行人的行为分布。第三，comparator 是冻结的
fixed-v1.20 shielded 直线策略，不是 actual TEB，也未构成对所有主流局部规划器的排名。
第四，净空与制动基于仿真模型和保守占据，不等于硬件标定或形式化安全证明。第五，本研究
没有真实车辆、传感器噪声、定位漂移、多人交互和复杂静态障碍的确认性证据。

因此当前结果适合作为“反应式横穿仿真中候选条件空间规划”的限定算法证据。若要形成更强
的通用导航论文结论，必须新建 actual-TEB paired baseline、multi-scene/multi-actor、
分布外压力和现场安全验证；不得复用 V3-09-F1 seeds 或追加其样本。

## 7.9 可复核论文资产

V3-10-P1 从冻结 assessment 导出三份 CSV、一张 SVG 和 claims YAML。证据 manifest SHA 为
`70353b355c6eec2f733901ca7d4a6820de550beff00999cfacdb359dec951938`，
独立闭环 review SHA 为
`bee5891b08f860a97a25ea71047b3e5fabb0c193695f221de131eacb5dcfe0f4`，
终态 manifest SHA 为
`57f043fdd485f01c6d194623acaedcee50d66447c635ae776bfb2b433c9ccf54`。

历史 V3-09-F1 的两份 SHA 清单含字面 `\n` 分隔符。该格式问题不影响直接 raw replay 和
独立 assessment，但不符合常规 `sha256sum -c` 输入。V3-10-P1 不改写历史文件，而在新
manifest 中绑定其原始 bytes，并使用真实换行重新形成规范清单。

## 7.10 本章结论

在 30 个 fresh 参数化反应式单 actor crossing blocks、180 个 paired episodes 的预注册
正式仿真中，V3 spatial-safe 相对 fixed-v1.20 shielded comparator 将平均受限目标时间从
11.4167 s 降至 8.8611 s，配对改善 22.3844%，单侧 95% lower 为 21.8235%，双方目标成功
均为 100%，且全部审计安全、重放与延迟门通过。

这一结果支持限定场景内的解析空间候选规划收益；它不支持 actual-TEB 优越、跨场景泛化、
学习收益、形式化安全、部署或实车性能声明。

## 7.11 Actual TEB 公平基线扩展

### 7.11.1 基线与公平执行合同

V3-11 使用工作区实际构建的
`teb_local_planner/TebLocalPlannerROS` 0.8.4 作为主要基线。插件库 SHA 为
`7d17474e7901821d7f69bf10cdb909e4445f9010e32ee045452ad42c6ba1883e`，
`fixed_teb.yaml` SHA 为
`9e3125eec7f1f9040356f245dae9620d0b866ac94482ac29fd2d600838b14c26`，
`max_vel_x=1.2 m/s`，动态障碍输入开启。

V3 与 actual TEB 共用 M2 Ackermann Gazebo 车辆、执行器动力学、1.2 m/s 速度上限、
`/cmd_vel_navigation` 命令边界、起始前缀、handoff gate、目标、22 s timeout 和 paired
simulator seed。双方还共用 Gazebo model-state 派生的 actor oracle track；TEB 通过
`/move_base/TebLocalPlannerROS/obstacles` 接收该 track，同时保留 laser/costmap。这个设计
隔离规划器差异，但不构成感知鲁棒性证据。

场景总体由三个冻结模板构成：open 90° crossing、wide corridor 70° oblique crossing 和
bounded corridor 110° mirrored crossing。V3 冻结预测器只支持 cautious/hesitant，因此
assertive 被预先排除，而不是依据正式结果删除。

Q1-R1 在 fresh seeds 7021--7032 上完成 24/24 资格回合。双方均 12/12 目标成功、零
contact；最大 paired gate actor coordinate/speed 差为 0.00320 m/0.02560 m/s；V3
spatial candidate 选择率、actor mode match 与 common gate validity 均为 100%。资格
assessment 和独立 review SHA 分别为
`d90f44ff5fde177b764bc71c4f42c2c60ebb2a3403d42651983de334e02adbbe`
与
`2d7674f52e33f6a002d4719404e9b8b0ab2ea3f59787d7f914dd8b6467bd4c3e`。

### 7.11.2 正式设计

V3-11-F1 在读取正式 outcome 前冻结：

- fresh seeds 7101--7130；
- 30 个 scene-policy-seed paired blocks / 60 episodes；
- 六个 scene×policy strata，每层 5 blocks；
- 方法顺序 counterbalanced；
- 每个身份仅 attempt 1，禁止 retry、resume、replacement 和 selective exclusion；
- 主要量为 22 s 限制下的 paired mean gate-to-goal time reduction；
- 200,000 次 paired-block bootstrap，seed 31100；
- 单侧 alpha 0.025；
- point reduction 至少 10%，单侧 lower 至少 5%；
- 每个 scene×policy stratum mean reduction 不低于 0；
- contact 为 0、保守净空非负、V3 目标成功不劣于 actual TEB。

执行 release SHA 为
`b21835d5d55a55fd130aae836c609fd117481b9d4cd54155ba8fc78b57dc9ea6`。

### 7.11.3 正式结果

60/60 episodes 均在 attempt 1 完成，没有 retry、resume、replacement 或 forfeiture。

| 指标 | V3 spatial-safe | Actual TebLocalPlannerROS |
| --- | ---: | ---: |
| episode 数 | 30 | 30 |
| 目标成功率 | 100% | 100% |
| 平均受限 gate-to-goal 时间 | 6.7683 s | 7.7733 s |
| 最小经验保守净空 | 0.5110 m | 0.0192 m |

配对均值降幅为 12.9288%，单侧 97.5% bootstrap lower 为 12.7253%，分别超过 10% 和
5% 门。六个分层的均值降幅为 12.5161%--13.5723%，方向全部为正。两方法 contact 总数
为 0；V3 最大 tracking error 为 0.03312 m；actual TEB 每回合至少发布 75 条 local plan，
并以 `SUCCEEDED` 结束。因而
`formal_result=true` 且 `performance_improvement_demonstrated=true`。

正式 assessment SHA 为
`9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`，
独立 review SHA 为
`2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39`。

首次冻结分析在构造两个未加引号的 claim-scope 标签时触发 `NameError`，没有写出
assessment。R1 只向冻结模块注入 `"cautious"`/`"hesitant"` 两个字符串；没有修改
正式数据、配对、bootstrap 随机序列或门槛。失败与 repair release 均已保留，repair SHA 为
`677d29954e216665c414ffb9f11b460e1a7f9d32d4a6a44f427177b95a23e0b5`。

### 7.11.4 更新后的有效性边界

V3-11 补足了“比较器不是 actual TEB”的主要缺口，但仍只是三个工程构造的
single-reactive-actor Gazebo 模板。双方使用 oracle actor track，未测试检测、跟踪或定位
退化；没有 multi-actor、非横穿交互、独立复杂地图、分布外压力或实车验证。actual TEB 的
最小经验保守净空仅 0.0192 m，零 contact 不能解释为无条件安全排序。训练步数仍为 0，
结果属于解析规划器而非 RL。

## 7.12 更新后的本章结论

历史 V3-09-F1 证明 V3 相对理想化 shielded comparator 的限定收益；新的 V3-11-F1 进一步
在 30 个 fresh paired blocks、60 个一次性 episodes 上证明，相对 actual
`TebLocalPlannerROS` 0.8.4，V3 将平均受限 gate-to-goal 时间从 7.7733 s 降至
6.7683 s，配对降幅 12.9288%，单侧 97.5% lower 12.7253%，双方目标成功 100%、contact
为 0，六个 scene×policy strata 均为正。

因此目前可以发表的核心结论是：在冻结的单 actor 反应式横穿仿真范围内，候选条件空间
规划相对 actual TEB 获得了预注册且可复核的明显时间收益。该结论不外推为感知鲁棒、
多人/OOD 泛化、形式化安全、学习收益、部署或实车性能。

## 7.13 多演员扩展与终止性阴性结果

V3-13 尝试把单演员候选条件预测扩展到两个反应式 actor。历史 R19-R6 首先暴露执行完整性
缺陷：40 行调度虽然覆盖五个动作，旧运行器却把所有行都硬编码为
`recovery_evasive_right_v120_r3`。该阶段在两个身份被消耗后停止，科学证据计数为 0；失败
release、raw report 与审查记录均被保留，未被修补或续跑。

V3-14-I1 随后建立了失效关闭的动作绑定门。每个单元在写入 `unit_consumed` 前，必须证明
调度行中的 candidate、seed、scene、block、propensity 与候选特定 horizon 分别等于唯一的
roslaunch override；运行报告生成后，还必须再次证明 candidate、seed、scene 与 block
相等。五个候选的正向检查、20 种跨候选启动替换和 20 种运行时替换测试全部通过。

新的 V3-14-S1 使用 fresh seed 7719 完成 5/5 非科学预检，再使用 seeds 7721--7728 完成
8 个匹配区块、40/40 科学 episodes。45 个单元均为 attempt 1，没有 retry、replacement、
forfeiture 或动作身份漂移；全部过程、采集、有限值、branch clock、ego pose、trace、r3
terminal guard 与 boundary-atomic 门通过。然而两个预注册的反事实配对门失败：

| 指标 | 观测最大值 | 冻结门槛 | 结果 |
| --- | ---: | ---: | --- |
| 同区块跨候选 actor 坐标差 | 0.066074 m | 0.050000 m | 失败 |
| 同区块跨候选 actor 速度差 | 0.072081 m/s | 0.050000 m/s | 失败 |
| branch clock 绝对误差 | 0.008000 s | 0.012000 s | 通过 |
| ego 决策 x 差 | 0.000000 m | 0.040000 m | 通过 |

最大坐标与速度差出现在 seed 7721 的 `occluded_staggered_corridor` 区块；此外 seed 7723
与 7726 的最大坐标差也分别为 0.050626 m 与 0.060978 m。该模式说明，车辆共同前缀虽然
稳定，两个闭环 actor 在候选分支边界前仍存在超过冻结容差的状态漂移。因而这 40 个
episodes 不能作为无偏的五动作 cloned-intervention 支持集。

按预注册合取门，V3-14-S1 是完整而有效的阴性结果：
`evidence_collection_gate_pass=false`，`predictor_fit_authorized=false`。没有拟合新的 r3
多演员 predictor，也没有启动 actual-TEB 多演员、感知/OOD、训练或实车阶段。确定性
assessment SHA 为
`58c472560699e10276f76c063dd243ab5666f8e534346c51e88aa9f2b49958b9`，独立 result review
SHA 为
`4773d76437a01148984770ca502dce96cfa6d3bc2743029e0bb7a1f10f94c60a`。

这一终态同时强化了 V3-11 的声明边界：单演员 actual-TEB 阳性结果仍成立，但当前项目未
证明多演员反事实可比性、多人预测器有效性、多演员规划收益或感知鲁棒性。

## 7.14 V3-16 共同前缀响应门控单因素实验

### 7.14.1 离线归因

V3-16-D1 对 V3-14 的 40 份科学回执进行纯离线、同路径重放。所有 action identity 与 ego
pairing 均通过；actor 在 warmup gate 的最大坐标差仅 0.00512 m，但在 12.5 s 分支边界达到
0.06607 m。插件与时序证据支持一个可证伪解释：actor 在候选动作生效前已经闭环响应，微小的
门时序差被放大为速度和位置差。归因 assessment SHA 为
`1f84987b14a9a8721eea5823e965d7c0283f0a4250f7efb7137cf060b08e7396`，独立 review SHA 为
`2a1b92099420861074b73540b9c4a9ff9322ef93c7f4607eda7c248c0c8cbd73`。

### 7.14.2 单因素设计与实现

唯一行为因素是 `reactive_actor_response_enable_sim_s`：control 为 0 s，V3-16 treatment 为
12.55 s，即在 12.50 s 候选分支之后 50 ms 才允许 actor 使用原有 cautious/hesitant 闭环响应。
场景几何、actor 初态/路径/基础速度、policy、车辆控制、五动作集合、采集逻辑和门槛全部冻结。
插件参数默认 0 s，因此历史世界行为不变；只有 V3-16 world 显式写入 12.55 s。

预检使用 seed 7741，5 个回合不进入科学证据；正式支持使用 fresh seeds 7742--7749，形成
8 个区块 × 5 动作 = 40 个回合。每个身份仅 attempt 1，禁止 retry、resume、replacement、
selective exclusion。C++、Python 与 ROS 测试通过；独立 release review SHA 为
`74830894bfccb9710970eae06184a51e9cea639b2dd886206aedf36674296852`。

### 7.14.3 结果

5/5 预检通过，最大 actor 坐标/速度差为 0.00828 m/0.00414 m/s。随后 40/40 正式回合全部
完成；45/45 均为 attempt 1，无重试、替换、forfeiture 或 action/response identity mismatch。

| 指标 | V3-14 | V3-16 | 冻结门槛 | V3-16 结果 |
| --- | ---: | ---: | ---: | --- |
| actor 坐标最大差 | 0.066074 m | 0.067472 m | ≤0.050000 m | 失败 |
| actor 速度最大差 | 0.072081 m/s | 0.008597 m/s | ≤0.050000 m/s | 通过 |
| branch clock 最大误差 | 0.008000 s | 0.008000 s | ≤0.012000 s | 通过 |
| ego 决策 x 最大差 | 0.000000 m | 0.016001 m | ≤0.040000 m | 通过 |
| 坐标门通过区块 | 5/8 | 5/8 | 8/8 | 失败 |

响应门控把速度最大差降低 88.07%，说明预分支闭环响应确实是速度错位的结构性来源；但位置差
增加约 2.12%，块 1、3、5 仍超过 0.05 m。因此 `evidence_collection_gate_pass=false`、
`predictor_fit_authorized=false`。该单因素只支持“速度同步机制改善”，不支持共同前缀整体通过、
多人 predictor、多人规划收益或性能声明。

原冻结 assessor 使用混合 resource list 的间接索引查找 raw report，产生一个保留的完整性失败。
R1 只改为 receipt 同目录下的 `episode_report.yaml`，不改证据、统计量或门槛。R1 assessment SHA
为 `6a30f54abea88949722cf10e58e4da59aaa11a631f9eece2ee373a9f766ebaf5`，独立 review SHA 为
`c4469571a1a8f8e999e733138d42466fa0c0a6bd472bcebe5d044fe512a4765c`。科学阴性结论保持不变。

## 7.15 更新后的扩展结论

V3-16 增加了一个可以写入论文的机制结论：延迟 actor 闭环响应足以消除速度共同前缀失败，但
不足以消除坐标共同前缀失败。它把 V3-14 的复合失败分解为“响应动力学”和“位置状态同步”两个
层次，同时以 fresh 证据否定了“只加响应门控即可获得五动作支持”的假设。后续若研究多人支持，
必须建立新的位置状态快照/重置设计和独立 fresh release；不得修改 V3-16 的 0.05 m 门槛、追加
当前样本或用其拟合 predictor。
