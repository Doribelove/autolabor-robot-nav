# 论文 Claims–Evidence Matrix

日期：2026-08-06

状态定义：

- **qualified**：冻结证据足以支持带限定语的陈述；
- **not demonstrated**：预注册正式检验已完整完成，但对应阳性声明没有通过；
- **forbidden**：证据范围内不允许作为肯定结论。

| 编号 | 论文陈述 | 状态 | 直接证据 | 必须保留的限定语 |
|---|---|---|---|---|
| C01 | 系统使用有限、类型化 Anchor Bank，而非无界直接参数输出 | qualified | V2-04 软件、配置与组件测试 | Anchor Bank 是未校准模拟候选 |
| C02 | 因子化模式可组合 base 与 scene overlay | qualified | 模式管理器实现与 shadow 证据 | 不等于已找到最优模式 |
| C03 | feasible decoder 检查类型、范围与运动学耦合 | qualified | decoder 源码、合同和测试 | 不覆盖全部真实动力学 |
| C04 | 平滑以 previous-executed 而非 commanded 为基准 | qualified | manager/decoder 实现和回归测试 | 不声称性能优越 |
| C05 | circle-contact TTC 以相对圆接触为冲突事件 | qualified | TTC 实现、组件测试、I5/P1 语义证据 | 使用短时运动与圆足迹近似 |
| C06 | 参数事务记录 commanded/feasible/safe/executed 链 | qualified | transaction 源码、journal 和执行证据 | 真实车辆在线写入未授权 |
| C07 | I5 使用 fresh seeds 5161--5163 完成 6/6 集成单元 | qualified | I5 report SHA `8ed096601c13cc45fba34d32d5ae78477cabd345b9730df8ab4eced7fc0e5599` | 纯模拟、六单元集成验证 |
| C08 | I5 全部 expected/observed TTC 状态一致且可重放 | qualified | 6 journals、36 raw bindings、I6 review | 只支持语义/执行集成 |
| C09 | 历史失败、失败 release 和 I5 均被原样保留 | qualified | I5 freeze SHA `40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935` | 后续阶段没有改写历史身份 |
| C10 | P1 使用 seeds 5201--5290 完成 90 blocks / 270 episodes | qualified | P1 stage report SHA `1c625037365eea69b9363c61400e709195f9647211308b81100b395f96805853` | 三个冻结模板的 stochastic-seed 分布，不是 90 个几何场景 |
| C11 | P1 没有 retry、resume、replacement、forfeiture 或完整性失败 | qualified | 270 receipts、270 journals、stage report | 只针对 P1 唯一 release |
| C12 | P1 的 1,890 项 raw resources 可统一重放 | qualified | raw replay SHA `91a213e51d043938637be71faafc89169a61d61527bff22ab49c4c8ececc60c3` | 可复核性不等于阳性性能结论 |
| C13 | circle-contact 在 P1 中通过安全/任务非劣门 | qualified | four-gate assessment SHA `89147e95f19c461926e302982ab4db3772f1847fa0ca273e05b0df73fe2cd6df` | 限于冻结模拟分布与预注册 margin；不是安全优越 |
| C14 | semantic-clear overlay relative reduction 为 100% | qualified | P1 mechanism gate；control 0.176721、treatment 0 | 机制结论，不等于 whole-system 性能提升 |
| C15 | semantic-clear 受限完成时间估计下降 3.762% | qualified | P1 efficiency estimate；lower 2.790%，adjusted p≈`1e-5` | 必须同时写明未达到预注册 5% 最小效应量 |
| C16 | P1 证明了性能提升 | **not demonstrated** | efficiency gate fail；`claim_authorized=false` | 四门合取失败，不得改写为阳性 |
| C17 | 性能改善至少 5% | **not demonstrated** | point estimate 3.762% < 5% | 显著性不能替代 practical-effect threshold |
| C18 | circle-contact 比 legacy 更安全 | forbidden | 非劣通过不证明优越 | 只能写“在 margin 下非劣” |
| C19 | fixed TEB 是 winner 或最佳配置 | forbidden | fixed TEB 仅 contextual | 未预注册 winner ranking |
| C20 | 已证明跨未见 family、geometry 或 timing 泛化 | forbidden | 无独立 held-out 分布研究 | P1 只覆盖三个冻结模板 seed 分布 |
| C21 | 已证明真实车辆性能提升或安全 | forbidden | 无实车数据 | 没有现场执行授权或验证 |
| C22 | 已证明形式化安全或无碰撞保证 | forbidden | 无形式化模型/证明 | 当前仅有约束和经验非劣评估 |
| C23 | 已校准可部署阈值或 `runtime_ready=true` | forbidden | P1 assessment `runtime_ready=false` | 不得升级部署状态 |
| C24 | 新增 SAC 训练带来提升 | forbidden | P1 training steps=0 | P1 不检验 learning gain |
| C25 | post-P1 搜索严格限制为三轮/2200 仿真单元 | qualified | P4-F1 budget accounting SHA `3d1189c7f6f779e4ddd0dae8eaa5f9c373734c49038745d692ec2e8bc7f567cc` | 4 consumed + 2196 retired；retired 不等于成功 episode |
| C26 | 五个 promoted Bank 通过严格离线 parser | qualified | P4-Q1 review SHA `26c03a7e31a06d1f45ccc7348cdd980974b2a00c2b9b7f8e8879af38f0d4b3fd` | 行为参数未改；没有 live runtime、安全或性能资格 |
| C27 | post-P1 搜索证明相对最强公平固定 TEB 明显提升 | **not demonstrated** | 无完整 calibration/test；P4-F1 terminal conclusion | P1 3.7624% / 2.7904% 也低于新 10% / 5% 门 |
| C28 | P4 高速候选是 winner | forbidden | P4-I1 在 Gazebo 启动前失败 | 没有 candidate/baseline selection 或 held-out test |
| C29 | 修复 ROS 环境后可以复用 seed5924 重跑 | forbidden | P4-I1 consumption journal；final budget closure | 身份已消耗，剩余 703 已退役，性能搜索终止 |
| C30 | V3-09-F1 使用 fresh seeds 6801--6830 完成 30 blocks / 180 episodes | qualified | assessment SHA `d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b` | 参数化反应式单 actor crossing family |
| C31 | V3 spatial-safe 相对 fixed-v1.20 shielded comparator 的配对时间改善为 22.3844% | qualified | V3-09-F1 assessment；one-sided lower 21.8235% | comparator 是冻结直线策略，不是 actual TEB |
| C32 | V3-09-F1 的点效应、lower、positive-block、任务、安全、重放和延迟门全部通过 | qualified | independent review SHA `206732c86b536e7f9a520c58dac6d386ff6752595bdc767d4957e4804e5f8d5a` | 只在冻结 formal population 内成立 |
| C33 | V3 与 comparator 的正式目标成功率均为 100% | qualified | 90/90 vs 90/90 receipts 和 assessment | 不等于复杂导航或实车成功率 |
| C34 | V3 在三种 actor policy 上都更快 | forbidden | assertive：11.5833 s vs 10.8917 s | 阳性结果是三 policy 等权 block 聚合；assertive stratum 为负向 |
| C35 | V3-09-F1 已证明优于实际 TEB | forbidden | V3-09 formal claim `actual_teb_superiority=false` | fixed-v1.20 shielded comparator 不是 TEB；不得用后续研究改写其历史设计 |
| C36 | V3 的 RL/学习排序器带来性能提升 | forbidden | V3-04 training steps=0，learning gate closed | 正式方法是解析空间安全规划器 |
| C37 | V3 已证明通用导航安全、跨场景泛化或可部署 | forbidden | claims YAML 与 independent closure review | 无多场景、多人、分布外、形式证明或硬件验证 |
| C38 | V3-10-P1 形成可复核论文数据闭环 | qualified | manifest SHA `70353b355c6eec2f733901ca7d4a6820de550beff00999cfacdb359dec951938`；review SHA `bee5891b08f860a97a25ea71047b3e5fabb0c193695f221de131eacb5dcfe0f4` | 纯离线导出，不新增正式证据 |
| C39 | V3-11-F1 使用 fresh seeds 7101--7130 完成 30 paired blocks / 60 episodes | qualified | execution summary SHA `9843ce28763e1f836b3230545b00b614019486ab89564d7063d247e1cb1fc2a4` | 三个冻结 single-reactive-actor Gazebo 模板、cautious/hesitant policy |
| C40 | V3 spatial-safe 相对 actual TebLocalPlannerROS 0.8.4 的配对均值时间改善为 12.9288% | qualified | assessment SHA `9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`；one-sided 97.5% lower 12.7253% | 只针对冻结 formal population 和固定 TEB 版本/配置 |
| C41 | V3-11 双方目标成功均为 100%、contact 为 0，六个 scene×policy strata 均为正 | qualified | independent review SHA `2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39` | 双方共享 oracle actor track；actual TEB 最小经验保守净空 0.0192 m |
| C42 | V3-11 证明 perception、multi-actor/OOD 或通用安全优越 | forbidden | claims YAML 与 formal scope | 未测试感知退化、多人、OOD、独立复杂地图或形式化安全 |
| C43 | V3-11 证明所有 TEB 版本/配置上都优越 | forbidden | 只冻结 TebLocalPlannerROS 0.8.4 与 fixed_teb.yaml | 合法表述必须带版本、配置和场景总体 |
| C44 | V3-12-P1 形成 actual-TEB 可复核论文数据闭环 | qualified | manifest SHA `319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`；review SHA `96d7d0333ef2eba5b62819fb0fadc689c3756e5015804d2ea8fb86b4a4ca17b2` | 纯离线导出，不新增正式证据 |
| C45 | V3-14-S1 完成 5 单元预检和 40 单元 fresh 多动作证据矩阵 | qualified | assessment SHA `58c472560699e10276f76c063dd243ab5666f8e534346c51e88aa9f2b49958b9` | 45/45 attempt 1 完成；预检不进入科学计数 |
| C46 | V3-14-S1 的调度、launch 与 runtime candidate identity 全部一致 | qualified | action-binding receipts；review SHA `4773d76437a01148984770ca502dce96cfa6d3bc2743029e0bb7a1f10f94c60a` | 只证明执行身份完整性，不证明反事实配对或预测有效性 |
| C47 | V3-14-S1 的五动作多演员支持门通过 | **not demonstrated** | actor 坐标差 0.066074 m、速度差 0.072081 m/s，均超过 0.05 门 | 完整执行不能替代冻结合取门 |
| C48 | 当前多演员数据可用于拟合 r3 predictor | forbidden | `predictor_fit_authorized=false` | 共同前缀 actor 状态在候选间不可比 |
| C49 | V3 已证明相对 actual TEB 的 multi-actor 或 perception/OOD 收益 | forbidden | V3-14 后续阶段未授权、未执行 | V3-11 仍只支持 single-reactive-actor oracle-track 总体 |
| C50 | V3-14 接触了硬件或实车 | forbidden | terminal result 与独立 review 均记录 hardware/real vehicle false | 本阶段全程纯 Gazebo/离线审计 |
| C51 | V3-16-D1 将 V3-14 失败归因为候选分支前的 actor 闭环响应放大 | qualified | attribution SHA `1f84987b14a9a8721eea5823e965d7c0283f0a4250f7efb7137cf060b08e7396` | 是冻结证据上的机制归因，不是一般 actor 模型结论 |
| C52 | V3-16-S1 完成 fresh 5 回合预检和 40 回合五动作支持矩阵 | qualified | R1 assessment SHA `6a30f54abea88949722cf10e58e4da59aaa11a631f9eece2ee373a9f766ebaf5` | 45/45 attempt 1；预检不进入科学计数 |
| C53 | 12.55 s 响应门控显著改善了 actor 速度共同前缀 | qualified mechanism | 最大速度差从 0.072081 降至 0.008597 m/s，下降 88.07% | 单因素机制观察；不等于位置同步、整体支持或性能通过 |
| C54 | V3-16-S1 通过多人五动作支持门 | **not demonstrated** | 最大坐标差 0.067472 m > 0.05 m；仅 5/8 区块通过坐标门 | 速度门通过不能抵消坐标合取门失败 |
| C55 | V3-16 数据可用于拟合 r3 predictor 或启动 actual-TEB 多人比较 | forbidden | terminal `predictor_fit_authorized=false`；R1 review SHA `c4469571a1a8f8e999e733138d42466fa0c0a6bd472bcebe5d044fe512a4765c` | 新位置同步设计必须另立合同、seed、预算与授权 |
| C56 | V3-16 使用了硬件、实车或在线 TEB 参数写入 | forbidden | terminal/R1 review 均记录 hardware/real vehicle false | 全程 Gazebo 与离线复算 |
| C57 | V3-18 建立了覆盖历史主要研究的统一 canonical episode dataset | qualified | R1 dataset SHA `f93b426b2de0b85ce8d03b8e7c7e42ade53c8371d3fe3f31dc68ab7bbdbbd1fe`；825 rows | 是历史证据适配，不新增性能样本或改变原统计单位 |
| C58 | V3-21 使用 fresh seeds 8201--8230 完成四方法 30 blocks / 120 episodes | qualified characterization | assessment SHA `c195fc7cea4c649582bb3c0017e55a1b98f5de3b7f971737b1f2abcb5596ccc8` | 一个 gate 容差偏差，`formal_result=false`；所有失败按 22 s 保留 |
| C59 | V3-21 证明 V3 相对 Actual TEB 具有时间优势 | **not demonstrated** | 点降幅 -0.193%，双侧 95% CI [-19.867%, 12.982%]，Holm p=0.755412 | 新鲜失效计入式研究未复现历史 V3-11 阳性；两研究不得合并 |
| C60 | V3-21 中 V3 相对去候选条件化消融缩短 49.454% | qualified exploratory mechanism | 双侧 95% CI [37.001%, 58.800%]；成功率 93.33%/43.33% | 冻结单 actor/oracle-track 总体；不等于通用 TEB 优越或安全证明 |
| C61 | Fixed-v1.20 失败说明所有固定规划器都无效 | forbidden | 本研究只有一个冻结直行屏蔽策略，0/30 goals | 只能说明该具体比较策略过于保守 |
| C62 | V3-21 的零 contact 和较大净空证明形式化安全 | forbidden | 四方法 contact=0；V3/TEB 最小经验净空 0.4764/0.0395 m | 有限仿真描述量，不是感知、实车或形式化保证 |
| C63 | V3-21 可被写成无偏差正式阳性研究 | forbidden | sequence107 gate 误差超限 2.286 mm；`formal_result=false` | 可报告完整 characterization 及排除敏感性，但不得升级标签 |
| C64 | V3-22 自动生成论文图、三线表和结果叙述且通过独立复核 | qualified | materials manifest SHA `658e906c3fae23d86e5cab5e65edbe6ca00cc74c1188b0404294f29c4fe973d0`；review SHA `88547275a6627027517aa55d2128469bb30de1721fc162e92ab9d6d787be0959` | 纯离线派生材料，不新增科学证据 |

## 正文审查规则

1. 所有 qualified 陈述必须同时保留该行限定语。
2. C15 不得省略“未达到 5%”后单独出现；其合法作用是报告估计量，不是升级 C16。
3. not demonstrated 是正式阴性结论，不得改回 pending，也不得用重跑、追加样本或修改阈值升级。
4. forbidden 陈述不得在摘要、结论、图题或宣传性文字中作为肯定结论出现。
5. `formal_result=true` 只表示 P1 正式研究完成；不得等同于 `performance_improvement_demonstrated=true`。
6. I5 的独立 block 数是 3；P1 的独立 block 数是 90。高频 TTC/overlay 样本不扩张统计 $n$。
7. fixed TEB 的描述性均值不进入 circle-contact 与 legacy 的确认性推断。
8. P4-Q1 的 strict parser pass 只说明 schema 可解析，不得写成 live transaction、安全或性能通过。
9. post-P1 的完整性失败、未运行单元和 retired budget 不能从 intention-to-treat 记录中删除。
10. V3 的阳性结果不得用于改写 V2-P1、post-P1 或 V3-05 的历史阴性结论。
11. C31 每次出现都必须同时说明 comparator 不是 actual TEB，且范围仅为参数化反应式
    single-actor crossing。
12. V3 的 aggregate positive result 不得删除 assertive stratum 的负向结果。
13. C40 每次出现都必须同时说明 actual TEB 的版本/配置、三个冻结模板、两种 actor policy
    和共享 oracle actor track。
14. “零 contact”必须与 actual TEB 最小经验保守净空 0.0192 m 同时解释，不能写成形式化或
    通用无碰撞保证。
15. V3-14 的 45/45 完整执行和动作身份通过必须与 actor 坐标/速度配对门失败同时出现；不得
    把执行完整性写成 multi-actor 科学资格。
16. V3-16 的“速度差下降 88.07%”每次出现都必须同时写明坐标差 0.067472 m 仍失败、整体
    `evidence_collection_gate_pass=false`；不得写成多人支持、性能或预测器资格通过。
17. V3-16 原 assessor/review 的完整性失败与 R1 修复都必须保留；R1 只修复 raw report 路径
    索引，不改变 45 个回合、门槛或科学阴性结论。
18. 每次引用 V3-11 的 12.9288% 历史阳性时，若讨论总体稳定性，必须同时披露 V3-21 的
    -0.193% 新鲜失效计入式结果；两个阶段不得池化、挑 seed 或删除 timeout。
19. C60 必须标为探索性消融机制证据；不能用其替代 C59 的 Actual TEB 主要比较。
20. V3-21 的 sequence107 偏差和排除 block27 后结论不变必须一起报告，且
    `characterization_result_available=true` 不得写成 `formal_result=true`。

## 性能声明终态

P1 已耗尽唯一 270-unit budget。四门结果为：

```text
integrity=true
safety_and_task_noninferiority=true
efficiency_superiority=false
mechanism_reduction=true
performance_improvement_demonstrated=false
claim_authorized=false
```

论文的最终合法概括是：

> 在预注册的三个冻结动态模板 fresh-seed 仿真分布上，circle-contact 语义通过了安全/任务非劣与机制下降门，并使 semantic-clear 配对的受限完成时间估计下降 3.762%；然而该点估计未达到预注册的 5% 最小效应量，因此本研究未证明性能提升。

这是一项完整而阴性的正式结果；不再存在通过当前 P1 release 升级性能声明的路径。

post-P1 三轮有限搜索也已终止：

```text
global_capacity=2200
consumed=4
retired=2196
remaining=0
new_held_out_test_accessed=false
post_P1_integrity_gate=false
obvious_performance_improvement_demonstrated=false
candidate_winner_frozen=false
```

因此最终合法结论升级为更严格但仍为阴性的一句话：“全局有限预算耗尽，未证明 FAM-TEB
相对最强公平固定 TEB 基线有明显性能提升。”这里“升级”只指证据闭环覆盖了后续搜索，
不代表把 P1 的阴性结果改成阳性。

V3-SCOPE 是独立于该 V2 搜索的新架构与新证据链。历史 V3-09 的合法概括是：

> 在 30 个 fresh 参数化反应式单 actor crossing blocks 上，V3 spatial-safe 相对冻结的
> fixed-v1.20 shielded 直线策略使受限目标时间配对下降 22.3844%，单侧 95% lower 为
> 21.8235%，双方目标成功均为 100%，全部审计门通过。

该句必须紧接以下边界：比较对象不是 actual TEB；结果不证明多场景泛化、学习收益、
形式化安全、部署或实车性能。

最新 V3-11 actual-TEB 研究的合法终态概括是：

> 在来自三个冻结 single-reactive-actor Gazebo 模板与 cautious/hesitant policy 的 30 个
> fresh paired blocks 上，V3 spatial-safe 相对 actual TebLocalPlannerROS 0.8.4 将受限
> gate-to-goal 时间配对下降 12.9288%，单侧 97.5% lower 为 12.7253%；双方目标成功均为
> 100%，contact 为 0，六个 scene×policy strata 均为正。

该句必须紧接以下边界：双方共享 oracle actor track；actual TEB 最小经验保守净空为
0.0192 m；结果不证明 perception、multi-actor/OOD、所有 TEB 配置、形式化安全、学习收益、
部署或实车性能。
