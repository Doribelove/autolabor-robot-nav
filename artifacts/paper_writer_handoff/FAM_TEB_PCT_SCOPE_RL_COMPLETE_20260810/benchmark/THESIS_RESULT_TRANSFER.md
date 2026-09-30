# PCT-SCOPE-RL、Actual TEB 与 DWA Gazebo 结果回填

## 可直接回填的实验环境

实验在 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 的独立 thesis 工作区完成。机器人为 Autolabor M2 Ackermann 模型，所有方法共享 1.20 m/s 最高速度、0.60 m/s² 加速度、0.90 m/s² 减速度、相同底盘/执行器动态、15 Hz 360° LaserScan、同一 warmup gate、目标、22 s 时限和 `/cmd_vel_navigation` 控制边界。研究仅为仿真，不使用真实车辆。

## 方法与公平性配置

- PCT-SCOPE-RL：冻结 PPO actor SHA `deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8`；8 个 Ackermann 可行候选，R2 唯一修改是左右低速空间候选由 0.4 调为 0.6 m/s。0.20 m 备份可行余量、0.33 m 膨胀、0.35 s 响应延迟与 1.50 s 停车后验证均不变。
- 解析先验：使用与 PCT 相同的候选生成、候选条件预测和硬验证，只以冻结解析排序选择。
- Actual TEB：`teb_local_planner/TebLocalPlannerROS` 0.8.4，开发集最强 v0 配置 SHA `ce9fd94f306a86fe821f9c4732cb0894b8974cf39e45ecf6ab573f72b7e04606`。
- DWA：开发集最强 v0 配置 SHA `efb8c37d540927f4bee2b97435c141d07751d8c623b8db6f5c7d44d990967921`。
- 公平性：传统规划器获得独立且充分的 development 调参预算；最终选择按六场景失败计入均时，未降低其 1.20 m/s 上限。四方法不使用 Gazebo actor 真值做规划，actor state 只供事后评价。所有超时、未到达、接触、负保守净空和系统失败按 22 s 进入主指标，无删除、补跑或替换。

## 研究设计与样本量

R1 在 pilot 功效分析后使用 160 个 fresh paired blocks（640 episodes，seeds197001–197160）；R2 以互斥 development 单因素消融和新 pilot 为基础，按冻结功效规则使用 200 个 fresh paired blocks（800 episodes，seeds198001–198200）。每块含 PCT、解析先验、Actual TEB、DWA，顺序平衡。两项主比较分别使用 200,000 次配对 block bootstrap 和双侧 97.5% 区间；阳性需点改善≥8%、区间下界>0、完成率差≥-5pp、PCT contact 不更高且 mask violation=0。

## 主结果

| 方法 | R2 有效到达 | 完成率 | 失败计入均时 (s) | 最小保守净空 (m) | contact | mask violation |
|---|---:|---:|---:|---:|---:|---:|
| Actual TEB | 119/200 | 59.5% | 14.4470 | -1.1259 | 0 | 0 |
| 解析先验 | 150/200 | 75.0% | 16.3988 | 0.1645 | 0 | 0 |
| DWA | 100/200 | 50.0% | 16.0925 | -0.3702 | 0 | 0 |
| PCT-SCOPE-RL | 180/200 | 90.0% | 14.8413 | -0.2696 | 0 | 0 |

| 正式研究 | 比较 | 相对降低 | 双侧 97.5% CI | 完成率差 | 全部门通过 |
|---|---|---:|---:|---:|---:|
| R1 | PCT vs TEB | -0.94% | [-8.36%, 5.64%] | 40.0 pp | false |
| R1 | PCT vs DWA | 6.37% | [1.09%, 11.14%] | 45.6 pp | false |
| R2 | PCT vs TEB | -2.73% | [-9.81%, 3.59%] | 30.5 pp | false |
| R2 | PCT vs DWA | 7.78% | [2.55%, 12.61%] | 40.0 pp | false |

R2 阳性比较：无。R1 是按原候选支持的合格阴性结果，不与 R2 合并；R2 使用全新 seeds 独立验证开发阶段预注册的单因素修复。

## 消融与机制诊断

development 配对消融的 R1 control/0.6 m/s treatment 均时为 15.5917/14.3917 s；静态改善 5.6250 s，动态均值变化 -0.4187 s（负值为改善）。候选覆盖修复减少平均 certified-stop 数 18.00→13.42，同时 treatment contact=0、mask violation=0。首个 control 在语义前发生 catkin 导入失败，按 22 s 保留且不重试；删除整对 B001 的敏感性方向一致。

## 失败案例与限制

- PCT 保留 18 次超时/未到达与 2 次保守净空违例；后者使最小保守净空为 -0.2696 m，因而不支持形式安全声明。PCT contact=0 且 mask violation=0。
- 解析先验保留 29 次超时/未到达与 21 次执行完整性失败；这些系统失败均按 22 s 进入主指标。
- Actual TEB 保留 81 次、DWA 保留 100 次“到达但负保守净空”失败；没有删除、补跑或替换。
- R2 候选覆盖缩短静态绕障和部分恢复，但未消除 PCT 在连续交互中的安全等待/超时。
- 结论总体仅包含六个冻结 Gazebo 模板与随机种子，不是独立几何、感知噪声/OOD、多机器人、形式安全、部署或真实车辆证据。
- Actual TEB 的比较在 pilot 推算中即使 200 blocks 仍略低于 80% 目标功效；阴性不得解释为等效。

## 阴性结果归因与下一步方案

封存后的分层归因显示，PCT 相对 DWA 的总体收益来自 partially-occluded（-8.3061 s）和 static-avoidance（-7.0338 s），负值为 PCT 更快；但在 head-on-yield 与 single-crossing 分别慢 5.2666 s 和 4.6618 s。head-on-yield 平均有 24.97 次 certified stop；single-crossing 平均有 14.00 次候选切换和 15.53 次 stop。sequential-interaction 中则有 11 次超时，平均 23.24 次切换、38.67 次 stop 和 9.00 次恢复。这是与保守停车/切换抖动一致的机制证据，不是独立随机化的因果证明。

可证伪假设是：在不降低任何硬净空、contact 或 mask 门限的前提下，仅修改“安全候选间的切换滞回/停车恢复释放”，应能在新的 development head-on、single-crossing 和 sequential 集上同时降低 stop、switch 与失败计入时间，而不恶化最小净空。最小实验应只比较一个冻结滞回/恢复因子，事先要求动态均时、stop 和 switch 三者改善，且 contact=0、mask=0、净空不低于 control。解析先验的 21 次 integrity failure 由 LaserScan 时间戳 TF 合同失败触发，应作为独立的非行为性 TF-buffer/warmup 健壮性因子在 development 测试。任何未来实验都不得使用 seeds 197001–197160 或 198001–198200 调参；需重新冻结、功效分析、预注册并使用全新 formal seeds。

## 图表与机器可读表

- [图1：两轮正式效应与 97.5% CI](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_effects_97_5_ci.svg)
- [图2：R2 方法均时与完成率](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_method_outcomes.svg)
- [图3：R2 六场景分层](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_scene_strata.svg)
- [图4：R2 development 单因素消融](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/r2_development_ablation.svg)
- [两轮方法汇总 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_round_summary.csv)
- [正式效应 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_effect_summary.csv)
- [R2 场景分层 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_scene_strata.csv)
- [R2 失败与机制诊断 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_failure_diagnostics.csv)

## 推荐论文结论措辞

两轮独立正式研究均未按预注册门限建立 PCT-SCOPE-RL 优于 DWA。 相对 Actual TEB 的改善为 -2.73%（97.5% CI [-9.81%, 3.59%），未建立优于 Actual TEB。 该结论仅适用于冻结的单机器人 Gazebo 场景总体；不应扩展为一般 TEB 优势、形式安全、感知/OOD 泛化、部署或真实车辆结论。

## 核心证据散列

- R1 assessment: `777ce9b1599713ebbdc62cb14691334e733c5f08e12e8cde881d2eeb07a0a04c`
- R1 integrity review: `f2fb67d34998c17adba1c174608a37613696187cdd882f758d9dbc641cec9422`
- R2 release: `c64dee01acbdf58ece4b9eb2056d039c6d2ee61121dbbbcffbc717c8c5253137`
- R2 assessment: `a0cf0ef415a0685411299524bcb9a71674edd59f58c378b037d8b91d64e6a473`
- R2 integrity review: `a937f5272756bb2b1a10526046d03ff9cbf0dd6f8d41be3658e7b8419b7a6265`
