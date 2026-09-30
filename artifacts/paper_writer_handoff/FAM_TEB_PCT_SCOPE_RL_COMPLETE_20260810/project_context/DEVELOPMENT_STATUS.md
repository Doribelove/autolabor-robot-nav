# TEB RL Thesis Development Status

更新时间：2026-08-08

## 当前阶段

- 阶段：V6-06-P1-R1 当前论文更新与跨电脑移交闭环
- 状态：V5 传感器端到端候选 RL 完成但正式实效门阴性；V6 以单因素 privileged counterfactual trajectory-rank distillation 完成 4×120k 训练、160 块验证和 80 块 fresh formal，并取得相对解析先验 8.4024% 的限定阳性结果。三条 Gazebo 迁移预检均因保守净空门失败而终止，资格/正式矩阵未启动。最新纯离线交付为 41 页真正整合的 V3+V6 技术稿、自包含 V3/V5/V6 数据、11 图/5 表、实车空白章节和另一台电脑 Codex 学校模板更新提示词；独立审查全部通过。禁止扩大为 Gazebo、Actual TEB、通用安全、感知/OOD、部署或实车结论
- 当前负责人：Ubuntu Codex CLI + 用户

## V5/V6 端到端候选强化学习终态

- 当前入口：`docs/thesis_experiment/CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md`
- 架构：LaserScan 历史 + odometry + local goal -> CNN-GRU sensor actor ->
  8 个 Ackermann 可行候选的残差评分；解析先验、硬动作掩码、0.10 优势门和 certified-stop
  保留独立否决/回退权限；输出仍经过 `/cmd_vel_navigation`
- 特权训练：critic-only 32 维仿真真值 + PCT 困难场景反事实候选排序标签；部署 actor-only
  checkpoint 不含 privilege key/module
- V5 正式结果：60 fresh blocks，降幅 6.0936%，95% CI [1.7867%,11.0069%]，低于 8% 门，终局阴性
- V6 正式结果：80 fresh blocks，PCT/解析目标 60/80 对 49/80，均时 13.1156/14.3188 s；
  降幅 8.4024%，双侧 97.5% CI [3.4840%,13.7135%]，全部冻结门通过；接触/掩码违规均 0
- 次要机制结果：相对冻结 V5 降幅 3.1611%，95% CI [0.4936%,6.5765%]；收益集中于部分遮挡；
  narrow-corridor 四方法均 0/20；V6/no-history 完全相同
- Gazebo 边界：R1/R2/V6-04 三次预检 V6 最小保守净空 -0.0774/-0.3366/-0.9902 m；
  均未授权 qualification/formal，不得声称相对 Actual TEB 有效
- 论文：`artifacts/v6/paper/v6_05_p1_thesis_export/FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf`，
  69 页 A4，SHA `c1a23074f1524cf967b89f674017f03529e4b54ab171abc5cabfa5749229d829`；
  independent review SHA `ff4b6423d8fb610fe29486ab059c86a1e86f284055c8b520cf9e38cf4e470b41`
- 当前论文移交包：`artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/`；整合 PDF 41 页 A4，
  SHA `a2fcacc82ee8f7fc290415661b34bd5324bbc157158d54a0780705a7737eadf1`；学校模板 Codex 提示词、
  combined claims、数据字典、V3/V5/V6 CSV、实车空白章节和 77 文件终态清单齐全；independent
  review SHA `00e4705f783632723018c6920f8e1b68dc6096a08cc28018d72e286e783d1f1b`
- 当前安全入口：停止训练和 Gazebo 参数追逐；若开展后续工作，必须新建“闭环交互一致的
  传感器预测/可达占用”阶段并使用新种子、新合同与独立授权；实车和在线 TEB 写入继续禁止

## V3-SCOPE 终态

- 快速入口：`docs/thesis_experiment/CURRENT_V3_11_ACTUAL_TEB_FORMAL_HANDOFF.md`
- 历史 foundation：`docs/thesis_experiment/CURRENT_V3_SCOPE_FOUNDATION_HANDOFF.md`
- 完整理论报告：
  `docs/future_research/v3_candidate/THEORETICAL_FEASIBILITY_AND_RESEARCH_ROADMAP.md`
- 架构代号：`V3-SCOPE`
- 核心动作：从已满足 Ackermann 约束的候选轨迹中选择，不再以高维 TEB 参数作为主学习动作
- 核心链路：候选条件交互预测 -> 校准可达占据管 -> 硬安全过滤 -> 解析排序 ->
  有条件启用 contextual bandit/offline RL
- 硬件策略：保留 M2、激光、GPS、里程计、相机和稳定命令安全链；未来输出仍须经过
  `/cmd_vel_navigation`、GPS limiter、FOD arbiter 和 M2 driver
- 理论状态：候选条件空间规划、反应式 actor、预测、硬验证与解析排序已实现并形成正式仿真证据
- 当前状态：`V3-11-F1 ACTUAL-TEB FORMAL POSITIVE / V3-12-P1 PAPER CLOSURE / NO TRAINING / NO REAL VEHICLE`
- V3-00 证据：`v3_scope_planner` 5 消息编译通过，7/7 pytest 与 7/7 catkin nosetests 通过；
  foundation evidence freeze SHA `4a08325b67c0706a860560ab7ced1191100e5ecb91e4b6138679aa0c11a05d5b`，
  independent review SHA `bb9ce5641ed650f2f3b14688f15f7a3787679f7e06c9bc4d62cd435be7b6b1bf`
- V3-01 证据：R2 36/36 attempt=1，9/9 policy×seed blocks 通过；actor 4 s 位置效应
  0.777245--2.310975 m，最大初态误差 0.006 m，最大跨 seed spread 0 m；independent
  result review SHA `f9363de30f018208d3e2adf0c9cdbe6b19e4c9b2708ec3eba356f03bca2d0ffd`
- V3-02 证据：C2 fresh seeds 6261--6263、36/36 attempt=1；coverage 0.9931、
  mean width 3.9098 m、MAE 0.1918 m、共同前缀 actor coordinate/speed spread 均为 0；
  C2-I1 integrity review SHA `56ef648a291a9078ad2dce947b3f308e690d09b6ac325b4554afdbeafc0fabb4`
- V3-07/08 证据：空间 predictor held-out 20/20，global/per-head coverage 1.0；A3 fresh
  integration 18/18，minimum conservative clearance 0.41937 m，exact replay 1.0
- V3-09-F1 证据：180/180、goal success 1.0/1.0、point 0.223844、one-sided lower
  0.218235、positive blocks 30/30、minimum conservative clearance 0.165663 m、
  exact replay 1.0、P99 18.087844 ms；assessment SHA
  `d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b`
- V3-10-P1 证据：paper manifest SHA
  `70353b355c6eec2f733901ca7d4a6820de550beff00999cfacdb359dec951938`，
  independent closure SHA
  `bee5891b08f860a97a25ea71047b3e5fabb0c193695f221de131eacb5dcfe0f4`
- V3-11-F1 actual-TEB 证据：60/60、goal success 1.0/1.0、mean time
  6.7683/7.7733 s、point 12.9288%、one-sided 97.5% lower 12.7253%、
  六个 scene×policy strata 全正、contact 0；assessment SHA
  `9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9`，
  independent review SHA
  `2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39`
- V3-12-P1 actual-TEB paper manifest SHA
  `319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b`，
  independent closure SHA
  `96d7d0333ef2eba5b62819fb0fadc689c3756e5015804d2ea8fb86b4a4ca17b2`
- 下一安全入口：当前 actual-TEB bounded study terminal；更强论文声明需新建
  纯离线 execution-integrity closure，先加入 scheduled candidate ID 与 runtime report ID
  的 fail-closed 一致性断言。V3-13 r3 完整支持、感知/定位退化和训练均未获授权；
  实车仍需现场授权与独立标定

## V2 架构设计状态

- 系统指南：`docs/thesis_experiment/V2_SYSTEM_GUIDE.md`
- 架构代号：`FAM-TEB V2`
- 设计状态：`DESIGN COMPLETE`
- 实现状态：`R6-I5 6/6 INTEGRATION PASS FROZEN / P1-E1 270/270 TERMINAL / P2-P4 FINITE SEARCH TERMINAL / P4-F1 OFFLINE CLOSURE PASS`
- 正式实验状态：`P1 FORMAL NEGATIVE COMPLETE / GLOBAL 2200-UNIT SEARCH CLOSED / obvious_performance_improvement_not_demonstrated / NO WINNER`
- 实车状态：`FORBIDDEN WITHOUT NEW CALIBRATION AND ON-SITE APPROVAL`
- 与 V1 的关系：V1/T00--T12 合同、runner、配置和 artifacts 保持冻结；V2 必须使用独立的
  schema、配置、manifest 和 artifact 根目录。
- 已完成入口：V2-00 基线隔离、V2-01 机器合同/消息骨架和两项基础修复、V2-02
  可信仿真动力学/五类场景/统一 evaluator、V2-03 局部世界模型/动态跟踪预测/健康/无标签
  规则监督器、V2-04 Anchor Bank/类型化 profile/feasible decoder/平滑 shadow 事务和零训练
  规则闭环；V2-04B 已增加 simulation-only 20 参数 typed TEB 真实事务和 calibration split，
  最新验收见 `artifacts/v2/component_acceptance/v2_04b_acceptance.yaml`。
- 最新入口：`V2-04G-P4-F1` 已完成全局有限搜索终态闭环。三轮容量
  `792+704+704=2200`，实际 `4 consumed + 2196 retired`，剩余仿真/训练权限均为 0。
  最终 P4-I1 release SHA
  `1bcd0392d475cdf17d7a2bf6540e024a61136150011945d323cae1234b568220`
  只运行 fresh seed5924 attempt1；执行环境未解析到 `m2_gazebo`，故在 Gazebo、move_base、
  参数事务和 raw trace 前终止。五个 promoted Bank 只通过了严格离线 parser，未获得 live
  性能或安全证据。终态 conclusion SHA
  `8ce6736dd7ea1f13d6c272f0420f4588d1e0c9e25949b7b1eabf5bd028b3eeb7`，
  independent review SHA
  `e0dc93790e092b99684c64a9d2b11e5ce02a0a16f5235649e3657376835270c8`。
  详见 `docs/thesis_experiment/CURRENT_V2_04G_P4_F1_TERMINAL_HANDOFF.md`。性能搜索停止；
  所有模式和参数阈值继续保持 `runtime_ready=false`。

## 已确认版本

| 项目 | 版本/commit | 备注 |
| --- | --- | --- |
| 主仓 | 见 `git log -1` | 当前本地分支 `base_rl`；本轮整理 R5--I6 的代码、合同、证据和论文草稿并发布到该分支 |
| arena-rosnav-3D | `634bcb091a90b362087cdba5a9cd3856466d493c` | dirty（70 项） |
| TEB fork | `b4cf0639775e4521cdf7681158043ad3eef4b01a`；package `0.8.4` | 本地 fork含静态 footprint 生命周期修复，dirty；优先于系统 TEB 0.9.1 |
| ROS | Noetic `1.17.4` | catkin 0.8.12，move_base 1.17.3 |
| Gazebo | `11.15.1` | gazebo_ros 2.9.3 |
| Python | `3.8.10` | 严格论文 `.venv`；`PYTHONNOUSERSITE=1` |
| Stable-Baselines3 | 严格 `.venv` `2.4.1` | bundled `1.1.0a1` 与外部 `catkin_ws` 均非 T09/T10 runtime |
| PyTorch | 严格 `.venv` `2.4.1+cpu` | CUDA false；Gymnasium `1.0.0`；RECORD hashes 已冻结 |

## 阶段状态

| 任务 | 状态 | 证据/命令 | 未解决问题 |
| --- | --- | --- | --- |
| T00 环境盘点 | DONE | 隔离无运动 TEB 探测；Git/版本/接口事实已并入 `CURRENT_TEB_RL_HANDOFF.md` | 旧 T00 prompt 和 inventory 已按用户要求删除；实车标定项保持 TBD |
| T01 包骨架 | DONE | `teb_rl_tuner`、`thesis_experiment`；9 个 pytest；配置 CLI；隔离环境检查 | 当时待完成的 RL freeze gate 已由 T09 严格 `.venv` 锁定与校验闭环 |
| T02 Gazebo M2 | DONE | `m2_gazebo`；9 项单元/静态测试；11/11 底盘定量回归；5/5 固定 TEB 导航回归；两份机器可读 YAML | 所有结构与动力学值仍为 `simulation_candidate`，运动学零制动距离不得用于实车结论 |
| T03 TEB 参数客户端 | DONE | 四重仿真门控；9 参数描述/类型/范围/当前值；原子 Reconfigure；ack/readback/延迟；快照恢复；20 项 catkin tests；`artifacts/t03/teb_parameter_client_acceptance.yaml` | cfg min/max 仅为接口范围；物理/实车安全范围与变化率仍为 TBD |
| T04 状态/奖励/episode | DONE | ROS 时间单调/同步；K=4；`t_ack -> t_active`；物理时间奖励积分；唯一 termination；activation timeout 丢弃 transition；多 episode Gazebo 验收 | 当前状态归一化与 reward 权重仍是 simulation contract，正式训练前需冻结 |
| T05 投影/安全/回退 | DONE | 精确 9 参数；NaN/Inf/类型拒绝；bounds/rate/coupling；四态+滞回；完整原子回退；Gazebo normal + emergency dry probe | Gazebo pilot 不等于安全标定；实车 bounds、制动/时延和保守参数仍待实车标定，当前配置禁止实车 |
| T06 日志/导出 | DONE | 43/57 字段 CSV；manifest；SHA256；原子写；run validator；长期 bundle 为 5 episode/18 step | 当前 bundle 明确排除正式结果；rosbag 仅记录 URI，未实现 T12 回放 |
| T07 标定/A_TEB | DONE | 3 场景、57 个 Gazebo episode、81 条观测；严格中心差分、符号证据和来源哈希；冻结 `A_TEB_v1` | 当前映射只对 Gazebo 训练合同有效；第二 seed 和实车验证仍未完成 |
| T08 基线 | DONE | 4 算法×3 场景×1 seed=12 episode；统一 CSV/evaluator；四个 bundle 均 valid；失败原样保留 | 当前为 validation pilot，单 seed；TEB-Tuned 是工程中点基线，不宣称最优 |
| T09 SAC Semantic-Eta | DONE | 5D Δη→冻结 A_TEB→9D θ；共享 254D observation；CPU SAC；VecNormalize；checkpoint/resume；20-step Gazebo smoke | 仅 smoke、非正式训练；正式 reward、总步数、curriculum/seed sweep 尚未冻结 |
| T10 SAC Direct-Theta | DONE | 9D normalized Δθ；与 T09 共享执行器/observation/reward/safety/预算；checkpoint/resume；配对验收；参数量/时延报告 | 20-step smoke 只能证明公平管线，不证明两种动作空间的性能差异 |
| T11 正式仿真/消融 | DONE (AMENDED) | 4 seed×5 组×70 test=20 run/1400 episode 完整矩阵；20/20 manifest valid；四组 paired comparison 各 280 对；3 个 seed105 run supplementary | 原 5-seed 预注册未完成，禁止宣称完整 5-seed 正式实验；500-step validation 阈值均未达到；NoSafety 有 1 次 move_base SIGSEGV fatal attempt |
| T12 安全修复/Residual SAC/rosbag-shadow | IN PROGRESS | 静态 footprint stress 通过；两 seed 各 2000-step 完整、14/14 test goal、0 crash/collision/emergency；validation change +0.4527/-0.0784 | 学习门失败且 projection 65.1%/69.5%，不扩预算、不做新配对；下一步单因素审查 Residual action/projection 匹配；rosbag adapter/live shadow 仍待实现 |
| FAM-TEB V2 | P1 FORMAL NEGATIVE / P4-F1 GLOBAL SEARCH TERMINAL | P1 270/270 attempt=1；效率 3.7624% / lower 2.7904%；post-P1 三轮总容量 2200，4 consumed + 2196 retired；P4-F1 independent terminal review pass | 明显提升未证明；无完整 post-P1 calibration/test、无 winner；禁止重试、扩轮/预算、训练、新 held-out、部署或实车结论 |
| V3-SCOPE | V3-11-F1 ACTUAL-TEB FORMAL POSITIVE / V3-12-P1 PAPER CLOSURE | fresh seeds 7101--7130；30 paired blocks / 60 episodes；goal 100%/100%；point 12.9288%，one-sided 97.5% lower 12.7253%；六 strata 全正；all formal gates + independent replay pass | 只证明三个冻结 single-reactive-actor Gazebo 模板内相对 actual TebLocalPlannerROS 0.8.4 的限定收益；共享 oracle actor track，不证明 perception、multi-actor/OOD、formal safety、training、deployment 或实车 |
| V5-E2E-SCOPE-RL | FAST FORMAL NEGATIVE / FROZEN | 60 fresh blocks；解析/V5 均时 14.1542/13.2917 s；降幅 6.0936%，95% CI [1.7867%,11.0069%]；contact/mask violation 0 | 未达到冻结 8% 实效门；不授权将 V5 改写为阳性 |
| V6 PCT-SCOPE-RL | FAST FORMAL POSITIVE / GAZEBO TRANSFER NEGATIVE / PAPER CLOSURE | 4×120k 新训练步；80 fresh blocks；V6/解析目标 60/49，均时 13.1156/14.3188 s；降幅 8.4024%，97.5% CI [3.4840%,13.7135%]；paper review 15/15 | 阳性仅限四个 sensor-level fast families；no-history 无差异；三次 Gazebo 净空门失败，禁止 Actual-TEB、formal safety、deployment 或实车声明 |
| T13 M2 闭环 | TODO | | |
| T14 统计/论文导出 | DONE (V2 + V3 + V4 + V6) | V6-05 导出 6 SVG/PDF、method/effect/family/episode/Gazebo CSV、claims、文献索引、manifest、20 页附录和 69 页合并稿；15/15 independent review pass | V6 Gazebo/Actual-TEB 迁移未通过；投稿必须保留该负边界 |

## 最近一次可复现命令

```bash
cd /home/robot/robot_ws_base_rl
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make -j4 -l4
catkin_make run_tests_m2_gazebo -j4 -l4
catkin_test_results build/test_results/m2_gazebo
catkin_make run_tests_teb_rl_tuner -j2 -l2
catkin_test_results build/test_results/teb_rl_tuner
roslaunch teb_rl_tuner t03_teb_client_acceptance.launch \
  gui:=false seed:=42 \
  report_path:=/home/robot/robot_ws_base_rl/artifacts/t03/teb_parameter_client_acceptance.yaml
roslaunch m2_gazebo m2_regression.launch run_test:=true gui:=false
roslaunch m2_gazebo m2_fixed_teb.launch gui:=false
roslaunch m2_gazebo m2_fixed_teb_regression.launch gui:=false seed:=42
catkin_make run_tests_thesis_experiment -j2 -l2
catkin_test_results build/test_results/thesis_experiment
roslaunch thesis_experiment t04_t06_pipeline_acceptance.launch gui:=false seed:=42
roslaunch thesis_experiment long_training_environment_acceptance.launch \
  gui:=false seed:=42 episode_count:=5
rosrun thesis_experiment validate_run.py \
  artifacts/t06/t04_t06_pipeline_run/run_manifest.yaml
rosrun thesis_experiment validate_run.py \
  artifacts/t07/training_environment_run/run_manifest.yaml
roslaunch thesis_experiment t07_gazebo_calibration.launch \
  gui:=false seed:=42 seeds_per_scene:=1
rosrun thesis_experiment analyze_teb_sensitivity.py \
  artifacts/t07/calibration_pilot/sensitivity_observations.csv \
  --mapping-version A_TEB_v1_gazebo_calibration \
  --output config/thesis_experiments/A_TEB_v1.yaml \
  --min-pairs 2 --min-sign-consistency 0.5 --min-abs-sensitivity 1e-8 \
  --top-k-per-eta 3 --freeze
roslaunch thesis_experiment t08_teb_baseline.launch \
  gui:=false seed:=42 algorithm:=TEB-Default run_id:=t08_teb_default_seed42
roslaunch thesis_experiment t08_teb_baseline.launch \
  gui:=false seed:=42 algorithm:=TEB-Tuned run_id:=t08_teb_tuned_seed42
roslaunch thesis_experiment t08_teb_baseline.launch \
  gui:=false seed:=42 algorithm:=Rule-TEB run_id:=t08_rule_teb_seed42
roslaunch thesis_experiment t08_fixed_dwa_baseline.launch \
  gui:=false seed:=42 run_id:=t08_fixed_dwa_seed42
rosrun thesis_experiment evaluate_t08_baselines.py
source scripts/activate_thesis_env.sh
python scripts/verify_rl_stack.py --output artifacts/t09/rl_stack_validation.yaml
roslaunch thesis_experiment t09_gazebo_sac_smoke.launch gui:=false seed:=42
roslaunch thesis_experiment t10_gazebo_sac_smoke.launch gui:=false seed:=42
rosrun thesis_experiment evaluate_t10_sac_pair.py
rosrun thesis_experiment evaluate_t11.py
sha256sum -c artifacts/t11/t11_reduced_study_checksums.sha256
roslaunch m2_gazebo m2_fixed_teb_regression.launch gui:=false seed:=42 \
  report_path:=/home/robot/robot_ws_base_rl/artifacts/t10/m2_fixed_teb_post_t10_regression.yaml
source scripts/activate_thesis_env.sh
python -m pytest -q src/application/teb_rl_tuner/tests src/tools/thesis_experiment/tests
rosrun thesis_experiment validate_thesis_config.py contract docs/thesis_experiment/experiment_contract.yaml
rosrun thesis_experiment validate_thesis_config.py metric-schema docs/thesis_experiment/schemas/episode_metrics_schema.csv
```

## 最近一次测试结果

```text
V3-11-F1 actual-TEB 正式配对评测：fresh seeds 7101--7130，30 paired blocks / 60
episodes 全部 attempt=1 完成，retry/resume/replacement/forfeiture 均为 0。V3 spatial-safe /
actual TebLocalPlannerROS 目标成功均为 30/30，平均受限 gate-to-goal 时间为
6.7683/7.7733 s，200,000-resample 配对改善 12.9288%，单侧 97.5% lower 12.7253%，
六个 scene×policy strata 全为正。contact 0，minimum conservative clearance 分别为
0.5110/0.0192 m，最大 V3 tracking error 0.03312 m；全部 17 项冻结门和独立 raw replay
通过。V3-12-P1 paper manifest SHA `319bce...8c0b`，independent closure SHA
`96d7d0...17b2`。结果只限三个冻结 single-reactive-actor Gazebo 模板和共享 oracle actor
track，不是 perception、multi-actor/OOD、formal safety 或实车结论。
V3-09-F1 正式配对评测：fresh seeds 6801--6830，30 blocks / 180 episodes 全部
attempt=1 完成，retry/resume/replacement/forfeiture 均为 0。两方法目标成功均为 90/90；
V3 spatial-safe / fixed-v1.20 shielded 平均受限目标时间为 8.8611/11.4167 s，配对
relative reduction 22.3844%，20,000-resample 单侧 95% lower 21.8235%，positive blocks
30/30。5320 decisions exact replay 1.0，minimum conservative clearance 0.165663 m，
clearance violation 0，P99 18.087844 ms。全部冻结 formal gates pass；assessment 与
independent reviewer cold check byte-identical。结果仅限 parameterized reactive single-actor
crossing，比较器不是 actual TEB。
V3-10-P1 pure-offline paper closure：5 个 deterministic exports 两次构建 byte-identical；
14-resource conventional multiline evidence manifest SHA `70353b...1938`，independent closure
review SHA `bee589...e0f4`，terminal manifest SHA `57f043...cf54`。未启动 ROS/Gazebo，
未创建 execution release/authorization/journal，未使用 seed、episode 或 training step。
V3-01--V3-10 终态回归为 104 passed / 0 failed（380.02 s）；V3-07--V3-10 快速回归为
17 passed。历史 V3-00 snapshot 单测为 5 passed / 2 failed：两项只检查 foundation 当时的
package.xml、CMake 与系统指南字节是否仍等于旧 persisted closure，后续 V3-01--V3-10
合法扩展使其漂移。旧 V3-00 artifacts 保持未改写，该历史 snapshot 不作为当前终态 gate。
R6-I6 pure-offline interpretation/design closure：I5 freeze manifest 的 13/13 资源 SHA 校验通过；
versioned reviewer 重验 14 项 source binding、6 journal binding、36 raw-resource binding、readiness
直接计数和六行 TTC/overlay 观察，并与 persisted review exact-equal，状态为
`offline_result_interpretation_design_closure_pass`。定向 pytest 4 passed。I6 没有启动 ROS/Gazebo、
没有创建 release/journal/attempt、没有训练或执行预算。future P1 design 固定 90 fresh blocks、
270 episodes、180 confirmatory paired episodes 与 0 additional training steps，但 execution 未授权。
发布前终态适用回归：I4/I5 release、I5 deterministic assessment 与 I6 共 87 passed；R5、TTC-D1
和 R6 execution-integration 共 53 passed；83 个 staged Python 文件 AST parse、218 个 staged YAML
safe-load 全部通过。完整历史快照集合另为 258 passed / 15 failed，15 项均要求后来已经合法生成的
I3/I5 release、attempt 或 downstream handoff “不存在”；这些执行前快照断言保留原样，不作为终态 gate。
R6-I5 bounded simulation execution：唯一 release SHA `9cef80f5...8b3d43` 通过 29-resource
exact-hash full prejournal；冻结 schedule SHA `b52d00a2...d5402` 的 6/6 unit 全部
`evidence_complete`，6/6 positive-clock bootstrap、direct readiness、scene snapshot 与 two-phase
teardown/restore 通过。expected/observed TTC 全部一致；semantic-clear 中 legacy 为 0 finite TTC /
18 non-NONE，circle-contact 为 0/0。retry=0、resume=false、terminal failure=null，未访问 held-out、
未训练、未连接实车。deterministic assessment replay 与 persisted report exact-equal，report SHA
`8ed09660...e5599`、`integrity_failures=[]`、`simulation_integration_validation_pass`；最终
ROS/Gazebo/process/ports 均清空。该结论仅限 fresh simulation semantic/execution integration，
不构成性能、泛化、winner 或 deployment claim。
R6-I3 readiness reviewer：`execution_readiness_closure_pass_release_absent`；
release-validator + readiness 共 24 项定向测试通过，reviewer `--check-only` byte-equivalent。
fresh scene 内存重编 14/14 byte-equal，3/3 execution compiled behavior equivalence；closure
136 local/187 edges、307 external、47 Python、9 runtime、0 unresolved，并重哈 inherited I2
106 local/301 external targets。全部为离线/静态检查，本阶段未启动 ROS/Gazebo。
catkin_make -j4 -l4：成功，共 73 个 catkin 包。
teb_rl_tuner：14 项 T03 核心事务测试、4 项配置测试、1 项 Gazebo rostest；
catkin_test_results 汇总 20 tests、0 errors、0 failures。
T03 Gazebo 验收：运行时发现并验证 9 个 double 参数的 description/min/max/current；
一次 9 参数 Reconfigure 的 request、ack、readback 一致，应用延迟和三个时间戳已记录，
启动快照恢复成功；报告为 `artifacts/t03/teb_parameter_client_acceptance.yaml`。
m2_gazebo：5 项 Ackermann C++ 测试与 4 项模型合同测试通过。
headless Gazebo 底盘回归：11/11，通过生成/reset、静止稳定、5m/10m、
低速倒车、左右定半径整圆、停止、固定障碍量距、TF 和 seed 重复性；结果在
`artifacts/t02/m2_chassis_regression.yaml`。
固定 TEB 回归：5/5，直线、左右转、单障碍绕行和 1.8m 窄通道均
`SUCCEEDED`；planner error 与控制周期超期均为 0，最大 cmd_vel 间隔
0.116s；结果在 `artifacts/t02/m2_fixed_teb_regression.yaml`。
T03 后再次运行固定 TEB 回归仍为 5/5，结果在
`artifacts/t03/m2_fixed_teb_post_t03_regression.yaml`。
T01--T12/V2 Python 与配置 pytest：201 passed。
teb_rl_tuner catkin tests：60 tests、0 errors、0 failures，包括 T03 Gazebo 验收和
300-step 无 ROS 长时压力测试。
thesis_experiment catkin tests：43 tests、0 errors、0 failures，包括 T04--T06 Gazebo 验收、
T08 evaluator、V2 合同、五类场景编译、统一 evaluator 和 V2-03/V2-04 验收；
nav_world_model 为 5 tests、teb_mode_manager 为 15 tests，均 0 errors、0 failures；m2_gazebo 为 34 tests、0 errors、
0 failures，包括 V2-02 在线 Gazebo 动力学 rostest。
T04--T06 Gazebo 验收：固定 TEB 完成 1.5m 目标并以 `goal` 终止；K=4、状态堆叠
244 维；`t_request < t_ack < t_active`；奖励窗口覆盖 9 个完整规划周期；全部 reward
和 CMDP cost 分量写入。T05 normal 路径不修改参数，独立 emergency dry probe 进入
`EMERGENCY` 并选择完整 9 参数保守回退；危险 probe 未写入 TEB。启动参数快照恢复成功。
T06 bundle 含 1 条 episode、1 条 step、manifest 和 2 个 SHA256 条目；CLI validator
返回 `valid: true`。该 bundle 为 pipeline validation，明确排除正式论文结果。
T06 后固定 TEB 回归仍为 5/5，报告为
`artifacts/t06/m2_fixed_teb_post_t06_regression.yaml`。
长期训练环境 Gazebo 验收：单次持久 Gazebo 会话完成 5/5 episode、18 条合法 transition；
每个 episode 重置 config sequence、恢复启动快照，bundle validator 返回 `valid: true`。
证据为 `artifacts/t07/training_environment_acceptance.yaml` 和
`artifacts/t07/training_environment_run/run_manifest.yaml`。
T07 标定按 3 种几何场景、每场景 1 个 seed 执行 57 个 episode（56 success、1 failure
原样保留），生成 81 条可追溯观测且无不完整正负扰动对。冻结映射为
`config/thesis_experiments/A_TEB_v1.yaml`，canonical SHA256 为
`1ca660f8d4f1863a93d75686bc0cafe8259942aaac60c3e2817c31162fcb1000`。
T07 后固定 TEB 回归仍为 5/5，报告为
`artifacts/t07/m2_fixed_teb_post_t07_regression.yaml`。
T08 统一矩阵完成 12/12 episode：TEB-Default 3/3、TEB-Tuned 3/3、Rule-TEB 2/3
（障碍场景安全急停）、Fixed-DWA 1/3（1 collision、1 timeout）。四个 bundle 均为
`valid: true`，统一 evaluator 完整矩阵和失败保留检查通过；报告为
`artifacts/t08/evaluation/t08_evaluation_report.yaml`。这些是 validation pilot，不是正式结果。
T08 后固定 TEB 回归仍为 5/5，报告为
`artifacts/t08/m2_fixed_teb_post_t08_regression.yaml`。
T09 严格 `.venv` 冻结 Python 3.8.10、Torch 2.4.1+cpu、Gymnasium 1.0.0、SB3 2.4.1；
CUDA false，SAC import、RECORD hashes、来源隔离和 `pip check` 全部通过。
Gazebo SAC smoke：16 timesteps 保存后恢复并继续 4 timesteps；replay buffer 20；actor 参数
L1 change 11.790；20/20 semantic transitions 合法存储；训练 8 episodes 全部 goal；
deterministic evaluation 2/2 goal。observation 已与 T10 统一为 244D core + 上一步实际
normalized-theta delta 9D + L1 1D。checkpoint 含 model/replay buffer/VecNormalize 和 SHA256。
证据为 `artifacts/t09/gazebo_sac_smoke/t09_gazebo_sac_smoke.yaml`。
T09 后固定 TEB 回归仍为 5/5，报告为
`artifacts/t09/m2_fixed_teb_post_t09_regression.yaml`。
T10 Direct-Theta smoke 同样为 16+4 timesteps、20 replay transitions；训练 7 episodes 全部
goal，确定性评估 2/2 goal。Semantic/Direct actor 参数量分别为 9546/9810，严格差异来自
5D/9D 输出层；短样本推理均值为 1.193/1.363 ms，仅作管线审计，不作性能结论。
配对 evaluator 验证 runtime、observation、reward、safety、training、smoke budget、checkpoint
和 acceptance 全部一致；报告为 `artifacts/t10/paired_sac_acceptance.yaml`。
T10 后固定 TEB 回归为 5/5，报告为
`artifacts/t10/m2_fixed_teb_post_t10_regression.yaml`。
T11 原始计划为 5 个训练 seed、25 个 run、1750 个 test episodes。用户报告剩余实验额度
不足后，保留原预注册不改，并新增 budget amendment：主分析仅使用 seed 101--104 的完整
4-seed 子矩阵。严格 evaluator 验证 20/20 run、1400/1400 test episodes、所有 run checksum，
四组 paired comparison 各 280 对；报告为
`artifacts/t11/evaluation/t11_evaluation_report.yaml`。seed105 已完成的 Semantic、Direct、
ProjectionOnly 三组仅作 supplementary，不插补缺失的 NoSafety/NoFallback。
主矩阵成功率：Semantic-Eta 42.86%、Direct-Theta 43.21%、ProjectionOnly 90.00%、
NoSafety 90.71%、NoFallback 42.86%；记录碰撞率均为 0。FullSafety 有 160/280 emergency-stop，
显示当前安全门限过度保守。8 个主训练 run 均未达到冻结 validation return 阈值 10，
因此 500-step 预算下不得宣称收敛。NoSafety seed104 另保留 1 次 move_base SIGSEGV fatal
attempt，不能把较高任务成功率解释为更安全。缩减研究总 checksum 在
`artifacts/t11/t11_reduced_study_checksums.sha256`。
T12 已先完成离线 shadow 核心与 T11 遗留安全修复。制动包络只触发 WARNING/保守推荐，
EMERGENCY 还必须满足净空不大于 0.35m 且持续 0.25s，避免将可绕行障碍直接误判为急停；
NaN/Inf 拒绝、9 参数边界/变化率投影和保守回退仍保留。四个 Semantic-Eta FullSafety seed
的 280 个测试 episode、991 条 step 遥测只读回放通过：原 160 个急停中 131 个净空大于
0.35m（81.875% 反事实误停候选）；已记录步骤中改进逻辑为 0 emergency、0 fault，
238 次投影、127 次 OOD 回退。EMA+投影将平均动作 L1 从 0.6232 降到 0.2353，降低
62.24%。报告和逐步决策位于 `artifacts/t12/offline_replay/`。该结果是离线反事实证据，
尚不等同于重新运行 Gazebo 后的任务成功率或实车安全结论。
T12 随后完成无训练闭环 Gazebo 配对复验：旧 FullSafety、T12Safety、ProjectionOnly 三种
方法，两个已选 T11 checkpoint seed（101/102），每组 10 个冻结代表性场景，共 60 个
episode。旧 FullSafety 为 8/20 goal、12/20 emergency-stop；T12Safety 为 18/20 goal、
2/20 emergency-stop；ProjectionOnly 为 19/20 goal，并有 1 次 planner failure。三组 collision
均为 0，T12Safety 的 planner failure、interface fault 和未知终止均为 0。相对旧安全逻辑，
T12Safety 成功率由 40% 提升到 90%（+50 percentage points），急停率由 60% 降到 10%
（-50 percentage points），全部预设门槛通过。报告为
`artifacts/t12/closed_loop/t12_closed_loop_report.yaml`。该结果验证安全修复的仿真闭环效果，
仍不是重新训练后的 SAC 优越性或实车结论。
T12 窄走廊优化与 Residual Semantic-Eta pilot 也已完成。安全状态机将全向最小净空用于
WARNING，将前向 ±20° 扇区用于硬 EMERGENCY 距离，并对确认的走廊结构使用 episode 级
latch 和逐步变化率限制的可行参数 profile。Residual 动作以 TEB-Tuned 为固定锚点、不累积，
风险时残差收缩，增加 EMA、4-step hold 和相同参数写入跳过。无训练 pilot 使用4个困难场景、
2 seed、3方法共24个正式 episode：legacy 3/8 goal、5 emergency；directional 5/8 goal、
0 emergency、0 collision、0 planner failure；zero-residual 6/8 goal、0 emergency、0 collision、
1 navigation planner abort。有效 run 中 process crash、dynamic-reconfigure failure 和 interface
fault 均为0，pilot acceptance passed。开发过程中曾暴露并保留由跳变/高频参数更新触发的
move_base SIGSEGV 尝试，修复后未进入正式 bundle。报告为
`artifacts/t12/residual_pilot/t12_residual_pilot_report.yaml`。这只证明新动作空间和运行链路
具备小规模训练前条件，不证明 residual SAC 已学习或优于基线。
配置 CLI：experiment_contract、runtime_defaults、A_TEB template、run manifest
template、episode schema（43 字段）和 step schema（57 字段）全部 valid。
隔离检查：仅保留本论文工作空间和 /opt/ros/noetic；外部 catkin_ws SB3
不可导入；rospy、PyYAML、pytest 可用。
本阶段已实现并配对 smoke-test Semantic-Eta 与 Direct-Theta SAC，但没有执行正式长训练或论文 seed sweep；
只向 T02 Gazebo 中的 TEB 发送训练验收参数且恢复启动快照；未连接
实车 ROS master、串口/CAN，未启动 `m2_driver`，没有真实车辆运动或实车参数写入。
T12 Residual Semantic-Eta SAC 两 seed 小预算训练已完成。seed101 validation mean return
为 24.4235（1000-step）和 24.1838（2000-step），seed102 为 23.5964 和 23.4201；两者均选择
1000-step checkpoint。合计 14/14 test goal，0 collision/emergency/planner/interface fault、
0 crash，checksum/validator 通过。该训练标记 `formal_result: false`；两个 seed 的 validation
均未随 1000->2000 step 改善，在离线诊断和冻结基线配对完成前不得宣称优化增益或扩预算。详见
`docs/thesis_experiment/CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md`。
T12 随后完成原训练离线诊断和 24-episode 冻结配对。原训练两 seed 实际均只访问
`t11-train-clear-straight`，projection intervention rate 为 85.87%，每 episode 首步 theta
到 residual anchor normalized L1 为 9.0。selected SAC、zero-residual、TEB-Tuned 在困难配对
场景中均为 4/8 goal；selected 相对 zero-residual paired return 平均 +2.0381，但成功率、
导航时间和路径长度没有优势，不能与 validation 下降共同构成稳定学习增益。
curriculum 单因素 amendment 已修复场景索引重置；两 seed 均覆盖全部 5 个训练场景，
合计 test 14/14 goal、0 collision/emergency/interface fault/crash。修复后 validation change
为 seed101 -0.2487、seed102 +0.0452，cross-seed mean -0.1018，学习门槛仍失败。修复后
projection intervention rate 仍为 87.06%。episode-anchor 第二单因素已冻结并启动，但 seed101
两次 bounded attempt 均在 episode reset 的参数恢复调用链上发生 `move_base` SIGSEGV；seed102
按完整性门未启动，validation 与学习性不可判定。完整结论见
`CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`。
随后 boundary atomicity 单因素在 500-step pilot 中通过全部门：0 crash/fault、7/7 test goal、
五场景和全 episode anchor 精确匹配，projection 降至 59.8%。但相同配置扩到 2000-step 后，
seed101 在活动 episode 的 oscillation/recovery 后发生 activation timeout 与 `move_base` -11；
seed102 和新三方法配对按门禁未启动。随后 activation-timeout recovery barrier 与静态 footprint
生命周期单因素修复通过 1300-step stress，并完成 seed101/102 各 2000-step：两 seed 均无
crash/SIGSEGV，合计 test 14/14 goal、0 collision/emergency/interface fault。validation change
分别为 +0.4527/-0.0784，projection 为 65.1%/69.5%；因此系统崩溃阻塞解除，但学习门仍失败，
新三方法配对未授权。seed101 boundary audit 另记录 1 次已恢复 activation-timeout barrier，
严格零事件结论不成立，详见冻结结论附录。
最新冻结离线诊断进一步拆分了 projection 来源：两 seed 训练 projection 为 65.1%/69.5%，
无前一安全干预时为 44.82%/48.81%，以 Ackermann 转弯半径耦合为主；前一步安全修改后
projection 为 92.48%/98.91%，rate-limit 主要来自 WARNING profile 后立即回指固定 anchor。
原始动作饱和仅约 3.8%，Residual radius 平均利用率最高约 14%，因此当前没有依据先改 radius、
reward、EMA 或 hold。禁止新训练和配对；下一入口是预注册一个动作—执行对齐学习因素 amendment。
```

## 当前阻塞项

- V3-14-I1 已完成调度--launch--runtime 动作身份的失效关闭完整性闭环；V3-14-S1 的 45/45
  fresh 单元随后因 actor 坐标/速度配对差 0.066074 m/0.072081 m/s 超过 0.05 门而形成完整
  阴性结果。V3-16-D1 纯离线归因支持候选分支前闭环响应放大假设；V3-16-S1 只把 actor 响应
  启用时刻从 0 s 改为 12.55 s，并使用 seed7741 预检、fresh seeds7742--7749 正式执行。
  新的 45/45 单元全部 attempt1 完成，速度差降至 0.008597 m/s（下降 88.07%）并通过门，
  但坐标差仍为 0.067472 m，只有 5/8 区块通过坐标门。因此整体仍是终止性阴性结果，
  `predictor_fit_authorized=false`，不得启动多人 actual-TEB、感知/OOD 或训练。原 assessor 的
  raw-report 资源索引失败被保留；R1 只修复路径解析并通过独立审查，不改变科学结论。V3-17
  论文导出把 V3-14/V3-16 的对照和单因素机制结果加入新 PDF。详见
  `CURRENT_V3_16_RESPONSE_GATED_TERMINAL_HANDOFF.md` 与 `CURRENT_V3_17_PAPER_EXPORT_HANDOFF.md`。

- 历史 V3-13 的 recovery-only r3 几何、终点保护、候选特定预测资格和 planner integration 已完成；
  R16 的单对 actual TEB/V3 预检仍是有效阴性结果：actual TEB 到达目标但动态净空
  -0.8965 m，V3 安全为 +0.06883 m 但旧 r2 未完成共同目标。后续 r3 支持链中，R19-R2
  40/40 因 ego/actor 配对误差失败，100 Hz 公共前缀修复通过；R19-R4 40/40 又因一个边界
  样本距0.12 m终点门多1.16 mm而使触发率7/8。0.15 m提前停车预检在 fresh seed7696
  通过，但最终 R19-R6 复用的一单元 runner 把所有调度动作硬编码为 r3；检测到前两行
  scheduled/runtime ID 不一致后立即停止。R19-R6 0科学单元、2 consumed、38 forfeited，
  seeds7697--7704不可复用；不得 repair-and-rerun、拟合r3 head、运行seeds7301--7316、
  开始感知退化或训练。下一入口仅是纯离线 action-binding fail-closed 完整性设计，详见
  `CURRENT_V3_13_MULTIACTOR_HANDOFF.md` 最后章节。

- R6-I5、P1-E1 与 post-P1 P2/P3/P4 链均已终态冻结。P1 是 `formal_result=true` 的有效阴性
  结果：安全/任务与机制通过，但 3.7624% / 2.7904% 未达到当前 10% / 5% 明显提升门。
  post-P1 的 2200-unit 容量已全部关闭，且没有完整 calibration/test matrix；不得再次调用
  `--execute`、替换 seed、修复后复用 P4 seed5924、扩轮/预算、训练或访问新 held-out。
  当前不能形成阳性性能、winner、RL 提升、部署或实车结论。
- 默认 login shell 混入 `arena_ws`/`catkin_ws` 的 ROS、Python 与 Gazebo 路径，P4-I1 正因
  `m2_gazebo` package resolution 失败而在 Gazebo 启动前终止。该根因已冻结为完整性证据，
  不是重试授权；后续仅做离线证据复核时仍应使用 `env -i` non-login shell，只 source Noetic
  与本论文 workspace。
- V2-04G-R3/R4/R4-R1 的历史失败与不利 seeds 继续冻结，不得重试或恢复。V2-04G-R5
  已在 readiness identity 1/6 因 TTC coverage mismatch terminally stop；seed5111 和全部
  失败证据已保留。R6-I1 也已在 sequence1 seed5141 的 bootstrap readiness 终止；1 unit
  consumed、5 units forfeited、resume forbidden。不得重试、resume、消费 R5 剩余 68
  budget 或 R6-I1 剩余 units，也不得继续任何 R6-I1 episode。

- 当前本地论文分支为 `base_rl`，跟踪 `origin/base_rl`；R5--I6 代码、合同、冻结证据和论文
  正文章节在本轮按显式路径整理发布，既有 dirty submodule 不纳入该发布。
- stable workspace `/home/robot/robot_ws` 在 I5 中未修改、未 source、未连接。I5 最终检查时
  ROS/Gazebo/move_base 进程和保留端口均为空；未来任何新执行仍必须重新建立独占仿真边界。
- FAST_LIO、arena-rosnav-3D、Livox driver 等复制来的子模块存在既有 dirty 状态，未清理或重置。
- T11 reward、课程、场景、checkpoint 和 seed 合同已冻结；原 5-seed 计划因额度约束缩减，主结论只能基于 amendment 中的 4-seed 完整矩阵。
- M2 模型当前是确定性运动学接口模型，不包含已标定轮胎力、悬挂、转向滞后或制动动力学。
- 当前仿真测得制动距离为 0m，仅表示运动学插件即时响应停止命令，不能用于论文中的实车制动性能结论。
- M2 footprint、轴距、轮径、传感器安装位姿、最小转弯半径、倒车语义、制动能力、时延和安全参数范围仍需实车标定。
- 实车源码在 `/cmd_vel` 倒车换算中使用 `abs(linear.x)`；仿真按标准 body twist 使用有符号速度，此差异必须低速实车验证。
- 当前 `ChassisParameterServer` 源码请求体为空，只能查询，不能按说明文字在线设置。

## 数据与安全状态

- 是否存在正式实验 CSV：是；V3-12、V3-15 与 V3-17 论文闭包均包含可复核 CSV
- 是否允许实车参数写入：否
- 是否允许实车运动：必须由现场用户逐次批准
- NoSafety/ProjectionOnly/NoFallback 实车运行：禁止

## 下一步

1. 新会话最后必须读取 `CURRENT_V2_04G_P4_F1_TERMINAL_HANDOFF.md`。P1/I5/T12、R5/R6、
   P2/P3/P4 的 release、seed、journal、失败和 retired budget 全部保持冻结。
2. FAM-TEB 性能搜索已终止；不再创建 P5、新 factor round、calibration/test release、训练任务或
   winner。禁止把环境修复视为 P4-I1 retry 权限。
3. 允许的 FAM-TEB 后续仅限纯离线论文整理、证据哈希复核和归档；必须保留
   “未证明明显提升”、P1 3.7624% / 2.7904% 与 post-P1 无完整 test matrix 的完整上下文。
4. T12 继续保持原冻结边界，不得用 P2/P3/P4 seed、配置或未来数据重新选择 checkpoint。
5. 在实车重新标定并逐次取得现场批准前，实车运动、CAN、online TEB 写入、实车参数范围、
   制动/时延、安全距离和实车保守回退均保持禁止或 TBD。

## V3-18--V3-22 论文材料整理终态（2026-08-06）

- V3-18-A0 已完成项目审计、系统目标/架构、实验库存、方法证据矩阵、标准采集 schema 和缺口分析；
  独立审查通过。V3-18-B0 将 V2-P1、V3-05、V3-09、V3-11、V3-16 适配为 825 行 canonical
  dataset；首版源哈希失败保留，R1 只修复 manifest，dataset 字节不变并通过独立审查。
- V3-19 建立 V3 spatial-safe、Actual TEB 0.8.4、Fixed-v1.20 shielded 与
  no-candidate-conditioning 四方法框架。24/24 资格回合完成，但 V3 5/6 goal 未过资格门；正式
  seeds 8001--8030 未执行并退役。
- V3-20 的失效计入式矩阵在 sequence78 因 ROS master 端口落入 ephemeral range 终止：77 完成、
  1 terminal failed、42 forfeited，不重试。V3-21 只修复端口 namespace，使用 fresh seeds
  8201--8230 完成 120/120 attempt1 回合。
- V3-21 中 V3/Actual TEB 成功率为 93.33%/100%，受限均时 7.785/7.770 s；V3 相对 TEB
  点降幅 -0.193%，双侧 95% CI [-19.867%,12.982%]，未证明时间优势。相对 fixed-v1.20 和
  去候选条件化消融分别缩短 64.614% 与 49.454%。四方法 contact 均为 0。
- sequence107 的 gate x 误差比 0.03 m 容差多 2.286 mm，回合按 ITT 保留；排除 block27 后
  点效应 -0.643%，结论不变。因此 `formal_result=false`，但 120 回合 characterization 可报告。
- V3-22-P1 已从冻结 CSV 确定性导出 5 张图的 PDF/SVG/300-dpi PNG、5 个 booktabs 三线表、
  中英文结果叙述和 claims YAML；manifest SHA `658e906c...973d0`，独立 review SHA
  `88547275...0959`。没有 ROS/Gazebo、训练、硬件或实车动作。
- 当前论文正文已加入四方法复现与消融章节，并成功编译为 32 页预览。终态必须同时报告：
  V3-11 是限定历史正式阳性；V3-21 是未复现 Actual TEB 失效计入式优势的新鲜表征；两者不池化。
- V3-22-P2 终态 PDF 归档与独立 paper review 已完成。PDF 为 32 页 A4，全部字体嵌入，
  无 LaTeX error/undefined reference；PDF SHA `22f4e423...58b7d`，review SHA
  `adb43177...3d7a5`。历史 V3-19--V3-22 identities、seeds、budgets 和 artifact 目录均不可
  重跑或覆盖。当前没有剩余仿真或论文导出任务。
