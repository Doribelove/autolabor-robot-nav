# TEB RL Thesis Development Status

更新时间：2026-07-21

## 当前阶段

- 阶段：T12 冻结证据维护 + FAM-TEB V2-04G-R6-I6 pure-offline result interpretation/design closure complete
- 状态：T00--T11 已完成；T12 两 seed 完整但学习门失败；V2-04G-R5 与 R6-I1 均永久停止，I5 终态冻结且不得重跑。I6 以 13-resource SHA 清单冻结 I5 关键证据，确定性复核报告 `c1fd43205d0f3b3c6a029590b33808812dc8db795bdcf4b270c345e033b9dd68` 返回 `offline_result_interpretation_design_closure_pass`；现有结论只限 fresh simulation semantic/execution integration。未来 V2-04G-P1 已预注册 90 个全新 scene-seed block / 270 episodes、零新增训练步数，但 `execution_authorized=false`、budget=0，未创建 release 或运行状态
- 当前负责人：Ubuntu Codex CLI + 用户

## V2 架构设计状态

- 系统指南：`docs/thesis_experiment/V2_SYSTEM_GUIDE.md`
- 架构代号：`FAM-TEB V2`
- 设计状态：`DESIGN COMPLETE`
- 实现状态：`R6-I5 6/6 SIMULATION INTEGRATION PASS FROZEN / R6-I6 OFFLINE INTERPRETATION CLOSURE PASS`
- 正式实验状态：`NO PERFORMANCE RESULT / P1 DESIGN PREREGISTERED / 0 AUTHORIZED + 0 CONSUMED / NO WINNER / formal_result=false`
- 实车状态：`FORBIDDEN WITHOUT NEW CALIBRATION AND ON-SITE APPROVAL`
- 与 V1 的关系：V1/T00--T12 合同、runner、配置和 artifacts 保持冻结；V2 必须使用独立的
  schema、配置、manifest 和 artifact 根目录。
- 已完成入口：V2-00 基线隔离、V2-01 机器合同/消息骨架和两项基础修复、V2-02
  可信仿真动力学/五类场景/统一 evaluator、V2-03 局部世界模型/动态跟踪预测/健康/无标签
  规则监督器、V2-04 Anchor Bank/类型化 profile/feasible decoder/平滑 shadow 事务和零训练
  规则闭环；V2-04B 已增加 simulation-only 20 参数 typed TEB 真实事务和 calibration split，
  最新验收见 `artifacts/v2/component_acceptance/v2_04b_acceptance.yaml`。
- 最新入口：独立 `V2-04G-R6-I6` 已完成纯离线解释闭环。I5 13-resource freeze manifest SHA 为 `40d9eba914840d33a7966f7c5bff972e94d9123239b1cc1cc0c0971752288935`；I6 reviewer 重验 14 项 source binding、6 journals、36 raw bindings 和六行语义计数，并与持久化报告 exact-equal。论文可宣称集成正确性和可复核性，不可宣称性能、泛化、安全、winner 或部署。最后一轮 `V2-04G-P1` 性能设计固定 3 roles × 30 fresh blocks × 3 methods = 270 episodes，主确认性对比 180 episodes，不增加训练预算；当前没有执行授权。详见 `docs/thesis_experiment/CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_R6_I4_R6_I5_R6_I6_RESULT_INTERPRETATION_HANDOFF.md`。
  不得把 V2 当作 T12 action/projection 单因素
  续跑，也不授权新训练或实车闭环。所有模式和参数阈值继续保持 `runtime_ready=false`。

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
| FAM-TEB V2 | R6-I5 EVIDENCE FROZEN / R6-I6 OFFLINE CLOSURE PASS / P1 DESIGN ONLY | 13-resource freeze；I6 deterministic interpretation exact-equal；论文 claims matrix；90 fresh-block / 270-episode final performance design，additional training=0 | I5 不可重跑；P1 `execution_authorized=false`、budget=0，须独立审查、新 release 和明确模拟授权后才可执行；当前无性能、winner、V2-05/SAC/实车结论，`formal_result=false`、`runtime_ready=false` |
| T13 M2 闭环 | TODO | | |
| T14 统计/论文导出 | TODO | | |

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

- R6-I5 已完成唯一获准执行并由 I6 冻结：6/6 consumed、0 forfeited、0 retry/resume，terminal
  assessment pass。不得再次调用 `--execute`、替换 seed、扩预算或从已有 journal resume。I6 只完成
  离线 result interpretation/design closure；未来 P1 尚无 execution authorization、budget、release、
  schedule materialization 或 journal。当前仍不能形成性能、泛化、winner、训练或实车结论。
- 默认 login shell 混入 `arena_ws`/`catkin_ws` 的 ROS、Python 与 Gazebo 路径。未来任何复核或
  runner 必须使用 `env -i` non-login shell，只 source Noetic 与本论文 workspace。
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

- 是否存在正式实验 CSV：否
- 是否允许实车参数写入：否
- 是否允许实车运动：必须由现场用户逐次批准
- NoSafety/ProjectionOnly/NoFallback 实车运行：禁止

## 下一步

1. FAM-TEB V2 新会话在 I5 execution handoff 后继续读 `CURRENT_V2_04G_R6_I1_R6_I2_R6_I3_R6_I4_R6_I5_R6_I6_RESULT_INTERPRETATION_HANDOFF.md`。R5/R6-I1 终止、I5 终态和失败 I3 release 均保持冻结，不得再次启动。
2. 论文继续以 `thesis_draft/CH03_FAM_TEB_SYSTEM_DESIGN.md`、`CH04_EXPERIMENT_INTEGRITY_METHOD.md`、`CH05_I5_INTEGRATION_RESULTS.md` 和 claims matrix 为正文基线；I5 只证明 semantic/execution integration，不得提前写成性能提升。
3. 最后一轮性能目标采用已预注册的 `V2-04G-P1` 设计：3 scene roles × 30 fresh blocks × 3 methods = 270 episodes，主配对 180 episodes，additional training=0。先做独立离线设计审查和可计算性/功效复核，不先扩训练预算。
4. P1 若要实际运行，必须建立新 closure/review/release、fresh budget 和独立明确的 simulation authorization；当前设计文件不是授权，不得复用 I5 identities、seeds 5161--5163 或任何历史 budget。
5. 1.5/1.0 秒不属于本轮 factor；仍只是 D1 frozen trace 上的离线讨论值，不是合格配置或授权。winner freeze、held-out 5001--5010、V2-05/SAC 和实车仍分别需要独立授权。
6. T12 新会话先读 `CURRENT_TEB_RL_HANDOFF.md`、`CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md` 和 `CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`，禁止重复启动。
7. 当前不扩预算，也不重启任何历史 pilot；Residual action/projection 离线审查已经完成。
8. 若继续训练，必须先登记一个动作—执行对齐学习 amendment；不得同时修改 residual radius、reward、EMA、hold、SAC 或 safety。只有两 seed validation 均上升后才启动新三方法冻结配对。
9. T12 后续仍需用新 Gazebo/rosbag 数据做 live shadow 确认；默认只读，不向实车 TEB 写参数。
10. T04--T11 继续复用 T03 四重仿真门控、快照恢复、manifest 和失败保留规则。
11. 在实车标定完成前，实车参数范围、制动/时延、安全距离和实车保守回退继续保持 TBD。
