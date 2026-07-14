# T12 Residual Semantic-Eta SAC 两 seed 小预算训练交接

更新时间：2026-07-13 21:29 CST

本文件保存用户要求“开始两 seed 小预算 Residual SAC 训练”后的完整状态。新 Codex
会话必须先按 `AGENTS.md` 的顺序阅读主交接书和实验合同，再读本文件。本文记录的是
仿真训练，不授权实车运动、CAN/串口访问或实车 TEB 参数写入。

后续离线诊断、三方法配对、curriculum、episode-anchor、boundary atomicity 与静态 footprint
生命周期单因素审查均已有冻结证据。最新修复消除了动态重配置时反复重建静态 footprint 的
竞态，两 seed 2000-step 均完整结束且不再发生 `move_base` -11；但 seed102 validation 下降，
因此学习门仍失败，新三方法配对未启动。最新 action/projection 离线诊断也已完成；训练
projection 均值 67.3%，主要结构来源为 Ackermann 耦合与 WARNING 后回指 anchor 的限速。
继续工作前还必须阅读
`docs/thesis_experiment/CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`。本文件第 1--10 节保留
原两 seed 训练的冻结事实，不应再把历史“下一步”当作当前入口，也不得重启 anchor pilot。

## 1. 当前最重要状态

- 训练总入口：`scripts/run_t12_residual_training.sh`
- 训练根目录：`artifacts/t12/residual_training/`
- 合同：2 个训练 seed（101、102），每个 2000 environment steps。
- checkpoint：每个 seed 在 1000、2000 step 保存；每个 checkpoint 用 3 个 validation
  episode 评估，模型选择只看 validation，不看 test。
- test：每个 seed 使用选中的 checkpoint 跑 7 个冻结 ID/OOD episode。
- 正式性：`formal_result: false`，这是是否值得扩展训练的两 seed 小预算门槛，不是论文
  正式性能结论。
- 2026-07-13 12:58 CST：seed 101、102 均完成，汇总报告 `passed: true`，当前没有训练、
  Gazebo、move_base 或 ROS launch 残留进程。
- 冻结总报告：`artifacts/t12/residual_training/t12_residual_training_report.yaml`，SHA256
  `655fba8c2c32347a1ce44d9e522d76e1c676f830ea75b2fa68fd69f1280fdaad`。
- 两 seed 均在 1000-step checkpoint 获得更高 validation mean return；继续到 2000 steps
  均下降。因此本轮只通过运行完整性与安全门槛，没有通过学习增益门槛，不得直接扩预算。

## 2. 用户目标和进入本轮训练前的结论

用户希望最终看到导航算法具有优化和较好效果，但要求先修复 T11 遗留安全问题，随后
进行 Residual Semantic-Eta SAC。已经完成的前置事实：

1. T11 的旧 FullSafety 过度保守，4-seed 主矩阵中 160/280 test episode 急停；500-step
   Semantic/Direct SAC 均未达到冻结收敛阈值，不能宣称学习有效。
2. T12 无训练 60-episode 闭环复验中，旧 FullSafety 为 40% success/60% emergency，
   T12Safety 为 90% success/10% emergency，三组 collision 均为 0，安全修复门槛通过。
3. T12 窄走廊/Residual 无训练 pilot 共 24 episode：legacy 3/8 goal、5 emergency；
   directional safety 5/8 goal、0 emergency/collision/planner failure；zero-residual 6/8
   goal、0 emergency/collision，另有 1 次 navigation abort。该结果只证明新环境具备
   小规模训练前条件。

## 3. 本轮新增实现

### 3.1 Residual 动作合同

配置：`config/thesis_experiments/t12_residual_training.yaml`

- `simulation_only: true`
- `real_vehicle_use_forbidden: true`
- `training_enabled: true`
- 固定锚点：T08 `TEB-Tuned`
- SAC 输出仍是 5D Semantic-Eta，经冻结 `A_TEB_v1` 映射到 9D normalized theta residual。
- residual 不跨步累积到整个参数盒，而是围绕固定锚点受限变化。
- 9 个 normalized residual radius：
  `[0.35, 0.25, 0.30, 0.30, 0.25, 0.20, 0.30, 0.25, 0.25]`
- 风险缩放下界：0.20。
- 动作 EMA：alpha 0.35。
- 决策保持：4 steps。
- 所有候选动作仍经过 T05 参数边界、变化率、耦合投影和 T12Safety。

### 3.2 代码接入

- `src/application/teb_rl_tuner/src/teb_rl_tuner/semantic_action.py`
  - `ResidualSemanticMapping.from_files` 接受布尔型 `training_enabled`；仍拒绝缺失、错误
    类型和非法配置。
- `src/tools/thesis_experiment/src/thesis_experiment/t11_training.py`
  - 新增 `semantic_mode=residual_training`；
  - train phase 强制 residual 配置的 `training_enabled=true`；
  - residual pilot 仍禁止训练；
  - residual training 使用 residual Gym wrapper；
  - checkpoint metadata、run manifest 记录实际 safety mode 和 residual config/hash。
- `scripts/run_t12_residual_training.sh`
  - seed 101/102 串行执行，避免两个 Gazebo/ROS master 争用；
  - 每个 seed 最多两次尝试；失败输出移到 `failed_attempts/`，不删除；
  - 已验证的 seed 会跳过；
  - 两个 seed 完成后生成汇总报告和 checksum。

## 4. 已完成的 seed 101 结果

目录：`artifacts/t12/residual_training/runs/t12_residual_training_seed101/`

完整性：

- 2000/2000 training steps 完成。
- checkpoint 1000 和 2000 均包含 model、replay buffer、VecNormalize 和 manifest SHA256。
- `episodes.csv`：509 行。
- `steps.csv`：2116 行，2116/2116 transition stored。
- split：train 496、validation 6、test_id 4、test_ood 3。
- 509/509 episode 均以 goal 成功。
- `failure_index.yaml` 为空列表。
- run validator：valid，校验 4 个 run checksum。
- 已在 run 目录执行 `sha256sum -c checksums.sha256`，4/4 成功。

模型选择：

| checkpoint | validation episode returns | mean return |
| --- | --- | --- |
| 1000 | 19.6478, 27.7144, 25.9083 | 24.4235 |
| 2000 | 18.9720, 27.4043, 26.1750 | 24.1838 |

- validation return change（2000-1000）：-0.2397。
- 按冻结 validation-only 规则选择 1000-step checkpoint。
- 7 个 test episode 未参与模型选择，结果为 7/7 goal：4/4 ID、3/3 OOD。
- test 中 0 collision、0 emergency stop、0 planner failure、0 interface fault。
- 全部 2116 step 的 safety mode：2038 NORMAL、78 WARNING、0 EMERGENCY/FAULT。
- projection modified 1803 steps，safety modified 54 steps，fallback 0。
- 日志中未发现 move_base SIGSEGV、Python traceback 或 dynamic-reconfigure service failure。

解释边界：seed 101 证明训练管线稳定且选中策略安全完成这 7 个测试场景；但 1000 到
2000 step 的 validation 均值没有提高，而且固定锚点本身较强，因此单个 seed 不能证明
Residual SAC 已学习出优于 zero-residual/TEB-Tuned 的策略。

关键文件：

- `artifacts/t12/residual_training/runs/t12_residual_training_seed101/model_selection.yaml`
- `artifacts/t12/residual_training/runs/t12_residual_training_seed101/t11_run_report.yaml`
- `artifacts/t12/residual_training/runs/t12_residual_training_seed101/run_manifest.yaml`
- `artifacts/t12/residual_training/runs/t12_residual_training_seed101/checksums.sha256`
- `artifacts/t12/residual_training/logs/t12_residual_training_seed101_a1.log`

## 5. 已完成的 seed 102 结果

目录：`artifacts/t12/residual_training/runs/t12_residual_training_seed102/`

完整性：

- 2000/2000 training steps 完成。
- checkpoint 1000 和 2000 均包含 model、replay buffer、VecNormalize 和 manifest SHA256。
- `episodes.csv`：512 行；`steps.csv`：2115 行。
- run validator：valid，校验 4 个 run checksum；run 目录 checksum 4/4 成功。
- 唯一 failure index 是训练预算恰好在活动 episode 中结束时记录的 `operator_stop` 截断；
  `operator_intervention_count=0`，不是进程崩溃、规划失败或人工干预。

模型选择：

| checkpoint | validation episode returns | mean return |
| --- | --- | --- |
| 1000 | 19.2049, 25.9211, 25.6632 | 23.5964 |
| 2000 | 19.3279, 25.9500, 24.9823 | 23.4201 |

- validation return change（2000-1000）：-0.1763。
- 按冻结 validation-only 规则选择 1000-step checkpoint。
- 7 个 test episode 未参与模型选择，结果为 7/7 goal：4/4 ID、3/3 OOD。
- test 中 0 collision、0 emergency stop、0 planner failure、0 interface fault。
- 日志中未发现 move_base SIGSEGV、Python traceback 或 dynamic-reconfigure service failure。

原始启动命令保留如下，用于审计，不得因为看到该命令而重新启动：

```bash
roslaunch thesis_experiment t11_formal_run.launch \
  gui:=false task:=T12 gazebo_seed:=102 \
  algorithm:=RL-TEB-Semantic-Eta training_seed:=102 phase:=train \
  safety_mode:=T12Safety semantic_mode:=residual_training \
  residual_config:=/home/robot/robot_ws_base_rl/config/thesis_experiments/t12_residual_training.yaml \
  acceptance_timesteps:=2000 acceptance_eval_seed_limit:=1 \
  run_id:=t12_residual_training_seed102 \
  output_dir:=/home/robot/robot_ws_base_rl/artifacts/t12/residual_training/runs/t12_residual_training_seed102
```

日志：`artifacts/t12/residual_training/logs/t12_residual_training_seed102_a1.log`。

## 6. 新会话核验顺序

第一步只读检查，绝对不要重复启动：

```bash
cd /home/robot/robot_ws_base_rl
pgrep -af 'run_t12_residual_training|t11_formal_run.py|t11_formal_run.launch|gzserver|move_base/move_base'
tail -n 80 artifacts/t12/residual_training/logs/t12_residual_training_seed102_a1.log
find artifacts/t12/residual_training/runs/t12_residual_training_seed102 \
  -name checkpoint_manifest.yaml -o -name t11_run_report.yaml -o -name model_selection.yaml
```

固定核验结果：总报告、seed101、seed102 三层 checksum 已通过；若将来发现文件 hash 漂移，
应停止后续分析并审计文件来源，不得用重跑训练覆盖现有证据。

`./scripts/run_t12_residual_training.sh` 现为历史复现入口，不是下一步命令。未经用户明确
要求和新的实验 amendment，不得重跑或扩大本轮训练。

已知脚本注意项：汇总器当前固定扫描 `_a1.log`。如果同一次脚本运行中 seed 自动进入
attempt 2，应先修正汇总器为读取成功 attempt 的日志，或人工核对 a1/a2；否则 crash
计数可能引用第一次失败日志。没有发生 retry 时无需改动。

## 7. 两 seed 完成后的验收和判断

冻结汇总：`artifacts/t12/residual_training/t12_residual_training_report.yaml`

完整性门槛：

- 两个 seed 均有 2000 steps、2 个 validation checkpoint、7 个 test episode；
- 0 collision；
- 0 interface fault；
- 0 process crash；
- 0 dynamic-reconfigure failure；
- checkpoint/hash/run validator 全部通过。

学习性判断必须与完整性分开：

- 比较两个 seed 的 `validation_return_change` 和所选 checkpoint；
- 汇总 14 个 test episode 的 success、collision、emergency、planner/interface failure；
- 不得因为 test 14/14 goal 就直接宣称 RL 有增益；这些场景中固定 TEB-Tuned/zero-residual
  可能本来就很强；
- 实际两个 seed 均无 validation 上升，冻结决定是先检查奖励尺度、探索动作、训练场景
  难度、residual 半径、EMA/hold、投影和安全干预，不直接扩大预算；
- 在同一冻结场景上做 selected SAC、zero-residual、TEB-Tuned 小规模严格配对评估；
- 只有诊断和配对结果共同显示稳定增益，才允许提出增加 seed/预算；否则只修改一个因素，
  先登记合同 amendment，再执行新的小预算 pilot。

## 8. 冻结结论边界

- 运行完整性：通过。两 seed、4 checkpoint、14 test episode、validator/checksum 完整。
- 仿真安全性：本轮 test 为 0 collision、0 emergency、0 planner/interface fault、0 crash。
- 学习趋势：未通过。seed101 与 seed102 的 2000-step validation 均低于 1000-step。
- 性能增益：未知。14/14 goal 不能排除 TEB-Tuned 强锚点或 zero-residual 本身已足够。
- 正式性：`formal_result: false`；本轮不得写成论文正式优越性结论或实车安全结论。

## 9. 最近测试和构建

在启动本轮训练前已经完成：

- Python/配置测试：当前 134 passed。
- 完整 `catkin_make`：成功，遍历 71 packages。
- 训练期间未接触 `/home/robot/robot_ws`，未启动实车 `m2_driver`，未访问 CAN/串口。

## 10. 不能丢失的工作区规则

- 当前分支 `base_on_rl`，dirty，尚未 commit/push。
- 大量既有用户文件、GPS 工作和子模块本来就是 dirty；不要 reset、checkout 或清理。
- 不要删除 `failed_attempts/`、失败日志或失败 episode。
- artifacts/checkpoint 是生成物，当前用于实验追溯；不要为了 Git 整洁而删除。
- 所有当前 T12 数值均为 Gazebo 仿真候选，不是实车标定或实车安全结论。
