# CURRENT V2-04G-R5 DESIGN HANDOFF

日期：2026-07-18

## 结论

V2-04G-R5 的独立、simulation-only、calibration-only TTC 鲁棒性单因素阶段已完成
预注册、候选材料化、fresh 场景派生/编译、fail-closed validator、单元测试和无
ROS/Gazebo dry-run 审计。

本阶段尚未启动任何正式 Gazebo batch，也没有执行 readiness episode、TTC component
evidence 或导航 episode。因此当前结论只是“设计与离线审计通过”，不是性能结果，
`formal_result=false`、`runtime_ready=false`，没有 winner。

权威 dry-run 审计：
`artifacts/v2/calibration/v2_04g_r5/v2_04g_r5_dry_run_audit.yaml`。
SHA256：`d7a3113c89b08889dc754a72f4e792c422225f19504ab3218d9712cf46dee8e1`。

## 冻结起点与唯一因素

冻结起点为 `CURRENT_V2_04G_R4_R1_HANDOFF.md` 及 R4-R1 stage report
`e1ad0aeb7739e8c1abad0f17059f8dbe31c671dd03584d96637830033e5ab22a`。

R4-R1 的 Maneuver `min_obstacle_dist=0.30 m` 只作为已证明能修复净空的固定、
非排名输入，不是系统 winner；forward/reverse 均保持 0.30 m，`inflation_dist`
保持 0.52 m。

唯一允许变化的行为字段：

```text
supervisor.dynamic.predicted_ttc_max_s
```

冻结 evaluator 的相对 TTC horizon 5.0 s，并冻结
`overlay_release_confirmation_s=0.20 s`。Anchor 其余值、supervisor 其余几何与
transition 字段、typed transaction、bounded transaction join、三流原子 world-model
join、倒车状态机、切换/滞回、净空与其他时间机制全部保持不变。

材料化后的候选除身份元数据外，行为差异只有上述一个字段：

| ID | horizon | 角色 | winner 资格 |
| --- | ---: | --- | --- |
| `r5_ttc_control_h500` | 5.0 s | 冻结 m030 timing control | 否 |
| `r5_ttc_h450` | 4.5 s | TTC timing repair candidate | 是 |
| `r5_ttc_h400` | 4.0 s | TTC timing repair candidate | 是 |

缩短 runtime supervisor horizon 的目的，是在冻结的 5.0 s evaluator envelope 内测试
较晚的 Dynamic overlay 介入/释放时序能否恢复 fresh conflict coverage；4.0 s 只是
runtime prediction window，不是保证反应时间。

## Fresh seeds、场景与预算

- readiness 实际探针 seeds：5111--5113，两个 conflict、一个 clear；
- readiness compile-support-only seeds：5114--5117，仅补齐冻结编译器要求的
  CRUISE、STATIC_DENSE、CORRIDOR、MANEUVER 四类，不执行、不计证据、不排名；
- 完整导航 calibration seeds：5121--5135，五类各 3 个场景；
- held-out seeds 5001--5010 保留且未消费；
- 历史 validation/calibration/probe seeds 全部列入 firewall。

预注册 evidence budget 为：

```text
6 readiness + 3 deterministic TTC component + 60 navigation = 69
```

60 个导航 identity 是 Fixed、5.0 s control、4.5 s candidate、4.0 s candidate
各 15 个场景的精确笛卡尔积。所有 readiness、component 和 navigation identity 的
attempt limit 均为 1。

## Readiness、coverage 与 hard gates

readiness 只对两个 winner-eligible candidate 执行相同的 3 个 fresh Dynamic 场景，
要求每个 candidate 得到 2 `OBSERVED_CONFLICT`、1 `NO_CONFLICT_IN_HORIZON`、
0 `TRACKER_INVALID`，六个 identity 全部完成。每个探针还要求连续稳定 10 次、
各消息流至少 20 条、transaction activated/valid/join-valid fraction 均不低于 0.95，
sequence mismatch、atomic input-join fault、backend transaction fault 和 unknown
transaction fault 均为 0。

readiness 通过后才允许执行确定性的 TTC 三状态 component gate；三种状态必须依次覆盖
`OBSERVED_CONFLICT`、`NO_CONFLICT_IN_HORIZON`、`TRACKER_INVALID`。component
gate 通过后才允许进入 60 个导航 episode。

导航 hard gates 保留 R4-R1 已通过边界：

- Fixed 至少 14/15 success，collision 为 0；
- candidate success 不低于 Fixed，collision 为 0，每个成功 episode 最小净空不低于
  0.25 m，typed transaction invalid 为 0；
- 每种方法的导航 TTC coverage 至少 2 conflict、1 clear、0 invalid；
- 每个 candidate 的两个 conflict episode 都必须实际出现 non-none Dynamic overlay；
- bounded join、Maneuver scan/truth clearance、contact、倒车、topology、centerline、
  switching/chatter、总时间和 family efficiency 门全部保持冻结值；
- evaluator truth 不得进入 runtime，禁止事后改 scene label、TTC horizon 或 gate。

## 停止与禁止重试

- 任一 readiness identity 失败：立即停止，不执行 component 或 navigation；
- TTC component gate 失败：立即停止，不执行 navigation；
- 任一 terminal evidence failure：保留该 identity 和失败 episode，立即停止整个阶段；
- terminal failure 后禁止 resume，任何失败 identity 禁止重试，禁止换 seed；
- Fixed collision 或少于 14 success：整个 split 无效；
- candidate collision、证据不完整或任一 hard gate 失败：该 candidate 不合格；
- 没有候选通过全部 hard gates：停止且保持 no winner；
- 禁止删除失败 episode、不利 seed，禁止扩展 69 单位预算或提前按性能选择候选。

## 已实现与测试

主要机器合同：

- `config/thesis_experiments/v2/v2_04g_r5_ttc_robustness_contract.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r5_preregistration.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r5_ttc_timing_candidates.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r5_scene_derivation.yaml`
- `experiments/manifests/v2/calibration/v2_04g_r5_ttc_readiness_scene_derivation.yaml`

离线工具：

- `src/tools/thesis_experiment/scripts/v2_04g_r5_candidate_materializer.py`
- `src/tools/thesis_experiment/scripts/validate_v2_04g_r5.py`
- `src/tools/thesis_experiment/tests/test_v2_04g_r5.py`

validator 使用 strict YAML duplicate-key 检查、workspace 路径边界和 39 个冻结资源 hash，
重新材料化全部候选并验证唯一行为 diff，重新从冻结源场景派生目标场景，并重新编译/
渲染 instance 与 SDF 做逐项比对。它还验证 seed firewall、6/3/60 精确 schedule、
69 单位预算、attempt=1、失败停止和全部未授权动作。contract 的 scope、readiness、
TTC coverage、完整 hard gates、budget/retry、execution order 和 stopping rules 使用
冻结 canonical 摘要校验；validator 与其测试文件自身也在资源 hash 闭包内。

测试结果：

```text
R5 定向测试：23 passed
全部 V2 离线回归：125 passed
dry-run audit：dry_run_audit_pass
ROS processes started by this stage：0
Gazebo processes started by this stage：0
episodes executed：0
```

## 当前未授权

- bounded 或 formal Gazebo calibration batch；
- readiness/component/navigation evidence 执行；
- winner freeze 或配置版本冻结；
- held-out 5001--5010 validation；
- V2-05、SAC 或任何训练；
- 实车连接、实车运动和实车 TEB 参数写入。

## 下一授权入口

审查本 handoff、预注册和 dry-run 审计后，如用户单独授权 bounded Gazebo calibration，
下一阶段只能：

1. 先重新运行 R5 validator，确认 preregistration hash 和全部冻结资源未漂移；
2. 实现最小 R5 execution/assessment wrapper，不修改 factor、候选、场景、seed、预算或
   hard gates；
3. 按固定顺序执行 readiness 6、TTC component 3、navigation 60；
4. 任一失败按上述规则永久停止并保留证据；
5. 完成 fail-closed assessment 后停止，不冻结 winner，不生成 held-out，不训练，
   不连接实车。

即使 bounded calibration 得到合格候选，winner freeze 仍需要之后的另一条独立用户指令。
