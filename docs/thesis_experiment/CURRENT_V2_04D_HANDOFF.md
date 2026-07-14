# FAM-TEB V2-04D Handoff

更新时间：2026-07-14 CST

## 当前结论

V2-04C 与 V2-04D 已全部执行完毕。V2-04C 在 Dynamic 跟踪轨迹接入 TEB 和动态目标/静态
costmap 点空间融合修复后通过 TTC qualification；随后完成 54 个多参数候选、180/180 个
calibration episode，并冻结 6-Anchor Bank。冻结文件为
`src/application/teb_mode_manager/config/v2_04c_anchor_bank_frozen.yaml`，SHA256 为
`b1365f2b14e4a77f4e715bc46369fa4277e9adbb8522190153f7abd2ce635b9d`。

V2-04D 完成 Fixed TEB、Single Balanced Anchor、Rule Multi-Anchor 三方法在 10 个 validation
场景上的 30/30 配对 episode。三方法均为 10/10 成功、0 碰撞、0 持久接口失败，成功率
不退化门通过；所有成功 episode 最小净空均不低于预注册的 0.25 m。Balanced 只激活
`anchor_balanced`；Rule 实际激活 Balanced、Cruise、Static Dense、Corridor 共 4 个 Anchor，
因此 typed transaction 与多 Anchor 机制门也通过。

但是，性能有效性没有被证明，当前不授权进入 V2-05。Rule 在五类场景的配对中位导航时间
相对 Fixed 全部退化：Cruise +62.5%、Dynamic +96.6%、Static Dense +51.4%、Corridor
+66.4%、Maneuver +54.1%。Rule 从未激活 Maneuver Anchor，Cruise/Dynamic/Static 经常在
Static Dense 与 Balanced 间切换；Rule 共发生 31 次几何 Anchor 切换。跨方法 Dynamic
`OBSERVED_CONFLICT` 只有 2/6（33.3%），低于预注册 80% 阈值；`TRACKER_INVALID` 为 0，
说明问题是交互时序/行为覆盖而不是 tracker 失效。

所以准确结论是：**安全和成功率不退化已证明；效率、模式识别和综合性能提升未证明。**
`runtime_ready=false`，SAC、任何学习和实车闭环继续不授权。

## V2-04C 完成证据

- Dynamic R4 qualification：5/5 成功、5/5 `OBSERVED_CONFLICT`、0 碰撞、健康覆盖通过；
- refinement：54 个候选，180/180 有效、179 SUCCESS、1 ABORTED、0 碰撞、0 持久接口失败；
- frozen Anchor Bank：
  `src/application/teb_mode_manager/config/v2_04c_anchor_bank_frozen.yaml`；
- freeze report：`artifacts/v2/calibration/v2_04c/v2_04c_freeze_report.yaml`，SHA256
  `59378fecb42471115d496815c6f6a30468e2e2db09de121d200d65fa5a9d4f14`；
- Dynamic bridge：`nav_world_model/tracks` 经 odom 变换发布到
  `/move_base/TebLocalPlannerROS/obstacles`；TEB 只在非零速度 tracked object 周边移除重复的
  静态 costmap 点，其他 costmap 障碍继续保留。

冻结 winner：

| Anchor | V2-04C winner |
|---|---|
| Balanced | `anchor_balanced-rc00-incumbent` |
| Cruise | `anchor_cruise-rc00-incumbent` |
| Static Dense | `anchor_static_dense-rc03-hfff-mpmp` |
| Corridor | `anchor_corridor-rc05-hfff-pmmp` |
| Maneuver Forward | `anchor_maneuver_forward-rc01-hfff-mmmm` |
| Maneuver Reverse | `anchor_maneuver_reverse-rc00-incumbent` |

## V2-04D 方法语义

- `fixed_teb`：旧固定 TEB 参数；世界模型只用于 evaluator，不向 TEB 发布 tracked obstacle，
  无 supervisor、无 typed transaction；
- `balanced_anchor`：冻结 Balanced Anchor + 动态跟踪桥 + 无标签 supervisor 的 dynamic
  overlay + simulation typed transaction；几何模式强制钳制为 `BALANCED`；
- `rule_multi_anchor`：与 Balanced 相同的感知和 transaction 链，但允许无标签 supervisor
  选择几何 Anchor；
- 运行节点不接收 manifest、family 或 scene 标签；只有实验管理器和 evaluator 读取 validation
  manifest；
- 所有 typed 写入都要求 `/use_sim_time`、Gazebo marker、有效 `/clock`、显式 simulation opt-in
  与固定 TEB namespace；每个 episode 独立启动并在退出时恢复启动快照。

## V2-04D 结果

| Method | Success | Collision | 全部场景最小净空 | 总导航时间 | 实际 Anchor |
|---|---:|---:|---:|---:|---|
| Fixed TEB | 10/10 | 0 | 0.254 m | 209.1 s | n/a |
| Balanced Anchor | 10/10 | 0 | 0.415 m | 277.8 s | Balanced |
| Rule Multi-Anchor | 10/10 | 0 | 0.433 m | 344.0 s | Balanced/Cruise/Static/Corridor |

Rule 相对 Fixed 的可确认收益主要是 Static 路径 -2.8%、Maneuver 路径 -7.7%，以及 Maneuver
最小净空配对中位数 +0.182 m；代价是五类时间全部明显增加，且 Maneuver 并未实际使用
Maneuver Anchor。Corridor 横向误差略有改善，但航向振荡更大。依据预注册的“不使用事后
单一加权分数”规则，这些局部收益不能覆盖系统性时间退化。

## 机器入口与哈希

- V2-04D 合同：`config/thesis_experiments/v2/v2_04d_paired_validation_contract.yaml`；
- 预注册：`experiments/manifests/v2/validation/v2_04d_preregistration.yaml`；
- validation 场景：`experiments/manifests/v2/validation/v2_04d_paired_validation_scenes.yaml`；
- progress：`artifacts/v2/validation/v2_04d/v2_04d_paired_progress.yaml`，SHA256
  `6996c31dd464cc158fb50d874b605b1809ece444096f81eed1f9a58bfb11a7e6`；
- assessment：`artifacts/v2/validation/v2_04d/v2_04d_paired_assessment.yaml`，SHA256
  `84a3ee3bb47717ef3ab57df8e1cd84ddd573c073d7aec32d616c9d5a43a3aa88`；
- 人读报告：`artifacts/v2/validation/v2_04d/V2_04D_PAIRED_VALIDATION_REPORT.md`，SHA256
  `49a3446ee9d6f9a9e67f2ee097929039d577f77090d1843857f830ee924d2db2`。

## 构建与边界

- 全工作空间 `catkin_make` 通过，共 73 个包；
- V2-04B/C/D 相关 16 个 nosetest 通过；
- 30 个 validation episode 期间没有加载 SAC/checkpoint，没有训练，没有连接实车 ROS
  master，没有启动底盘驱动，也没有修改 `/home/robot/robot_ws`；
- frozen Anchor Bank 未因 validation 结果回改；
- `formal_result=false`、`runtime_ready=false`、`training_started=false`、实车禁止。

## 下一入口

不得直接进入 V2-05，也不得在已经使用过的 V2-04D validation seeds 上调阈值。下一步应新建
calibration-only 的规则 supervisor 修复阶段：

1. 预注册新的 calibration seeds、模式识别混淆矩阵、每类驻留率、切换次数和 Dynamic TTC
   覆盖门槛；Anchor Bank 保持冻结；
2. 修复 Cruise 被 Static Dense 吸收、Maneuver 不触发、Static/Balanced 抖动三个问题；重点审查
   obstacle-density 归一化、dead-end/后向覆盖语义、置信度与 dwell/hysteresis；
3. Dynamic 场景时序改为对三方法均有可观察交互，且保持
   `OBSERVED_CONFLICT/NO_CONFLICT_IN_HORIZON/TRACKER_INVALID` 三态；
4. calibration gate 通过后，用全新的 held-out validation seeds 重新做三方法配对；不得复用
   V2-04D validation 做选择；
5. 只有新配对同时保持成功率/安全，并消除五类系统性时间退化、实际触发 Maneuver 模式后，
   才可授权 V2-05 专用机制。SAC 与实车仍需独立授权。
