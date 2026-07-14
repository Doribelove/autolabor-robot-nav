# FAM-TEB 论文项目周报

更新时间：2026-07-14  
工作空间：`/home/robot/robot_ws_base_rl`  
当前分支：`base_on_rl`

## 一、本周总体结论

项目已经从“固定参数 TEB + 单一强化学习调参设想”推进到“场景感知、多模式、风险约束、可事务执行的 V2 系统”。

当前最重要的结论是：

- V1/T00–T12 的历史工作已冻结并与 V2 隔离。
- FAM-TEB V2-04C 已完成 Anchor Bank 标定与冻结。
- V2-04E–E4 已使用独立 calibration seeds 修复规则监督器，解决了 Cruise 被 Static 吸收和 Maneuver 不触发问题。
- V2-04F 使用全新 held-out validation seeds 完成 30 个三方法配对 episode。
- Fixed TEB、Balanced Anchor、Rule Multi-Anchor 均为 10/10 成功、0 contact collision，成功率不退化已证明。
- Rule 已实际覆盖 Cruise、Static Dense、Corridor、Maneuver 等 Anchor。
- 但综合性能门仍未通过：净空、抖动、Dynamic TTC 覆盖和相对 Fixed 的导航时间仍存在问题。
- 当前不授权 V2-05、SAC 训练、实车闭环或实车 TEB 参数写入。

## 二、T00–T12 历史工作进度

| 阶段 | 工作内容 | 当前结果 |
|---|---|---|
| T00 | 环境、版本、ROS、Git、实车/仿真边界盘点 | 已完成，建立论文工作空间与稳定实车工作空间隔离 |
| T01 | RL/TEB 软件包骨架、实验合同、配置校验 | 已完成，配置和运行边界机器可读化 |
| T02 | M2 Gazebo 底盘、Ackermann 接口、动力学基础 | 已完成，固定 TEB 和底盘回归通过；仍属于 simulation candidate |
| T03 | TEB 参数客户端、dynamic reconfigure、ack/readback、快照恢复 | 已完成，9 参数事务和恢复链路通过验收 |
| T04 | 状态、奖励、episode、ROS 时间和 activation timeout | 已完成，状态/奖励/终止语义冻结 |
| T05 | 参数投影、安全盾、回退和 emergency 状态机 | 已完成，NaN/Inf、边界、变化率、原子回退通过测试 |
| T06 | 日志、CSV、manifest、checksum、结果校验器 | 已完成，实验数据可追溯 |
| T07 | TEB 敏感性标定和 A_TEB 映射 | 已完成，冻结 Gazebo 标定映射 |
| T08 | TEB-Default、TEB-Tuned、Rule-TEB、Fixed-DWA 基线 | 已完成，作为 validation pilot，不作为正式最终结论 |
| T09 | Semantic-Eta SAC 环境与 smoke training | 已完成 smoke，证明训练管线可运行，不代表学习有效 |
| T10 | Direct-Theta SAC 与 Semantic-Eta 配对 smoke | 已完成 smoke，动作空间和执行器链路通过 |
| T11 | 多 seed SAC 研究矩阵与消融 | 已完成缩减矩阵；原 5-seed 计划因预算约束缩减为 4-seed 主矩阵，学习门未通过 |
| T12 | 安全修复、Residual pilot、闭环复验、离线 projection 诊断 | 已完成；成功率和安全链路改善，但 Residual SAC 学习门失败，不扩预算 |

### T11/T12 当前边界

T11/T12 已证明安全投影、回退和状态机可以运行，但不能宣称 SAC 已经学到稳定策略。最新 Residual 诊断显示训练 projection rate 约为 65.1%/69.5%，主要来源是 Ackermann 耦合和安全干预后的回指 anchor。根据冻结边界，不再重启旧 pilot，也不扩充训练预算。

## 三、V2 系统建设进度

| 阶段 | 主要建设内容 | 结果 |
|---|---|---|
| V2-00 | V1 隔离、配置根目录、manifest 和 artifact 根目录 | 已完成 |
| V2-01 | architecture contract、parameter registry、mode thresholds、ROS 消息骨架 | 已完成 |
| V2-02 | 可信仿真动力学、制动/时延模型、五类基础场景 | 已完成 |
| V2-03 | `nav_world_model`、动态目标跟踪/预测、健康状态、无标签规则监督器 | 已完成 |
| V2-04B | Anchor Bank、类型化 profile、动作解码、平滑参数事务、simulation-only typed TEB transaction | 已完成 |
| V2-04C | Dynamic TTC qualification、多参数 refinement、Anchor 冻结 | 已完成：54 候选、180 个 calibration episode、Anchor Bank 冻结 |
| V2-04D | Fixed / Balanced / Rule 三方法首次配对 validation | 30/30 成功，但 Rule 时间全面退化，Maneuver 未触发 |
| V2-04E–E4 | calibration-only 监督器修复 | 45 个 calibration episode，最终候选通过全部 calibration hard gate |
| V2-04F | 新 held-out seeds 三方法配对 | 30/30 成功；成功率不退化通过，但综合 hard gate 失败 |

## 四、当前 V2 系统架构

系统已经固定为三层架构，而不是单一 SAC 直接输出 9 个 TEB 参数。

```text
传感器 / 里程计 / 局部规划路径
              │
              ▼
┌─────────────────────────────────────────┐
│ 第一层：Local World Model                │
│ - LaserScan 合同与全向覆盖               │
│ - 局部几何 clearance / density / corridor│
│ - 动态目标跟踪、速度估计、短时预测         │
│ - TTC、tracker health、stale/fault         │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│ 第二层：Scene / Risk Context Supervisor   │
│ - BALANCED / CRUISE / STATIC_DENSE        │
│ - CORRIDOR / MANEUVER                     │
│ - Dynamic Overlay: CROSSING / HEAD_ON ... │
│ - 无场景标签、无 manifest、无 Gazebo truth  │
│ - entry confirmation、dwell、exit hysteresis│
│ - challenger score margin                 │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│ 第三层：Anchor / Typed TEB Transaction    │
│ - Frozen Anchor Bank                      │
│ - geometry Anchor + dynamic overlay       │
│ - commanded → feasible → safe → executed   │
│ - Ack/readback、变化率限制、原子回退        │
│ - simulation-only TEB dynamic reconfigure  │
└─────────────────────────────────────────┘
              │
              ▼
      TEB local planner / velocity execution
```

### 当前已经实现的关键机制

- 静态障碍：地图/costmap 与较长前视几何共同决定绕行，不允许单个静止 track 直接吞掉 Cruise。
- 动态障碍：tracker 输出位置、速度、运动类别和预测轨迹；规则层使用 TTC 和相对运动 overlay。
- 狭窄通道：根据两侧 clearance、通道宽度和并行置信度进入 Corridor。
- 死角机动：使用前方受限、两侧围合、后方可退的 pocket geometry 触发 Maneuver。
- 模式过渡：进入确认、最小驻留、退出确认、score margin 和平滑参数事务共同抑制跳变。
- 安全边界：supervisor 不发布速度，不读取场景标签，不读取 validation manifest，不读取 checkpoint。

## 五、V2-04F 配对结果

| 方法 | 成功率 | Contact collision | 全部场景最小净空 | 总导航时间 |
|---|---:|---:|---:|---:|
| Fixed TEB | 10/10 | 0 | 0.254 m | 203.0 s |
| Balanced Anchor | 10/10 | 0 | 0.000 m | 293.4 s |
| Rule Multi-Anchor | 10/10 | 0 | 0.429 m | 268.4 s |

Rule 的机制结果：

- Cruise 场景 Static fraction 为 0，原有 Cruise/Static 混淆已消除。
- Maneuver 场景实际激活 `anchor_maneuver_forward`，最低 Maneuver fraction 为 18.5%。
- 五种 geometry Anchor 均至少激活一次。
- Rule 最大 Anchor switch count 为 4，超过预注册上限 3。
- Dynamic 场景有限 TTC 覆盖为 0/6，低于预注册 80%。
- Balanced 有一个 Dynamic episode 的最小扫描净空为 0.000 m，低于 0.25 m 安全门。
- Rule 相对 Fixed 的导航时间在五类场景均增加：Cruise +3.9%、Dynamic +4.0%、Static Dense +73.3%、Corridor +60.7%、Maneuver +51.2%。

因此论文当前可以安全地表述为：

> 场景感知规则监督器在全新 held-out 场景上保持了基线成功率，并修复了 Cruise/Static 模式混淆和 Maneuver 不触发问题；但当前尚未证明整体导航效率和综合性能优于 Fixed TEB。

不能表述为“V2 已经全面优于基线”或“已授权 SAC 训练”。

## 六、接下来的工作安排

### P0：冻结当前结果和边界

1. 保持 V2-04E4 supervisor 和 V2-04C Anchor Bank 不变。
2. 保持 V2-04F validation 只读，不用 4801–4810 调阈值。
3. 保持 `runtime_ready=false`、`training_allowed=false`、`real_vehicle_use_forbidden=true`。
4. 在论文和周报中分别报告“成功率不退化”和“综合性能门失败”，不使用单一加权分数掩盖失败。

### P1：建立新的 calibration-only 问题修复阶段

下一阶段如继续，必须重新预注册新 calibration split，不能把 V2-04F validation 转作 calibration。重点应分成三个互不混淆的问题：

1. Dynamic TTC 可观察性：调整横穿 agent 时序和速度，使三种方法都能形成 `OBSERVED_CONFLICT`，同时保留 `NO_CONFLICT_IN_HORIZON` 和 `TRACKER_INVALID` 三态。
2. 安全净空：调查 Balanced dynamic episode 的 0.0 m 扫描 footprint clearance 是否代表真实近接风险、传感器几何误差或 evaluator 语义问题；在新 calibration seeds 上修复并重新验证。
3. 时间效率：分析 Rule 在 Static Dense、Corridor、Maneuver 中的时间代价，重点检查 Anchor 参数是否过于保守、切换是否导致速度降级，以及是否需要 topology lock、corridor centerline 和离散机动规划器。

### P2：仅在新 calibration gates 通过后

- 重新冻结规则 supervisor 或新的机制模块。
- 使用第三组全新的 held-out validation seeds。
- 仍然先检查成功率、碰撞、净空、接口和 TTC，再比较时间、路径稳定性与机动能力。

### P3：SAC 和实车

只有规则多 Anchor 基线在独立 held-out validation 上通过成功率/安全和性能门，才讨论 SAC 的单因素训练。实车必须另行完成动力学、制动、时延、传感器和参数范围标定，并获得现场逐次批准。

## 七、当前明确禁止事项

- 不得直接开始 SAC 训练。
- 不得把 V2-04F validation 结果用于调参。
- 不得修改已冻结 Anchor Bank 作为 validation 后补救。
- 不得启动真实车辆、连接 `m2_driver`、写实车 TEB 参数。
- 不得重跑或扩大 T11/T12 历史 pilot。
- 不得把当前 V2-04F 结果写成“综合性能提升已证明”。

## 八、证据入口

- 最新交接：[CURRENT_V2_04F_HANDOFF.md](CURRENT_V2_04F_HANDOFF.md)
- V2-04F 合同：[v2_04f_fresh_paired_validation_contract.yaml](../../config/thesis_experiments/v2/v2_04f_fresh_paired_validation_contract.yaml)
- V2-04F 预注册：[v2_04f_preregistration.yaml](../../experiments/manifests/v2/validation/v2_04f_preregistration.yaml)
- V2-04F 评估：[v2_04f_paired_assessment.yaml](../../artifacts/v2/validation/v2_04f/v2_04f_paired_assessment.yaml)
- V2-04F 人读报告：[V2_04F_PAIRED_VALIDATION_REPORT.md](../../artifacts/v2/validation/v2_04f/V2_04F_PAIRED_VALIDATION_REPORT.md)
- 冻结 supervisor：[v2_04e4_rule_supervisor_frozen.yaml](../../src/application/teb_mode_manager/config/v2_04e4_rule_supervisor_frozen.yaml)
- Anchor Bank：[v2_04c_anchor_bank_frozen.yaml](../../src/application/teb_mode_manager/config/v2_04c_anchor_bank_frozen.yaml)

