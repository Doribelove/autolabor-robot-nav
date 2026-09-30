# N04 首个 analytic 静态失败：离线机制诊断

该回合在 22 s 超时，采集完整性与已声明工程安全检查通过。最有直接证据的后继假设是：**backup-only 资格与进度优先的软 full-preview 评分允许朝不利状态继续前进，随后上墙附近的延迟制动支持坍缩，现有八个前向/停止动作难以恢复。** 这是一条可证伪机制假设，不是单条轨迹的因果归因。未改源码、裕量、结果或种子；仅使用 root 已提取 JSON 和六个姿态的静态包络检查，没有 bag/完整规划器回放或新仿真。

`decision_table.csv` 与 `decision_rows.json` 关联全部 88 次候选决策及 439 条控制记录。88/88 analytic 选择可从已记录 feature 重构，动态占用计数全部为 0。41 次候选的 backup mask 全空。19 次所选移动候选的 3 s full-preview clearance 为负，16 次在该决策的第一条控制中执行了移动；其中 3 次同时存在 backup 合格且 full-preview 非负的移动替代。15 次 deadline zero、206 次 backup_revalidation_failed、111 次 backup_stop_requested 均保留。计数不是独立样本、时间比例或效果分解。

| 实际源时间 | 观察 | 解释边界 |
|---|---|---|
| 9.482 s | 选择 route_v120，full-preview −0.02086 m；有其他非负移动前瞻，首次控制却因 deadline 输出零 | 能证明 rank 偏好，不能把本次选择算作实际运动原因 |
| 11.008 s | 首次八 mask 全空，stop backup −0.03371 m；当前位置静态包络裕量 +0.29219 m，即时 stop preview +0.23705 m | 延迟 hold/brake 与即时停止 preview 不是同一个轨迹；这不是当前位置已碰撞 |
| 11.512 s | 实际 yaw 0.8114 rad，y=1.4031 m；上墙成为最近静态边界 | 已偏离低曲率的路线对齐姿态 |
| 12.516 s | 速度约 0.0836 m/s；backup 全合格，仍选择 full-preview −0.15345 m 的 route_v120 | 短时能停不保证远期导向可行 |
| 29.352 s | base=(−0.62166,1.71724)，相对平台路线横向误差 0.54224 m、朝向误差 44.88° | 最后观测状态，非目标；共享 coordinator 的超时权威为 29.460 s |

代码的语义清楚：`route_candidate_core.py:199` 用 0.7 m lookahead 的纯追踪转角；三秒 preview 是持续该动作，而 backup 先保持实际速度/转角 0.35 s，再执行该动作 0.05 s，然后制动回正。`build:265` 只用 backup clearance≥0 构造 hard mask，full-preview 仅以 `1.5*clip(clearance,-1,2)` 进入 prior；route 进度增益可以超过负前瞻罚项。每 20 Hz `command:297` 再从实际状态重验 backup。M08 的新 core 继承同一 `step/rollout`，保留同一 prior 与 backup-only 资格，因此增加轨迹记忆本身不会修复此静态支持问题。

共同 A* guide 的平台为 y=1.175 m、x∈[−0.825,1.075]，入口航向约 0.35445 rad，折点未做 Ackermann 可达性资格。其 1.0 m 中心参考净空不是实际闭环跟踪保证，最大转角对应最小转弯半径约 1.20 m。实车参考点、全组件包络及转向/动力学滞后仍可能影响追踪；这次没有独立拆分这些贡献。

六姿态检查使用真实 URDF 构造的九组件、全转角与 tilt 外包 16-plane envelope。最后包络相对上墙栅格的端点裕量为 −0.000405 m；相同包络相对实际物理墙为 +0.018533 m，二者差值来自栅格边界向自由区扩展约 0.018938 m。最后 backup 更负（stop −0.001308 m、其他约 −0.0028 m），还含预测运动及段间扣减。这些量与实际 articulated collision、2 ms 工程 sweep assessment 不同；**既不构成物理碰撞证据，也不支持缩小 0.33 m 裕量或放宽数值门。** 把机器人放到最近路线点并沿路线航向对齐时包络裕量为 +0.2511 m，但这只是几何对照，并非可执行恢复路径。R2 9.84 s 与实际 TEB 8.116 s 成功也只表明场景可通行，不能证明当前八动作必然可达。

建议唯一后继因素为 **analytic selection eligibility**：保持原 observation `action_mask`（backup-valid）和全部 candidate/prior 数值；另记 `selection_eligibility[a] = backup_mask[a] && full_preview_clearance[a]≥0`（仅 a>0），停止动作沿用原资格和 20 Hz 重验。仍以原 prior 在新资格内排序；无合格动作则沿用现有零输出停止。它改变 analytic selector，不改真正 backup mask、不混入 M08 tracking，也不冒充新学习模型。需要新方法/语义标识，旧 frozen core 保持原样。

新鲜配对开发仿真才能回答该因素是否减少陷入支持空集。若它更早停住、仍超时，或在非负前瞻下仍发生上墙追踪偏离，便反驳“只修 soft-preview 选择便足够”的解释，并指向 guide/controller/action support。不能由日志中三次替代推断成功；所有失败、时限与完整性门保持不变。当前只提出因素，未实现或接入运行。
