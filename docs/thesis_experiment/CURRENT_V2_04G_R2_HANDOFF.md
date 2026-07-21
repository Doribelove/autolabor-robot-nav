# CURRENT V2-04G-R2 HANDOFF

日期：2026-07-14

## 结论

V2-04G-R2 已建立为独立的 simulation-only、calibration-only 机制修复阶段，
但在预注册的 activation-readiness 第 3/6 次重复处 fail-closed 停止。没有启动
TTC 组件探针或任何导航 episode，没有执行候选排名、机制冻结或 held-out validation。

权威机器可读边界是：
`artifacts/v2/calibration/v2_04g_r2/v2_04g_r2_stop_report.yaml`，SHA256 为
`0a1725f045c847a27dff1e8c7835672e292226d1fe77a308deaef1a9eef40987`。

## 已实现的系统修改

- R1 join 基础设施按文件和数值双重冻结：32 条缓存、最大到达年龄 1.0 s、
  最大序号差 2、最大时间戳差 0.45 s，禁止未来序号和未来时间戳。
- 新增幂等 typed TEB 事务后端：仅当全部 20 个类型化参数等于最后一次 ack/readback
  时合并重复 dynamic_reconfigure 写；commanded/feasible/safe/executed 四阶段语义、
  action trace、启动快照恢复和改变参数时的原子 request/ack/readback 均保留。
- 修复 Maneuver 倒车判据：不再使用在现有 Maneuver 几何中不可达的 1.90 rad 航向阈值，
  仍由后向覆盖、后方净空和前方净空共同约束。
- 建立 Static/Corridor/Maneuver 的 balanced/aggressive 目标 Anchor 与残差候选；
  `r2_control_g2` 保留 R1 g2 数值作为诊断对照。
- 新建 seeds 4951--4965 的 15 个 calibration 场景和 seeds 4971--4976 的
  readiness-only 计划；5001--5010 未生成、未消费，继续保留给未来 held-out validation。

## 预注册前设计证据（不得用于 R2 排名）

只使用已消费的 R1 calibration 场景 4928/4931/4933 做机制设计。最终 balanced
设计探针相对同场景 Fixed 的结果为：

| 族 | Fixed 时间 | balanced 时间 | 相对变化 | 倒车样本 |
| --- | ---: | ---: | ---: | ---: |
| Static | 19.9 s | 22.0 s | +10.55% | 0 |
| Corridor | 10.6 s | 11.2 s | +5.66% | 0 |
| Maneuver | 24.4 s | 21.1 s | -13.52% | 51 |

这证明候选设计同时具备倒车可观察性和三目标族 15% 时间上限的可行性，但它不是新
calibration 结果。完整排除边界位于
`artifacts/v2/calibration/v2_04g_r2/v2_04g_r2_design_probe_summary.yaml`。

## Readiness 停止证据

| 重复 | profile / seed | 激活率 | 事务有效率 | join 有效率 | fault | 结论 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 1 | balanced / 4971 | 1.0000 | 1.0000 | 1.0000 | 0 | PASS |
| 2 | balanced / 4972 | 1.0000 | 1.0000 | 1.0000 | 0 | PASS |
| 3 | balanced / 4973 | 0.9667 | 0.9667 | 1.0000 | 1 | FAIL |

失败重复的 30/30 join 均为 `EXACT_SEQUENCE_JOIN`，所以当前证据不支持把失败归因于
已冻结 join。探针只累计了 `fault_reason` 的布尔次数，没有保存原因域，因此也不能严谨地
断言它是后端 request/ack/readback 故障，还是一次预期的 fail-closed context hold。
分类固定为 `UNCLASSIFIED_TRANSACTION_FAULT_WITH_VALID_EXACT_JOIN`。

## 验收状态

- 新增合同、种子防火墙、候选物化和幂等事务测试：8/8 通过。
- `teb_mode_manager`、`thesis_experiment`、`m2_gazebo`、`nav_world_model` 定向
  `catkin_make`：通过。
- Readiness：2 pass、1 fail、3 未执行。
- TTC 三态：未执行，因为执行顺序规定 readiness 失败后停止。
- 导航：0/60；没有打开 fresh navigation evidence。
- `runtime_ready=false`、`formal_result=false`。

## 当前边界

- 不得重跑 V2-04G-R2 并覆盖本次失败。
- 不得用设计探针或 4971--4973 进行候选排名。
- 不得冻结任何 R2 候选，不得生成 held-out validation。
- 不得消费 seeds 5001--5010。
- V2-05、SAC、实车闭环和实车 TEB 参数写入仍未授权。

## 下一阶段要求

若继续，应新建独立的 calibration-only readiness fault-taxonomy 修复阶段，保持 join 和
R2 候选数值冻结，并使用全新 seeds。唯一接口修复应包括：

1. 在每个 readiness report 中保存 `fault_reason` 计数和状态上下文。
2. 分离预期 fail-closed context hold 与真正的 backend request/ack/readback fault。
3. 测量前要求连续稳定 readiness 窗口，而不是单条有效消息。
4. 先通过新的确定性 readiness，再运行 TTC 和完整 60-episode calibration。

该后续 readiness-only 修复已由 V2-04G-R2-R1 完成，6/6 通过。继续工作时以
`docs/thesis_experiment/CURRENT_V2_04G_R2_R1_HANDOFF.md` 为最新入口；不得回写或
覆盖本文件记录的 R2 失败证据。
