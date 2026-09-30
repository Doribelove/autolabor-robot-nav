# Arena 算法对比实验

本目录记录一个独立的 Arena 场景迁移研究。它使用 Arena 的 hospital 世界资产，但继续使用本项目的 Autolabor M2 Ackermann 机器人与冻结算法，不调用 Arena 的 `start_arena_gazebo.launch` 或 `task_generator`。

## 为什么单独建阶段

现有 PCT-SCOPE-RL 正式 Gazebo 证据来自六类简化模板，并不包含 Arena 丰富室内几何。Arena 默认启动链会原地改写 world，原生机器人也不是 M2；直接运行既有数据风险，也会破坏算法公平性。因此 Q1 使用以下隔离链：

```text
只读 hospital.world + SHA 校验
  → episode 专属 SDF 快照
  → 移除快照中的原生 actor
  → 可选注入一个受控横穿 actor
  → 同一 M2 / LaserScan / Odom / TF
  → PCT / 解析先验 / Actual TEB / DWA
  → 失败计入的独立报告
```

冻结协议见 [arena_pct_q1_contract.yaml](../../../config/thesis_experiments/arena/arena_pct_q1_contract.yaml)。

## Q1 范围

- 世界：Arena hospital 单层世界。
- 路线：一条约 8.2 m 的宽走廊局部规划路线。
- 任务：静态通行、单个受控圆柱代理的反应式横穿/让行交互。
- 方法：PCT-SCOPE-RL、同候选解析先验、Actual TEB 0.8.4、DWA。
- 规模：3 个配对块、12 个 episode、每个身份仅 attempt=1。
- 统计地位：资格/探索性，不进行显著性推断，不授权“普遍优于 TEB/DWA”的结论。
- 比较类型：完整导航系统栈比较。PCT/解析方法用同一 PGM 做候选硬净空验证，TEB/DWA 通过静态全局代价地图和 Navfn 使用它；不能写成四方法内部信息语义完全相同。
- 任务边界：路线是 hospital 大场景中的一条宽直走廊，不代表转弯、复杂静态绕行或真实人群导航。
- 接触证据：运行时检查 bumper 发布者、消息类型、连接和连续心跳，并拒绝任何非零接触状态；但本 Q1 没有冻结的碰撞敏感性正控，因此“零接触状态样本”只作诊断，不能单独写成零碰撞或形式安全证明。

## 证据边界

Q1 若完整，只能证明 Arena 资产到四方法的共享传感器/执行接口已经打通，并提供一组系统级描述性效果。结果必须并列报告原始到达、安全规范合格到达和失败计入时间，不能用净空违规封顶值单独制造性能优势。资源清单是“精确选择的执行资源名册”，不是整个操作系统、Python 和动态链接库的完全传递闭包。要形成“在复杂 Arena 环境中具有稳定先进性”的论文证据，仍需在多个冻结世界、任务类型和新鲜种子上预注册正式矩阵，并保留所有超时、接触状态、净空失败和基础设施失败。

Q1 已完成，结果与论文图说明见 [ARENA_Q1_RESULTS.md](ARENA_Q1_RESULTS.md)。机器生成结果位于：

```text
artifacts/arena_pct_teb_dwa_qualification/q1/analysis/
```

终局状态为 12/12 证据链与平台完整性通过、12/12 原始到达。PCT-SCOPE-RL/解析先验/Actual TEB/DWA 的安全规范合格数分别为 3/3、3/3、1/3、1/3。这是 hospital 单路线的 n=3 描述性资格结果；`formal_result=false`，不授权一般 Arena 先进性结论。
