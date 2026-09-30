# 方法、公平性与实验设计

## 四种方法

- PCT-SCOPE-RL：部署 actor 读取 LaserScan、里程计/局部目标、候选特征、解析先验和硬 mask，
  输出 8 个 Ackermann 可行候选之一。仿真特权状态只用于 critic/PCT 训练标签。
- 解析先验：与 PCT 共用候选生成、候选条件预测、硬验证、切换平滑、tracker 和控制边界，只用
  冻结解析得分选候选。
- Actual TEB：`teb_local_planner/TebLocalPlannerROS` 0.8.4，development 选择 v0。
- DWA：ROS DWA 本地规划器，development 选择 v0。

## 公平性

四方法共享 M2 Ackermann 模型、1.20 m/s 速度上限、0.60 m/s² 加速、0.90 m/s² 减速、15 Hz
360° LaserScan、warmup gate、目标、22 s 时限和 `/cmd_vel_navigation` 边界。传统规划器没有被
降速：TEB/DWA 分别评估多种更保守距离、膨胀、同伦和 horizon 变体，最终按六场景失效计入均时
选择完整评估中最强的 v0。全部无效到达、超时、负保守净空、接触和系统失败统一记 22 s。

PCT development 从 V6 seed9501 初始化，用 seed9701 在对齐分布训练约 40,008 步；训练 575 个
episode、571 个目标、contact=0、mask violation=0。最终 actor SHA 为
`deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8`。

## 场景与隔离

六个冻结模板为 static avoidance、single crossing、head-on yield、sequential interaction、
partially occluded 和 width change。development、pilot、R1 formal、R2 development、R2 pilot 与
R2 formal 使用互斥身份。R1 formal seeds 为 197001–197160；R2 formal seeds 为 198001–198200。

## 预注册主门

每个基线比较必须同时满足：失败计入时间点改善至少 8%；双侧 97.5% paired-block bootstrap
下界严格大于 0；完成率差不低于 -5 pp；PCT contact 不高于基线；PCT mask violation=0；统计
有限。任何一个条件失败即不得声称 superiority。

## R2 单因素

R1 阴性封存后，只在新 development 数据中把左右低速空间候选目标速度由 0.4 调到 0.6 m/s。
PPO actor、解析公式、其他候选、硬安全门、TEB、DWA 和统计定义均不变。12 个 development 配对
块显示均时改善 1.2000 s，随后用新 pilot 冻结 200-block R2；没有从 R1/R2 正式结果继续调参。
