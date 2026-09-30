# V3-SCOPE V3-02 共同决策状态预测器终态交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`v3_02_c2_i1_pure_offline_integrity_repair_pass`

## 1. 最终结论

V3-02 已补齐规划器接入所需的 candidate atomicity：四个候选在
`t=0.00--0.50 s` 共享完全相同的 ego 前缀，只在观测/决策时刻
`t=0.50 s` 后分支。C1-R1 冻结的 complete-quadratic predictor 随后在全新 C2
seeds 6261--6263 上一次性完成 36/36 episode，并通过所有预注册的共同前缀、
覆盖率、宽度、误差、候选敏感性和物理边界门。

该结论只覆盖当前 parameterized-crossing 场景族。它使 V3-03 的离线 hard
verifier / analytic ranker 设计具备一致的输入语义，但不证明导航安全、系统性能、
跨场景泛化、部署或实车可用。

## 2. 为什么需要共同前缀修复

旧 T2 有效证明了“给定一条从起点开始就不同的 ego 轨迹时”的有限预测能力，但
预接入审查发现候选在 `t=0` 已分叉，而预测观测发生在 `t=0.50 s`。不同候选因此
可能对应不同 actor history，不能代表规划器从同一个在线状态比较多个未来。

保留旧 T2 正式结果及其有限 claim，但禁止直接用旧 T2 模型接入规划器。修复链为：

- C1：固定共同前缀并收集 72 train + 36 calibration episode；
- C1 原 7-feature basis 的平均包络宽度为 4.5902 m，超过 4.50 m，校准失败；
- C1-R1 只把 feature basis 改为完整二次 10-feature basis，所有门槛、数据、
  ridge、conformal score 和物理 clip 不变；校准通过；
- C2：冻结 C1-R1 模型后，以 fresh seeds 6261--6263 执行唯一一次 36-unit
  held-out test。

## 3. C2 正式结果

- episode：36/36，prediction points：144，attempt=1，retry=0；
- marginal coverage：0.9930556（门 0.90）；
- per-horizon coverage：1.0 / 1.0 / 1.0 / 0.9722222（门 0.80）；
- mean full envelope width：3.9098074 m（门 4.50 m）；
- actual-mode MAE：0.1917888 m（门 0.50 m）；
- minimum candidate sensitivity：0.2387947 m（门 0.20 m）；
- physical coordinate violation：0；
- 最大共同前缀 actor coordinate spread：0 m（门 0.005 m）；
- 最大共同前缀 actor speed spread：0 m/s（门 0.02 m/s）；
- hidden actor policy runtime input：false。

冻结模型：

- 路径：
  `artifacts/v3/prediction/v3_02_c1_r1/model/v3_02_c1_r1_candidate_predictor.yaml`
- model SHA：
  `03cdc5b4d49c90e07bd7d1f0dbbde260dbfedc7c5b4db7c640c27ffb940580df`
- conformal radius：`0.4198179162478387 m`
- 10 features：bias、actor coordinate/speed、candidate speed、三项平方和三项两两交叉。

## 4. C2-I1 离线完整性修复

C2 仿真结束后，评估器已先写出 assessment，但在构造 evidence-freeze 资源表时因
`contract` 未加载到 `main()` 作用域而抛出 `NameError`。原 assessment SHA
`843a3bc76ee54d1bd7304ca5b88a3a641f32e8f5b1a5dd2d3eaeab942d2768e3`
被立即冻结且未改写。

C2-I1 只修复这一个变量作用域和模型清单路径问题：

- 没有重跑仿真、替换 seed、修改模型、重算半径或改变门槛；
- 从 36 条原始轨迹确定性重放，重建 assessment 与原 SHA 完全一致；
- evidence manifest 绑定 C1-R1 模型，不再错误指向历史 A1-R1；
- 独立完整性复核状态为
  `v3_02_c2_i1_pure_offline_integrity_repair_pass`，SHA
  `56ef648a291a9078ad2dce947b3f308e690d09b6ac325b4554afdbeafc0fabb4`。

## 5. 核心证据

| 资源 | SHA-256 |
|---|---|
| C1-R1 frozen model | `03cdc5b4d49c90e07bd7d1f0dbbde260dbfedc7c5b4db7c640c27ffb940580df` |
| C1-R1 calibration report | `e399f03fff133e76649375a6a0be2d6644777725e598827e998b8d2d4a9c5c2f` |
| C1-R1 independent model review | `28f3c72b677d7bbee3d5b056d5714c34d656d2a0e73363a0e82233237acdadde` |
| C2 release | `7d5670991cd6347dc30cc47f83159ef9caac5208fc80d442b800302d6100f57d` |
| C2 release review | `7d94588f098a6c8f8058248f1cada2d856663376ffe3d613fe1edccd974a4531` |
| C2 assessment | `843a3bc76ee54d1bd7304ca5b88a3a641f32e8f5b1a5dd2d3eaeab942d2768e3` |
| C2 evidence freeze | `ad89b4e40036cb90f8b98388c8541c1f10799aa8d00e47285ca909a5379a3229` |
| C2 result review | `9b8dfe277d7238f028b819ae94c2f9744c069a4f6ac099f31058a5eae73fc3a0` |
| C2-I1 integrity review | `56ef648a291a9078ad2dce947b3f308e690d09b6ac325b4554afdbeafc0fabb4` |

## 6. 权限与下一入口

V3-02/C1/C1-R1/C2/C2-I1 均已终结。不得重跑、续跑、复用其 seeds/budget，
不得测试后再拟合该模型，也不得将结果表述为安全或性能提升。

下一安全入口为独立 V3-03：

1. 以同一 decision state 生成 certified stop 和有限 Ackermann-feasible candidates；
2. 用冻结预测 envelope 做保守几何冲突过滤，并单独检查制动可达性；
3. 当预测缺失、时间戳过期、候选非法或所有运动候选被拒时确定性回退到 stop；
4. 解析 ranker 只能在 verifier 接受的候选内排序；
5. 先完成纯离线边界/故障注入与性质测试，再建立新的 simulation release；
6. 预测覆盖率不得被解释为绝对安全概率。
