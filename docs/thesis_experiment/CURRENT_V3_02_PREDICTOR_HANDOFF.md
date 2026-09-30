# V3-SCOPE V3-02 候选条件预测与占据校准交接

日期：2026-07-24  
工作区：`/home/robot/robot_ws_base_rl`  
终态：`v3_02_t2_independent_heldout_review_pass`

## 1. 结论

V3-02 已完成 parameterized-crossing 范围内的 candidate-conditioned latent-mode actor
预测器、物理路径约束、split-conformal reachable envelope 和一次 fresh held-out 正式验证。
最终 T2 以 frozen model / fresh seeds 6231--6233 完成 36/36 episode，并通过全部冻结门。

这证明的是三个 crossing 角度/初距/基速 block 上的有限 held-out 预测，不证明其他场景族、
导航安全、规划性能、实车或部署。

## 2. 数据与模型

- A1 train/cal 数据：6 training blocks / 72 episode，3 calibration blocks / 36 episode；
- runtime features：actor conflict coordinate、观测速度、candidate ego speed/trajectory、actor
  path direction；
- actor policy label 只用于 evaluator 分层和 mode-head 拟合，runtime 总是输出 cautious /
  assertive / hesitant 三个 head，不读取真实 policy；
- 模型：每 mode × 4 horizon 的 ridge regression，共 12 heads；
- 物理约束：输出坐标夹紧在 `[观测坐标, actor path endpoint 3.5 m]`；
- conformal score：`actual_mode_absolute_residual`；
- 最终 radius：0.09460804165531622 m。

## 3. 自纠正链

- A1 原模型：覆盖率/MAE/敏感性通过，但未夹紧的线性外推使平均包络宽度 5.1506 m，
  超过 4.50 m，校准失败；
- A1-R1：只加入物理路径 clip，宽度降为 2.1866 m，校准门通过；
- T1 frozen-model test：36/36，但 coverage 0.7986、最低 horizon 0.75，正式阴性；
- B1：诊断发现 outer-envelope nonconformity 会因“其他 mode 偶然包住 truth”把 residual
  压成 0；只把 score 改为 actual-mode absolute residual，未使用 T1 outcome 拟合或求半径；
- T2：使用全新 seeds 6231--6233 一次性验证，模型/半径在测试前冻结，测试后未改动。

T1 负结果、旧模型、旧 seeds/release 均保留且不得重跑或重新解释。

## 4. T2 正式结果

- 36/36 episode，144 prediction points，attempt=1，0 retry；
- marginal coverage：0.9930556（门 0.90）；
- per-horizon coverage：1.0 / 1.0 / 0.9722222 / 1.0（门 0.80）；
- mean full envelope width：2.3397001 m（门 4.50 m）；
- actual-mode MAE：0.0482765 m（门 0.50 m）；
- minimum candidate sensitivity：0.7625172 m（门 0.20 m）；
- physical coordinate violation：0；
- hidden-policy runtime input：false。

核心 SHA-256：

| 资源 | SHA-256 |
|---|---|
| frozen B1 model | `34458918fd937ef07fc7f87b0954b0f2fdc432d5839c0947ad456b7771ed7ae0` |
| B1 calibration report | `f6d9f0ff5994e0a5eb7b931227b4c9f37f4c0beb2f535bbc4cda76fb673d1a84` |
| B1 independent review | `846b891acf04e41d1ac0454004ce5e4ff01331c0c1bb01d0389b34b8a9e6ed53` |
| T2 assessment | `ac342ce7069f2c1cf6a189c69140087b0f72b5140fd26cd5ab99d840a658cb09` |
| T2 independent result review | `758cc530c74b44655b60a69c1a2b0da49b22dd251b8fe98c386e92c5aee6e7dd` |
| T2 evidence freeze | `5d14209c093e7c92158d084d760c2d6627d338fa785089293aa75acd33b9f330` |

## 5. 权限与下一入口

V3-02 没有运行 move_base、RL、CAN/串口或实车。`performance_improvement=false`，
`planner_safety=false`，`real_vehicle_ready=false`。

下一安全入口为独立 V3-03：

1. 用 frozen B1 reachable envelope 做硬碰撞/制动可达性过滤；
2. 至少保留 certified stop 与 deterministic fallback；
3. 解析 ranker 只能在安全候选中排序；
4. 先做纯离线 verifier/ranker fixtures、边界与故障注入，再做独立 bounded simulation；
5. 不得把 predictor coverage 直接写成系统安全保证。

