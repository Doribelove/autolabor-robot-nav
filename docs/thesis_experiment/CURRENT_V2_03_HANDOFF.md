# FAM-TEB V2-03 Handoff

更新时间：2026-07-14 00:18 CST

## 当前结论

V2-03（局部世界模型、动态目标跟踪/预测、健康状态和不读取场景标签的规则模式监督器）
已经完成组件级实现与验收。机器记录为
`artifacts/v2/component_acceptance/v2_03_acceptance.yaml`，SHA256 清单为
`artifacts/v2/component_acceptance/v2_03_acceptance.sha256`。

所有部署参数和模式阈值继续保持 `runtime_ready=false`。本阶段没有启动 SAC、没有执行
实车闭环、没有发布新增速度命令，也没有写入在线 TEB 参数。

## 已完成范围

- `nav_world_model` 新增严格 LaserScan 元数据/角域/四方向覆盖校验、前后左右净空、局部
  障碍密度、侧墙直线拟合、走廊宽度/中心偏移、dead-end score 和可选全局路径几何；
- 新增按扫描顺序聚类、门控最近邻数据关联、alpha-beta 速度估计、miss 生命周期、时间
  回退 reset，以及带增长协方差的 0.5--2.0 s 常速度预测；
- 新增 scan、TF、里程计、tracker、方向覆盖和序列一致性的健康检查。stale、TF 失败、
  定位失败或序列错配均 fail closed；
- `teb_mode_manager` 新增纯规则监督器，从运行时几何和轨迹产生五种 `GeometryMode` 与
  五种 `DynamicOverlay`，包含置信度、进入确认、最短驻留、过渡和 overlay 释放滞回；
- 健康故障立即输出无效 `BALANCED/NONE/FAULTED`；低置信度使用 `BALANCED`；
- 运行时节点不订阅 `/gazebo/model_states`、Pedsim 真值，不读取 scene manifest、family、
  split 或 evaluator-only 标签；Gazebo 真值只由独立验收脚本读取；
- 两个候选节点必须同时看到 `/m2_gazebo/simulation_only=true` 和显式
  `allow_unfrozen_simulation_candidate=true` 才能启动，默认 launch 仍拒绝运行。

## 验收结果

- 全工作空间 `catkin_make -j4 -l4`：通过，73 个包；
- 全 Python/config：177 passed；
- catkin：`nav_world_model` 5/5、`teb_mode_manager` 5/5、
  `thesis_experiment` 34/34、`m2_gazebo` 34/34、`teb_rl_tuner` 60/60；
- 合成动态目标：48 个有效样本，位置 RMSE 0.02074 m，1 s 预测 RMSE 0.07800 m，
  ID switch 0；
- 合成五类模式矩阵：5/5 对角命中，macro recall 1.0；四种健康故障 4/4 fail closed；
- 动态 crossing Gazebo 探针：运行时仅用激光/里程计/TF，独立 evaluator 获得 88 个跟踪
  样本，位置 RMSE 0.26991 m，ID switch 0，95 个有效健康样本，13 个 `CROSSING`
  overlay 样本；
- V1 调参器和 V2-02 动力学回归继续通过。

## 机器入口

- 合同：`config/thesis_experiments/v2/world_model_contract.yaml`；
- 世界模型候选：`src/perception/nav_world_model/config/v2_03_candidate.yaml`；
- 规则监督器候选：`src/application/teb_mode_manager/config/v2_03_rule_candidate.yaml`；
- 合成验收：`artifacts/v2/component_acceptance/v2_03_synthetic_acceptance.yaml`；
- Gazebo 探针：`artifacts/v2/component_acceptance/v2_03_gazebo_runtime_probe.yaml`。

## 不得误解的边界

- 当前是二维激光聚类和最近邻 alpha-beta tracker，不包含多传感器融合、遮挡重识别、
  JPDA/MHT 或学习型预测；
- 常速度预测不适合长时或强交互行为外推；
- 五类混淆矩阵来自冻结合成特征，尚未替代完整五场景 Gazebo 感知矩阵；
- Gazebo 真值仅证明 evaluator 能计算误差，不是策略输入；
- 尚未接入 TEB dynamic obstacle bridge、Anchor Bank、可行动作解码或任何规划后端切换；
- 本阶段不证明导航成功率、效率或学习效果提升，也不提供实车安全结论。

## 下一实施门：V2-04

下一步是 Anchor Bank、类型化 profile、从上一 `executed` 参数开始的平滑过渡、可行动作
解码和四阶段 action trace，然后进行无训练规则闭环。V2-04 仍不得启动 SAC 训练或实车闭环；
正常路径 projection rate、切换连续性和参数事务可重建性必须先达到门槛。
