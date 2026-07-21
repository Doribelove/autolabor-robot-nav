# Thesis TEB RL Experiment Handoff

本目录是 Ubuntu 20.04 上开发 TEB 强化学习调参、Gazebo 仿真与 Autolabor M2
sim-to-real 实验的交接入口。

阅读顺序：

1. `CURRENT_TEB_RL_HANDOFF.md`：当前跨会话主交接书；
2. `CURRENT_T12_RESIDUAL_TRAINING_HANDOFF.md` 与
   `CURRENT_T12_RESIDUAL_LEARNING_REVIEW.md`：T12 冻结事实和禁止重跑边界；
3. `V2_SYSTEM_GUIDE.md`：FAM-TEB V2 系统架构、接口、软件骨架、实施和验收指南；
4. `CURRENT_V2_FOUNDATION_HANDOFF.md`：V2-00/V2-01 实施事实和基础验收；
5. `CURRENT_V2_02_HANDOFF.md`：V2-02 仿真动力学、五类场景、统一 evaluator 和下一门禁；
6. `CURRENT_V2_03_HANDOFF.md`：V2-03 世界模型、跟踪预测、健康与规则监督器验收；
7. `DEVELOPMENT_STATUS.md`：阶段状态和复现命令；
8. `experiment_contract.yaml`：V1/T00--T12 机器可读论文、接口和安全合同；
9. `UBUNTU20_TEB_RL_EXPERIMENT_BOOK.md`：V1 完整开发和实验书；
10. `schemas/`：episode 与 RL step 数据字段；
11. `templates/`：`A_TEB`、run manifest 和统计预注册模板。

旧 T00 首轮提示词和旧 `system_inventory.yaml` 已按用户要求删除。新会话不要
重做 T00--T11，也不得重启历史 T12 pilot。V2-00--V2-03 组件已实现，但规划后端、正式
实验和实车闭环均未开始；所有运行阈值保持 `runtime_ready=false`，该状态不授权新训练
或实车闭环。

最重要的执行约束：

- 默认仿真、离线回放或 shadow；
- Codex 不得自主启动实车运动；
- NoSafety、ProjectionOnly、NoFallback 不得实车闭环；
- DWA 只作固定传统基线；
- 论文数值必须由原始数据和冻结统计脚本生成。
