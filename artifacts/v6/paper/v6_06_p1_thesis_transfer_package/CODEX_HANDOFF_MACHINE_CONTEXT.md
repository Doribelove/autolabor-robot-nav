# Codex 机器交接上下文

- 原项目根目录：`/home/robot/robot_ws_base_rl`；复制到其他机器后优先使用本包相对路径。
- 稳定实车工作区 `/home/robot/robot_ws` 不属于论文写作输入，且本阶段未修改。
- V3 原始终态 TeX/PDF、V5/V6 正式评估均为冻结上游；学校模板迁移只能派生编辑，不能回写历史证据。
- `FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.tex` 是可独立编译的内容参考，不是学校模板。
- V5-00 历史架构合同有已记录的 YAML 缩进缺陷且被发布哈希绑定；论文写作不应修补它。
- 另一台电脑不需要重跑 ROS/Gazebo/训练即可完成论文更新；所有论文数值已导出到 `data/`。
- 实车实验状态为 `pending_no_evidence`。未提供新日志前，不得让 Codex补全任何结果数字。
- 如初稿中的数字与本包冲突，先查 `combined_claims_evidence_matrix.yaml`，再查对应 CSV/assessment。
