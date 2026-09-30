# N06 manager r03 独立离线复核

结论：**原 r02 两项修补通过，但发现一个终态故障回收阻断，r03 不可用于 release。** 原 r01/r02 负结果不改。

实际运行62项作者测试和本报告绑定的 focused 检查：metadata 闭15字段，run_id 覆盖及错误 canonical 路径/hash/process 被拒绝；任意路径最小 assessment、缺9checks/6sources/11sweep来源被拒绝。实际 navigation_evidence_n06.assess AST 的9checks和6sources、dense_sweep_n05.main 的11来源与新校验一致（N06 不启用 R2 条件分支）。冻结 world/URDF 即使重算下游报告 hash 仍不能替换；collector/closure身份和结果不一致被拒绝。负 assessment、事后 error 及真正无报告的启动失败均保持阴性；新增process依赖在首CSV替换后变化会阻止后续表/完成，恢复字节可续提交且完成后重复调用不写文件。全部临时报告、bag字节、PID都是明确标注的结构夹具，不是仿真证据。

新反例使用实际 reconcile_worker_exit：先持久 consume 一条；构造与当前 validator 一致的完整 assessment 文件，模拟 assessor 已原子写出报告后 worker 在报告返回/receipt发布前退出。supervisor 调 terminal_receipt(report=None)，该函数发现已有 assessment 并绑定它，但 task_outcome 仍为 None。close_unit 的 validator 因报告 task 与 receipt 不一致抛错。实测 ledger 仍 active、首消费行 receipt 未绑定、退休数0。assessment 字节保留，失败 receipt 文件也留下，但预算终态闭包未完成。

这是退出窗口处理缺口，不要求物理重放，也不允许借读回报告把基础设施失败晋升为成功。最小新版本应从已保留、经完整验证的报告派生 task，保留 interrupted worker 的整体 integrity=false 和原错误；实际 supervisor 回收须完成1 consumed/7 retired并不重跑。r04 由作者另建，本报告不审其中间字节。

全部 r01 352、r02 733、r03 764 资源在检查与封存时重哈希匹配。manager/process 除依赖名外与 r02 字节相同；继承 r01 的实际 disposable-child 清理测试与 r02 的转换认证验证，不冒称本轮重新执行。真正三CSV bytes/inode/mtime未变。无ROS/Gazebo、训练、实际release、seed登记或真实行政apply。

限定：本报告审结构一致性与故障闭合，不认证 synthetic report 内的物理值。最终N06 source/lifecycle/runtime闭包仍需单独新身份审查；三CSV更新仍为可恢复顺序替换，不具联合原子可见性，N05外层143原因仍未知。
