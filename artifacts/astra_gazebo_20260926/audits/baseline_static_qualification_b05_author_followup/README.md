# B05 作者补充复核（非独立审查）

原作者复核，不能代替独立审查。原 B05 的264个封存资源均重新核验一致，没有修改源码、设计或历史报告。

保留一项真实 API 输入类型缺口：`proposal_schedule` 的 `other_proposed_seeds` 使用原值构建 set；若调用者给十进制字符串 `'263004001'`，不会与 int 新提案发生相交而被接受。同一个 int 值会被拒，registry 的 CSV 字符串也会因该分支调用 int 而被拒。当前已冻结 prepare 从 N06 JSON取整数，实际当前 registry 无冲突，重建提案与封存 schedule 完全一致。因此当前具体零权限提案没有发现种子冲突，但该 helper 不能声称任意 caller 类型均闭合。后继修订应严格校验整数元素或显式规范化并拒绝歧义；保留本版，不暗改修复。

完整参数比较只证明与传入 prepared 树相等。两个相同但不完整的树仍可通过，并显式返回 `runtime_default_completion_proved_by_this_function=false`；必须由后继 prepare/release 独立认证完整 defaults 和来源。记账 helper 也能对人为提供的布尔/零时目标做算术，但明确 `qualification_determined=false`；不能代替真实源时间/几何/回执验证。

冻结设计确实把 initialized defaults、实际九个 collision 部件在后轴参考下的投影包含、命令到 driver receipt 闭包和新 manager/collector 标为 pending。九个是实际碰撞部件数量，不是九种 primitive。B03 tilt、partial-batch 缺回执、public base终态权限和动态 actor不支持的边界继续保留。

此次只有5组小型离线探针，未启动 ROS/Gazebo、训练、登记种子或创建 release。独立审查应由非作者在 N06 优先任务完成后另行执行。
