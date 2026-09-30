# B06：B05 提案种子类型门的零权限最小修复

新模块 `baseline_static_qualification_b06.py` 只提供一个 `proposal_schedule` 包装函数。先要求 `other_proposed_seeds` 是 list/tuple，再拷贝为 tuple，并要求每个元素 `type(seed) is int`，最后调用未改的 B05 原函数。没有 coercion、过滤、自动换种子、参数/defaults、物理、回执、manager、collector、ROS 或执行实现。

全部十个 B05 数字字符串绕过反例都保留，B06 对它们拒绝。bool、float、int子类、混合类型都拒绝；字符串、bytes、映射和惰性 iterable 不再被隐式解读成种子容器。有效的 list/tuple 参数保留旧行为；不会另加种子范围/正数等语义变化。原 registry CSV 字符串解析策略仍由 B05 负责。

当前真实 B05 frozen scenes + 原registry快照 + N06整数提案输入，返回值逐项和规范JSON均与原 B05 schedule 相同。B06 是修复组件身份，不是另一个可执行实验阶段；输出保留原 B05 的零权限提案身份、20单元设想、allocated_budget=0。既未复制执行授权，也未登记或占用任何种子。

10项作者纯 CPU tests通过，未启动ROS/Gazebo或训练。B05原264资源、原阴性与独立审查保持原字节。严格参数默认值来源、实际九collision/后轴物理、命令回执及manager/collector的缺口仍全部pending。本包需非作者独审，不把本作者报告当独立通过。
