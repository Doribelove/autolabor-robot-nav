# B05 独立离线设计与纯组件复核

结论：**设计范围与参数比较/失败包括性算术有限通过；通用种子冲突 API 有确认缺口，未合格。** B05 保持执行权限、已分配预算、登记种子均为零。未来使用通用排程入口前须新版本修复类型门，不能修改本次冻结字节后仍沿用旧报告。

实际复验了全部 264 资源、20 项作者测试和 10 组独立检查。两份实际 B04 离线展开参数树的每个标量分别单独修改：TEB 417 次、DWA 405 次均被拒绝；覆盖超出原 B04 11 字段 overlay 的值。精确类型、signed zero、非有限值、路径转义和三个必需阶段均符合比较器声明。

确认反例：对全部十个提案 seed，`other_proposed_seeds=[str(seed)]` 都被接受，结果与无冲突输入相同；同值整数及历史 CSV 数字字符串都被拒绝。原因是源码第98行仅对 historical_rows 的 seed 做 int 转换，第100行直接与未验证的 other_proposed_seeds 集合求交。建议在新版本对该参数容器和所有元素做明确类型验证（推荐 exact int，拒绝 bool/string/float），或用显式、经过审核的规范化接口。不能继续声称当前通用 typed 冲突门已闭合。

实际冻结输入不受此反例影响：已绑定 N06 prepared 的 seed 都是整数，和 B05 十个 seed 不相交；历史 registry 快照与原输入重算出的 20 行 proposal 完全相同。该检查不预约号码，未来登记前仍须重新验所有历史状态/其它提案。

失败包括性检查用独立手算复核混合成功、timeout、capture 失败、geometry 失败和未启动退休：所有尝试失败保留22秒；退休仅进入明确标注的行政 cap 表，不进入尝试均值；十对完整性与 DWA−TEB 算术一致。缺行、重复、伪造退休测量、private_success 和错误类型被拒绝。这个函数只消费调用者标签，不认证真实 goal、geometry 或 stop policy；合成零秒全成功仍可算术通过且明确 qualification_determined=false。不能拿标签本身当运行证据。

四项实现边界仍阻断实际运行资格：

- 完整已初始化默认参数树尚未生成。比较器会接受两个相同的部分树，并明确不证明默认值完整性；实际 B04 launch dump 也不等于 initialized runtime namespace。
- 九个实际 collision shape 的逐源 rear-yaw 原始/外扩 footprint 包含检查未实现；此前 tilt 0.000406165m 差异保留，不能新增容差掩盖。
- 真实 command→relay→driver ingress 和 rear partial-batch 回执闭包未实现。driver receipt 表示接收/入队，不等于力生效；headerless Twist、重复值与末端无 trace zero 必须由新采集器明确处理。
- 新 manager/collector、stage stop-policy 实施、完整 native/converter/runtime 闭包以及实际静态物理运行均不存在；动态 actor 不在本设计范围。公共 base goal 仍是唯一终态权限，private rear SUCCESS 不能替代。

审查独立性：本审查者未编写 B05 源码、设计或作者测试；之前仅给过三条边界提醒。审查者曾编写 B04 上游集成包，本报告没有把该上游作者工作重新宣称为独审，而是继承 environment_audit 的已冻结 B04 独审并仅将其实际参数输出作为新比较器测试输入。

全程无 ROS/Gazebo、训练、真实 release 或种子登记；真实三 CSV 的 bytes/inode/mtime 未变。冻结作者文件不改，全部反例保留。
