# P05 独立接收时间测量修复审查

结论：`offline_driver_ingress_measurement_review_pass`。这是离线探针与管理器审查，不是仿真资格结果。审查时没有 P05 release、分配或运行。最终 probe SHA `d19bed1dddc202c6eacf9230de6d53126d34a94ed54ec4f6e6053d3bfa0c1c29`，manager SHA `2dac51996951377ac5e23a27b1354098ade69b6bbb0cbbc63d23747be8e50420`。

独立合成测试 **68/68**，作者探针测试复核 **53/53**，管理器测试复核 **55/55**，共 **176/176**。完整解析 toy 的 **122** 门全真。使用实际函数/`finally` AST 的假服务与回调控制流测试，不调用 ROS、Gazebo 或 build。

初始未发布版本 123 门的三项发现与原反例全部保留：暂停 watermark 之后的未来 ingress 可误通过；解析异常会在采集关闭后追加 raw；只改变无 Header 的回调时钟 +250 ms 会误触发旧 relay 匹配门。最终版要求 ingress 位于实际 2 ms 源格点且不越过稳定暂停 watermark；整个 protected 解析/记录/异常路径与关闭共用 RLock；旧 `command_relay_observed` 资格门删除，匹配率、0.98 阈值及通过标志仍作为诊断保留。严格100% source→driver 的数量、顺序、载荷及因果检查替代较弱的重复载荷匹配。原 loss 回调门替换成真实 driver source 接收门。最终122门不冒充最初123门未改变。

所有六合同的 gates、36 s schedule 和 world 与冻结 P02 完全相等；物理计算主干、所有共同 gate 条件、未变纯函数 AST 相等；toy 共同数值指标全部相等。实际转向门 **0.85 rad/s**、loss grace **0.12 s**、命令 gap **0.075 s**、末段 **0.75 s**、暂停排空墙钟预算 **5 s** 均未放宽。整数ns只修复 formal/phase/tail 半开与闭区间成员判定。

独立反例覆盖：真实 driver 在 loss grace 后接收命令必失败；未来/非格点、缺失、重复、源事件之前接收、额外 bootstrap、数量与载荷错误必失败；driver terminal 延迟须等到达，永久缺失须终止；10条源流尾包、四项 join、暂停服务和墙钟 deadline 的失败均保存原始数据与失败报告。关闭后、以及已等待 RLock 的回调均不能再次解析或改动 raw。

450项继承依赖逐一复核通过，只有预声明 CMakeLists.txt 与 package.xml 两项允许替换；旧420项 roster 仍包含在450项冻结继承中。旧 P02 physical negative、P03 2 consumed/4 retired、P04 1 consumed/5 retired 与原失败门保持不变。管理器六回合顺序、同 seed 配对、仅 Kd4→6 行为因素、physical-negative继续、infra失败停并退休、完整六回合且treatment3/3全门通过才能晋级均保留；未复用旧预算。

Driver由本代理编写，不能把本审查称作 driver 独立审查；该责任由单独绑定的 root报告 `m12_physics_p05_driver_root_review.json` 承担。本报告独立于 probe/manager 作者 root。

剩余限制：真实执行须另取新 source-bound 证据；driver ingress 是接收/排队时间，不是施力时刻。旧 Bool receipt-gap、无 Header 命令 callback 时刻是诊断；任意明确 Bool fault=true仍拒绝。源流完整性与单锁关闭不构成实时调度保证。P02/P03/P04 原失败不重分类。

机器证据与命令：`m12_p05_ingress_independent/`。初始快照、反例、全部测试日志、450项 SHA、源码 diff、数学身份核查和工具错误记录均保留。审查 JSON SHA：`9e0dc5a853f5d7cc2e7c938cdf796411df15b2c5673515954fc646880e3bf541`。
