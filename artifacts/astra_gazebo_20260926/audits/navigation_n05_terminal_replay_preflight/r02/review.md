r02 前置独立复核：原绑定、正式时窗、终态坐标、完整 release 调用与空分母问题已修复，18 项 focused fixture 通过。

尚需两项收口：preview 的 schema/method 和 action 类型/范围；所有 consumed 失败行也要验证 receipt 并列明。五个 preview 反例与无 receipt 失败矩阵均为 synthetic，不能表述为实际 N05 失败。

本轮未打开实际 bag、未重验活跃 release、未运行 ROS。main 的完整 verifier 调用使用明确 stub 检查控制流，实际函数源码另作只读审查。候选时钟是 decision now；trace 是 prepublication end_ns；每消息 stop 占比不是停留时长。
