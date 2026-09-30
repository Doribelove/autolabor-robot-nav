r03 的 preview 严格 schema/method/action 类型与边界已通过；42项 focused synthetic检查通过，两个真实已闭receipt和6个候选payload兼容。

仅剩 failed report 路径/哈希未验证：空dict、不存在文件、写后篡改的绑定报告均可接受。这三项是合成负例，不是N05运行结论。主代理已另建r04修补，r03保持不改。

首次harness manager.NAME写短导致实际receipt误拒；原脚本/log保留，使用真实常量后通过。未完整回放、未运行ROS。
