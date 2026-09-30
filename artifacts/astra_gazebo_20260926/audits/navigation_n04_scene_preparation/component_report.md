N04 六族开发输入离线准备完成：6 个原几何，各 2 个参数/噪声重复。21 项作者测试通过，原始输入字节和 parent geometry 身份全部保留；263001001–263001012 仅提出，未登记。

每场包含 world.sdf 身份模板、共同 map/route、精确 blocked-cell 矩形并集及 GEOS 逐格参考、9 个 converted-SDF 碰撞体初始诊断、actor 全路径扫掠和原 parent 文件。模板必须经 bind_gate_identity 绑定具体 run/seed/method；模板自身不能执行。

遮挡场景保留原 sin^4 见证裕量不足的结果。另存 quintic 补充见证：静态物理区间下界 0.479084 m，减 0.33 后为 0.149084 m，中央转角解析上界 0.288257 rad。仅说明理想平面静态 Ackermann 几何存在性；动态 actor 和真实跟踪待实跑。

全部路线当前为 yaw=0。冻结 R2 接收相同 blocked-cell 输入但仍不消费共同 polyline；任意旋转路线需要独立方法桥接。新 actor 事件 gate、实际48份launch、接触/稠密姿态/原生相机仍需 root 的独立发布和 live 资格。没有启动 ROS/Gazebo，没有生成虚构图片。
