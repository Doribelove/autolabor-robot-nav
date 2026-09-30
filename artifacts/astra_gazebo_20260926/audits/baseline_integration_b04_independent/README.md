# B04 独立离线审查

结论：冻结 B04 静态源码、合同、两份实际 launch 展开及合成 bag 接线检查通过；未取得真实 TEB/DWA 后轴物理运行、动态 actor 或实时回执完整性资格。本报告不是 execution release。

199 个作者资源重算一致；48 项作者测试、10 项独立边界测试、10 项实际输入/源码聚合检查通过。独立测试提取当前 `navigation_evidence_b04.assess` 的原字节 bag 读取块，并用真实 ROS 消息写读临时合成 bag，检查全部后轴证据类型。缺最终回执、错 caller、缺 provided_path、重复或缺源时格点、错误 availability 都被拒绝；接收时刻没有被当成测量源时刻。没有真实 B04 bag，也没有声称完成原生 planner live transport。

公共成功仍由原始 base_link 源里程计判定（0.25 m / 0.25 rad），22 s 截止及物理评价沿用 N02。后轴是私有的 -0.325 m 刚体参考点；完整三维姿态、角速度叉乘项、实际 vy 和协方差不被截断。原始 map 保持；完整有向路径/目标转换服务与实际 provided_path 进入证据链。私有 action SUCCESS 只是诊断，不能替代公共终态。B03 已复核的 callback 顺序、部分输出缺回执与倾斜差异限制继续保留。

两个实际 XmlLoader/Xacro 展开与保存的所有参数/节点完全相同，未启动 ROS。完整 64 点 costmap footprint、TEB 16 点 raw footprint 在局部 x 加 0.325 m，无减少；TEB 40 个、DWA 28 个非 overlay baseline 字段与冻结原配置相同。DWA acc_lim_y=2.5 仍是采样器虚拟变化率，不能表述为真实横向执行能力。body command 与 P05/Kd6 原物理控制不改。

参数核验边界必须明确：当前运行时 `rear.planner_parameters` 检查 planner class 和 11 个 rear overlay 字段，全 namespace 虽已保存，但不自动核完其他值。独立反例把 max_vel_x 改成 17，该窄 checker 仍接受。本次实际展开值正确；未来闭合 manager/release 必须对照完整 prepared 参数，而不能用这 11 字段的通过替代全参数公平性证明。rear status fault 最终导致完整性失败，不构成已验证的即时制动行为。

B03 `publish_pending` 已发表第一条后、下一条过期会让第一条无回执，这个真实函数反例在本审中保留且 B04 拒绝不完整序列，没有事后补回执。暂停后的 5 s 总 drain、50 Hz base/rear/receipt 到最终 watermark 的源格点齐全要求、同锁冻结和独立 bag 重放均保留。早期无 gate 情形保持失败，不制造有效任务。

尚需独立未来阶段：fresh schedule/预算/release、完整 converter/SDF/原生 ELF/interpreter/参数闭包、实际 rear TF/odom/goal transport 和完整回执、steering/braking/goal order 物理资格。动态 actor 明确不支持；当前仅静态 clear fixture，seed=1 仅离线语法占位且 run 根不存在。0.0004061653716359581 m 的 tilt/yaw-only 示例差异、折线曲率与 stop-and-rotate 的执行局限未修饰为合格。没有一般安全、性能、训练、实车或动态六族比较结论。
