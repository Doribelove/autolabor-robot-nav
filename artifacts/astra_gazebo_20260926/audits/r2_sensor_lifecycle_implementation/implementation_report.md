新 `frozen_r2_v060_sensor_lifecycle_v1` 已完成：60项实际代码离线测试通过，旧 R1/R2/runtime/权重 SHA 未变。

构造器移除模型真值/接触订阅；实际扫描按原浮点 0.20 s 规则写入六次后才就绪。候选、特征、掩码、排序和控制循环继承原实现；测试重放同一有效传感器流得到相同输出。共同终态取自 coordinator，诊断姿态来自 odom，不再由 model_states 早停。决策/控制 trace 在追加时发布，终态报告不依赖原 ROS tail sleep。

共享 map/path 有内容校验和完整 receipt，但路径使用仍明确为 audit_only；旧 R2 绝对 x 候选和 ±20 m 隐式边界保留。未宣称所有方法路线消费等价、物理安全或主矩阵资格。

三个新源文件及快照、旧依赖/权重、完整参数和 topic API 均绑定在 JSON。当前 N01/旧八组件未改；新 method enum 需后继整合。尚无 ROS/Gazebo、构建、种子分配或训练；等待独立审查和新阶段实时资格。
