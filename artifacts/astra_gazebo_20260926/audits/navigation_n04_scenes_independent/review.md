N04 场景独立离线复核通过；没有启动 ROS、Gazebo、登记种子或改变旧证据。

直接从实际转换 SDF 解析7个刚体、9个碰撞部件，独立使用4×4矩阵运动学、盒体8角点和外接48边形圆柱投影；未调用作者的 ReferenceRobot / project_box / project_cylinder 来得到见证数值。对每类901个采样点重算净空，再使用全区间导数上界减去半步位移及1e-5米轮轴量化余量。48份已准备的实际转换 SDF 碰撞、惯性和关节结构全部与参考一致。

|几何族|实际共享guide长度/米|采用的独立静态见证|区间物理净空下界−0.33米|
|---|---:|---|---:|
|static_avoidance|9.382549430|sin4|0.233311061|
|single_crossing|9.000000000|sin4|无静态障碍|
|head_on_yield|9.000000000|sin4|1.661125920|
|sequential_interaction|9.000000000|sin4|无静态障碍|
|occluded_emergence|9.173214386|quintic|0.149083801|
|width_change_corridor|9.000000000|sin4|0.751090000|

所有场景的名义起终点纵向距离仍为9米。共享guide与见证曲线各自不同，不能将这张表理解为实际行驶轨迹、时间或跟踪性能。

遮挡场景原sin⁴见证的采样物理净空0.241362796米，扣区间余量及0.33米后为−0.117345710米；原失败文件保持不变。独立重算的新quintic为采样0.490048601米、区间下界0.479083801米、扣0.33米后0.149083801米，中心转角上界0.288257126rad，小于0.4964rad。该曲线端点的横向位移、航向、曲率均为零；中点两段连接C²，分段三阶导数界足以支持所用Lipschitz余量。九部件半径界≤1米、前轮转向局部半径≤0.3米，量化轮轴的旋转形状偏差约1.14047e-6米，低于所扣1e-5米。

所有actor起点到终点的恒定朝向平移凸包均与static不相交。有限最小净空分别为head-on1.094735920米、遮挡0.241650000米；crossing和sequential的静态障碍集合为空，结果以null表达，未编造大净空数。机器人spawn/gate/goal九部件与static/初始actor、actor彼此初始形状均不相交。遮挡静态ray复算得到gate可见1/30束，另一静态位置60/60束；不能据此声称真实动态可见时序已通过。

12行严格继承旧CSV顺序、6个旧parent、原碰撞几何与actor路径/策略参数/response_seed。新种子为263001001+原index。24个已提供旧development parent的canonical ID、精确geometry SHA与split桶独立重算，10个actor参数cell同属development，24个parent的近重复检查无冲突。这里没有已物化train/validation/final总集合，故不声称全局held-out隔离证明或完整样本量；6个旧parent和每组两次输入变体不能记作12个独立几何。

调用实际纯identity函数完成60个正常绑定和132个拒绝反例；全XML比较证明正常绑定只改变expected_run_id/expected_seed/expected_method。显式剔除所有declared actor后静态输入XML与保存内容相等，PGM/YAML回放字节相等；另以独立整数cell栅格覆盖逐像素证明R2矩形输入与所有非free栅格一致。每组两次输入的静态图和共享guide也字节相等。旧R2仍不消费共享guide，静态栅格相等不代表全部方法的路线或图外语义等价。

尚无本审计产生的运行授权。12×4配对设计与顺序平衡复核通过，种子未出现于保存的registry快照；这里只读快照，不修改全局注册表。旧碰撞/物理资格与未来actor共同gate运行资格均应使用各自正式证据。

复现命令见commands.json；实际结果见recompute_result.json、closure_result.json及7份逐点CSV。两脚本首次执行通过，纯路径定位错误另存tool_errors.json。输入在脚本结束及封存时均重新计算SHA。
