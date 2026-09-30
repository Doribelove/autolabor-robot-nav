# B08：`move_base` 根命名空间生成默认值清单

只读对照当前隔离工作区的 `move_base.cfg.MoveBaseConfig` 与 B04 保存的两套实际 TEB/DWA 离线 launch 展开。机器报告见 [inventory.json](inventory.json)，SHA256 `e924f3d39db4b94c3ee42d8656cda9e58733401fcd3a75857cc6c52a790a2c04`。两个方法的 16 个生成默认键中，均有以下 5 个未在离线展开根命名空间显式列出：

`conservative_reset_dist`、`make_plan_add_unreachable_goal`、`make_plan_clear_costmap`、`max_planning_retries`、`restore_defaults`。

这补充 [B07](../baseline_static_defaults_b07/README.md) 的活动 planner/costmap 清单；两份清单都只是源码候选。生成默认值可能被初始化改写，也可能不以持久参数出现，不能代替 paused bootstrap、positive clock ready、paused terminal 三阶段完整 `/move_base` 实际树的读取与精确比较。B05 的静态双基线仍无运行资格。

生成器 [inventory.py](inventory.py) 会绑定 cfg、两份离线展开、B07 及自身哈希，并拒绝覆盖已封存报告。未启动 ROS/Gazebo、未登记种子或创建 release。
