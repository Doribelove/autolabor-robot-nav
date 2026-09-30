# B07：实际 TEB/DWA 参数默认值的离线缺口清单

这是只读源码清单，不是新运行资格。用当前隔离仿真工作区的 generated dynamic-reconfigure Python 配置，对照 B04 两方法保存的 `effective_move_base.json` 离线 launch 展开值。结果见 [inventory.json](inventory.json)，SHA256 `ff7cfe847a06545ccd6ecc91d0cc3e71e45e531bc11cc69994a94896ae830559`。

| 方法 | 活动 planner 的 generated 默认键 | 现有 launch 展开中未显式列出的键 |
|---|---:|---:|
| TEB 后轴参考 | 91 | 51 |
| DWA 后轴参考 | 35 | 7 |

两方法的 global/local costmap 各有 3 个 Costmap2D、1 个 InflationLayer、3 个 ObstacleLayer generated 默认键未在对应展开子树显式列出。这些键**可能**由插件初始化时写入参数服务；清单不等于实际初始化后的 `/move_base` 参数树，也不表示展开配置错误。当前 B05 的完整树比较器不能凭已有 launch 展开值自行证明默认值完备。后续应先从绑定的 cfg/源码构造版本化期望，再在 fresh 静态资格回合的 paused bootstrap、positive clock ready 与 paused terminal 三个阶段读取完整实际树，按类型和所有键比较；任何意外默认值或初始化差异均保留为失败证据。

未启动 ROS/Gazebo、未创建 release、未登记种子、未改动 TEB/DWA 参数或稳定实车工作区。重新生成时，在仓库根目录 source **仅隔离仿真工作区**环境后运行 `python artifacts/astra_gazebo_20260926/audits/baseline_static_defaults_b07/inventory.py`，并使用新版本输出路径保留本次原件。
