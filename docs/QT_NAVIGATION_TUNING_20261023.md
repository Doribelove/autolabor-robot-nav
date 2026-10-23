# Qt 导航调参与长方形覆盖区域

本次开发目录为 `/home/slam/robot_j6m_ws_navigation_tuning_20261023`，分支
`feature/qt-navigation-tuning-20261023`，从原目录 `0914` 的 HEAD 建立 Git worktree，
并逐文件复制开工时的 tracked/untracked 源码修改。原目录及分支保持原状；没有提交或推送。

## 使用

Qt 新增“导航调参”页，内含覆盖规划、转场 TEB、Hybrid A* 三个子页签。原清扫页的基础参数整组迁入此页，清扫页提供跳转按钮。

- 覆盖规划：清扫宽度、重叠率、前进/倒车速度、倒车许可、角速度、线/角加速度、
  换向/交接时间和异常重规划等待。沿用原事务接口及参数持久化，不改变原安全上限。
- 转场 TEB：独立前进/倒车速度、角速度、线/角加速度（0 表示跟随覆盖参数）、
  前视距离、路径点间距、障碍/膨胀距离、路径点/横向/航向/前进运动学权重、候选倍率及内外迭代次数。
- 混合 A*：运动步长、碰撞采样间距、状态分辨率、航向离散、转向采样、最大扩展节点、
  搜索预算、启发式权重、转向/转向变化惩罚和障碍代价倍率。

高级组先点“读取后端当前值”，成功后可编辑；修改经 400 ms 防抖异步下发并用响应回显实际值。
失败不会显示为成功；请求被后端拒绝或修正时明确提示，并显示真实值。
滚轮不会修改数值，避免滚动页面时意外自动调参。

无需重启即可采用参数，但生效边界如下：

- 覆盖规划与转场参数只在无预览、无任务、无规划/启动或恢复事务时接受修改；
  已有轨迹须先取消预览，执行中的任务须先完成或由操作员取消，再修改和重新规划。
  转场参数直接更新管理器下一次下发给 TEB 的专用 profile，普通导航和清扫 profile 不继承它。
- Hybrid 参数在当前完整搜索结束后切换，之后的搜索直接使用新配置，不中途更换一次搜索的目标函数。
  已验证的参考路径保持有效，下一次需要搜索时使用新配置。
- 高级参数经后端原子写入持久 JSON 后才确认应用；重启优先读取保存值，覆盖 launch/YAML 中的默认值。覆盖基础参数保留原 YAML 与 Qt 设置持久化流程。

转向采样必须为 3～11 的奇数，偶数会下修为相邻奇数并回显。所有可调项保留有限数值、
范围及配置的硬件上限约束，不开放物理车体尺寸、转弯半径、运动授权、停车/定位/时效安全门。

## 长方形规则

Qt 确认选区后先拟合，回显四角矩形并提交规划；后端也独立拟合所有单区与批次区域，
包括历史保存的多边形和其他客户端的请求。规划使用矩形，发布区域与标记也使用同一矩形。
新保存的区域来自已确认的矩形；旧区域文件不静默改写，重新确认/批次执行时拟合。

算法先对去重后的点求凸包，再枚举凸包边方向的包围矩形，选择面积最小者。
矩形可以旋转，不限制地图轴方向；乱序、凹形、自交点序均按点集求外接矩形。
非有限坐标、共线点、少于三个不同有效点或外接矩形面积小于 0.5 m² 明确拒绝。
原障碍物、车体完整外廓和可达性检查继续作用于拟合后的区域；外接扩展区域不代表一定可清扫。

## 主要实现

| 文件 | 改动 |
| --- | --- |
| `autolabor_operator_gui/src/navigation_tuning_page.cpp` | 独立页面、异步回读和自动应用 |
| `autolabor_operator_gui/src/main_window.cpp`、头文件 | 页签、迁移基础组、Qt 选区拟合 |
| `autolabor_operator_gui/include/autolabor_operator_gui/minimum_rectangle.h` | Qt 点集最小面积外接矩形 |
| `autolabor_coverage/src/autolabor_coverage/coverage_geometry.py` | 后端独立矩形算法 |
| `autolabor_coverage/scripts/coverage_manager.py` | 单区/批次约束及转场运行参数接口 |
| `autolabor_coverage/cfg/TransitTuning.cfg`、`HybridTuning.cfg` | 类型、范围与参数描述 |
| `autolabor_coverage/src/coverage_global_planner.cpp`、头文件 | 线程隔离的 Hybrid 运行配置 |
| 两包 CMake 与测试 | 配置生成、矩形、调参、Qt 页面和插件回归 |

以上路径相对 `src/application/`。

## 验证与运行边界

`./scripts/validate_navigation_tuning.sh build` 构建本次两个包的 Release；
`./scripts/validate_navigation_tuning.sh test` 运行覆盖与 Qt 测试。脚本只允许本目录可写，
使用独立网络/PID 命名空间和回环 ROS 地址，原目录的既有 devel 仅作为只读依赖。
不复制原目录构建缓存、私有配置、地图、模型、运动授权或运行进程。

原 `optimized.sh` 的目录保护仍保留：本新目录尚未迁移生产启停、J6M release/rootfs、
地图和私有配置路径，不能把旧目录的启动/部署入口改个路径后直接启用。
本次验证脚本仅用于隔离构建和模拟，不构成实车启动或部署入口。

原始失败与最终验证证据保留在 `validation/navigation_tuning_20261023/`，
插件合成场景细节另见 `validation/yaw_free_goal_simulation/`。
验证中遇到旧 devel 优先导致测试误加载旧几何模块/旧插件的问题，修正新工作树优先级后重跑；
旧失败日志保留。单点假区域夹具已更新为真实可接受的矩形，新增 C++ 测试入口也已补齐。

未部署到 J6M，未启停实车、发初值/目标/速度或修改授权；没有重新查询板端 current。
AGENTS 的 `20260914_095005` 仅为历史记录，不代表本轮实测状态。

最终验证：两个包的 Release 构建通过；覆盖包 222 项（162 状态机、26 几何、15 契约、
3 mux、8 新矩形/调参、8 Hybrid C++），Qt 105 项（52 契约、11 区域库、20 垃圾注册、
2 矩形、8 Qt 实例/回调、12 待清扫页）全部通过，0 错误/失败。
Catkin 显示 390 是嵌套计数汇总，独立叶子用例为 327，不把嵌套重复视作新增测试。
真实插件合成 ROS 14 项（新增运行调参 4 项、原无朝向导航 10 项）通过；
无实车、BPU、实际地图移动或云端验收。
最终日志为 `build_overlay_fixed.log`、`tests_tabs_corrected_final.log`、
`plugin_runtime_overlay_fixed.log`，逐项结果见 `results.json`。
新增页签导致旧页签列表断言失败，已更新期望列表并仅重跑该相关测试；旧日志保留。
`git diff --check`、Python 编译及验证脚本 `bash -n` 通过。

## 后续：永久保存与本机预览（2026-10-23）

用户随后明确要求关闭当前 Qt、打开算法调参 Qt，且每次修改后重启仍使用修改值。
新增后端持久化：

- Hybrid：`$ROS_HOME/autolabor_navigation/hybrid_tuning.json`；可由
  `/move_base/CoverageGlobalPlanner/tuning_state_file` 指定绝对路径。
- 转场 TEB：`$ROS_HOME/autolabor_navigation/transit_tuning.json`；可由
  `/coverage_manager/tuning_state_file` 指定绝对路径。
- 版本化文件写入临时文件、fsync、原子 rename；提交成功后才更新运行配置并应答。
  保存失败回显原值，不接受修改；损坏、缺字段或越界的历史配置阻止启动，避免静默恢复默认。
- 启动顺序为 launch/YAML 默认值 → 已保存值 → 注册运行调参服务。
  Qt 检查 `tuning_persistence_protocol=1`，旧后端不允许误报“永久保存”。
- 覆盖基础参数无需地图渲染/区域库就绪即可编辑，但仍要求静态地图模式、后端在线、状态新鲜、
  无活动任务/预览/并发操作；规划、启动和运动仍保留原地图/定位/授权门。

本次只打开独立离线调参会话：`scripts/navigation_tuning_preview.sh start`，
关闭为同脚本 `stop`，状态为 `status`。完整生产 `optimized.sh` 的目录保护保留。
预览在独立网络/PID 命名空间内运行，仅 lo、私有 master 11543，使用真实 move_base/TEB、
覆盖管理器和 Qt，地图/TF/零速里程计是合成数据，没有连接实车、相机或 CAN。
界面标题标明“离线预览（未连接实车）”，默认显示导航调参页。

预览持久目录为
`runtime/navigation_tuning_preview/ros-home/autolabor_navigation/`，基础参数文件为
`runtime/navigation_tuning_preview/coverage.yaml`，Qt 设置为同会话 `qt-config/`。
它们只在首次运行初始化，之后启动不会覆盖。预览值不会自动同步到板端；后续正式部署/启动
需单独适配并明确同步这些保存值，不能把本机预览当作实车参数已更新。

原正在运行的 Qt 属于 `j6m-dev-b-surround` 的 `j6m-b-surround-operator.service`。
已通过该分支自带 `operator_qt.sh stop` 正常关闭；其资源清理/恢复由原 supervisor 完成。
未修改该分支源码；没有启停实车导航主链或改运动授权。

验证：Release；覆盖/Qt 独立叶子用例333项全部通过（Catkin400含嵌套重复）；
真实后端7项过程检查通过，包括三组参数修改、后端进程重启后保持、以及真实 Hybrid
文件提交失败时保留运行值。测试数据位于 `validation/tuning_persistence_20261023/`，
测试目录与用户实际预览保存目录分离。

最终GUI实例新增无地图可调参、活动任务/已有预览/陈旧状态继续锁定的回归；9项Qt实例检查通过。
最终截图为 `validation/tuning_persistence_20261023/qt_tuning_final.png`，
当前Qt来自本分支，服务active；未修改原工作树、环视源码或板端release。
