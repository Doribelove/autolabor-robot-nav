# N06 四静态父几何 lookahead 配对准备（零执行权限）

已准备 D04 r05 的四个静态父几何，各用同一拟议 seed 对照两种解析方法，共 4 块／8 回合。顺序为 control→treatment、treatment→control、control→treatment、treatment→control；seed 仅提案 `263003001–263003004`，没有登记、保留或消费，没有 release、attempt ledger 或 ROS/Gazebo 运行。

输入：`scenes/navigation_n06_static_prepared_r01/`。每回合 contract、展开 URDF、world、gate 期望、fixture.launch 与零权限 schedule 在 `contracts/navigation_n06_prepared_r01/`。未来 `runs/<run_id>` 仅作为 launch 的输出路径文字出现，准备脚本未创建这些目录。

## 唯一方法因素

- 对照 `astra_analytic_route_v1` 使用冻结 `astra_route_coordinated_runtime.py`，默认 lookahead `0.7 m`。
- 处理 `astra_analytic_lookahead_1p2_n06` 使用 root 预先创建的 `astra_route_lookahead_runtime_n06.py`，lookahead `1.2 m`。此文件未由准备阶段修改。

逐字节替换审查确认处理节点只改变说明文字、方法身份检查和 `RouteConfig.lookahead_m`。四组实际 map/URDF 构造的 core identity 也只在该 config 字段不同；候选生成器、backup mask、解析选择、命令与重验函数仍为同一个冻结 core。观察 schema、8 个动作及其语义定义保持原版；具体预测轨迹因 lookahead 改变而改变，core semantics SHA 因而不同。不加载 learned actor，也不使用 N05 preview eligibility。源导入图中没有 N05 preview 节点或 selector。

N06 的 coordinator、collector 和 evidence 是独立副本，仅改变两方法白名单、evidence schema 和对应源资源引用。数值 gate、生命周期、唯一公共 `/cmd_vel_navigation` 所有权、实际 base goal 源时裁决保持父版。N05 的 event actor 只读验证依赖保留；本批 actor roster 为空，world 没有 actor 插件。这不是引入 N05 选择器。

## 共同输入和离线验证

D04 四个静态 parent 的碰撞几何、map.pgm、map.yaml、common_route.json 及合格 `raster_witness.csv` 均逐字节复制。world 只加共同的已冻结 N01 物理观测 shell（ground、光照、P03 metadata 与 dense observer），相机改 N06 命名空间。由完整 N06 world 重新导出的四份栅格仍与 D04 精确相同；成对 world 和实际 P05 Kd6 URDF 字节一致。

15 项作者测试通过。8 个 exact `roslaunch --dump-params` 和 resolved node/参数检查通过，确认 paused Gazebo 参数、拟议 seed、P05 Kd6、方法脚本、同一 bundle、22 秒 cap、唯一命令链、collector `required=true` 和 recorder topic roster。这里只解析 launch，没有启动节点。所有实际节点路径都可在独立 Astra workspace 环境中解析。

首次 launch 检查脚本把 `remap_args` 的 list 对象与 tuple 直接比较而失败，原脚本、第一单元 dump 和失败日志完整保留。r02 检查只规范化容器类型，输入、源码和 launch 未改；随后 8/8 通过。原包名规范与 xacro `--inorder` 提示也保留。

## 后继执行仍需的条件

D04 作者 freeze 被精确绑定，但 prepared 文件不会自动因独立审查完成而获得权限。root 需要另外绑定 D04 split/raster 与 N06 独立审查、完成实际 runtime/转换/ELF 闭包和拟议 seed 的全局冲突检查，再决定是否建立新 release。当前没有实现可执行矩阵 manager，也没有调用任何旧 runner。

D04 slot001 原 native witness 的栅格负例 `−0.0117327 m` 继续保留；本批复制的是另一个预声明并合格的 raster witness。导出 A* guide 的正静态包络余量不证明 Ackermann 折线转弯、实际跟踪、22 秒完成或动态安全。四个静态父几何只能作失败保留的配对开发诊断；不得据此自动晋级六族验证或性能确认。`execution_authorized=false`，`allocated_budget=0`，`seeds_registered=false`。
