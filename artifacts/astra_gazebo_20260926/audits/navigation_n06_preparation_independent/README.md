# N06 准备包独立审核

通过范围为已冻结的 4 个静态父几何 × 2 个方法、8 份离线准备契约／launch 和唯一 lookahead 因子。不是执行 release；没有登记或消耗种子，没有启动 ROS、Gazebo 或训练。

作者报告 `0a9dfaee...` 与 292 个封存文件已逐个复验。另绑定 D04 r05 独立报告 `a40a0f4f...`，保留作者准备包内“独审待另行绑定”的历史字段，不回写旧文件。

独立检查结果：

- 新 collector 相对 N05 只改 helper import；新 coordinator 核心只缩小 method whitelist；ROS coordinator 只改核心 import。helper 仅改 schema、两个方法身份和对应四个本地 source 路径。旧 activation、goal、timeout、deadline、物理收尾与证据流程未变。
- 新 treatment 节点与旧 coordinated 节点进行穷尽文本差异比对：仅文档、method guard 和 `RouteConfig(lookahead_m=1.2, ...)` 三处。实际 `RouteConfig` 构造差异只有 `.7→1.2`；原 candidate generation、mask、backup-only analytic ranking、command 和传感器时序代码保持原样。候选数值变化可由 lookahead 因子产生，不能把“代码不变”误称数值轨迹不变。
- 每块实际构造两套 `RouteAwareRuntime`，除 lookahead 外整个 identity 相同，两种 semantics hash 按预期不同；actor 都为 `None`。局部源依赖图无 N05 preview selection/runtime。训练和旧权重没有加载。
- 八行 schedule 是固定四对，方法顺序交替，attempt=1。每对共享提议 seed、world、URDF、map、route、manifest、collector 条件；除 method/run_id 和同内容的 per-run 路径外，契约逐字段一致。
- D04 四静态原输入、全部 `raster_and_guide` 文件、原负例和合格 raster witness 逐字节复制。完整 world 对每个原静态 model 做 XML 结构对比；仅加入继承的共同 ground/camera/physics/evidence shell，camera 没有碰撞。shell 的其余物理及插件配置原样匹配 N01 参考。作者 15 tests 中包含完整 world 再导出 raster 与 D04 逐字节相等，本次已独立复跑通过。
- 用 ROS XML parser 重新展开全部 8 份真实 fixture（仅参数命令执行离线 xacro，没有节点启动），参数集合与作者保存 `roslaunch --dump-params` 输出完全相同。检查实际源码入口、私有命令 remap、唯一公共 relay、required collector、paused Gazebo seed 参数、P05 Kd6 URDF、22 s cap、完整 bag topic 清单和空 checkpoint 参数。节点路径实际解析到新 workspace，未借用稳定 workspace。
- 当前 seed registry 中未见 263003001–263003004 或 N06 run ID；八个 run 目录均不存在。registry 在审查前后字节一致，快照保存为 `seed_registry_at_review.csv`。这是本次时间点的无分配证据，不能代替将来全局碰撞审计／授权。

15 个作者测试在明确 `.venv/bin/python` 环境独立复跑通过；独立 `check_preparation.py` 的 10 项实际资源／配对／展开检查全部通过，最终输出 `check_attempt04.json`。

独审脚本的三个失败保留：初版缺少 `else` 分隔空格的 SyntaxError；第二版替换名包含于 `astra_` 名导致计数错误；第三版拿默认 roslaunch config 自动加入的 `/rosversion`、`/rosdistro` 与不加载 roscore 的 `--dump-params` 输出比较。修复仅在独审脚本，最后按本机 `param_dump.py` 使用 `ROSLaunchConfig + XmlLoader`，没有忽略参数或修改被审文件。每个失败的原源码／日志保留。

后继仍需 root 实现并审查 exact matrix manager，绑定实际转换 SDF、转换器／native ELF、runtime closure、资格 decision 和完整 source review，再审种子并创建独立 release。准备包目前没有这些运行产物；本审核不证明 collector 在新实际场景成功、不证明动态安全／22 秒成功、不证明 1.2 m 优于 .7 m，也不支持六族 promotion。D04 slot001 原 native witness 的 −.01173 m 栅格负例保留，后继要用实际合格 `raster_witness.csv`；guide 的折线转角没有 Ackermann 可行性资格。

`resources.json` 只封稳定输入与审查产物。易变的全局 seed registry 只以快照封存，原路径及当时 SHA 记录为观测，不能将它列为将来执行期必须永远不变的源依赖。全部 runtime 源、作者准备包和 D04 文件未修改。
