# N05 场景派生源码：未执行 prepare

新源码 `workspace/scripts/astra_gazebo/prepare_navigation_n05_scenes.py` 从冻结 N04 preparer（SHA `7cd9e1c87bdc56f3588713208d0bafe4eddf5758df663068af3d3abe8f345af7`）派生；N04 文件保持不变。此交付只有源码/测试/审计，**没有生成 scene 目录、地图、几何 QC、matrix 或 registry 条目**。

后续 API 与旧 preparer 相同：`derive(parent, world_text, seed)`、`static_input(world, scene)`、`bind_gate_identity(world_text, run_id, seed, method)`、`build(source, output, registry, reference=REFERENCE)`。只有 root 后续审阅并主动调用 build 才生成离线输入。默认 source 仍是 `scenes/development_a02_live_v2`，不是 N04 已消费 world；六个旧 parent × 两个旧 actor 参数重复保持相同。`build` 保留旧完整几何/map/QC 流程，当前未调用。

精确新增/变化：

- 方法 roster 严格为 `astra_analytic_route_v1` 与 `astra_analytic_preview_eligible_n05`。
- actor 文件名 `libastra_event_actor_n05_plugin.so`；模板标记 `UNBOUND_N05_RUN/METHOD`；scene/schema/input_variant/environment 使用 N05 身份：`astra_navigation_n05_development_scene_v1`、`*_n05_event_actor`、`astra_n05_physical_p05_event_actor_50ms`。
- 拟定 simulator seeds `263002001–263002012`，逐条取自已冻结设计 `contracts/navigation_n05_preview_selection_design.json`（SHA `959b214b3f4a10da276ee46ce9c319c9f5ea8001551f9dc4626b97f9714baefc`），不是本组件分配。actor 的旧参数/response seeds 不改。
- 新 `load_design` 校验该文件 SHA；`validate_design_data` 检查 24 个 exact run_id/method/order/seed/attempt、12 blocks、六 parent×两 replicate 和未授权/零 allocation 状态。`derive` 检查 seed 对应的 parent identity；`bind_gate_identity` 在 actorless world 也先检查真实 proposed schedule 身份。actor world 再检查一次性 UNBOUND 标记与 seed。
- manifest/report 增加 paired method roster、设计 SHA、冻结 N04 preparer SHA；保留所有源字节检查、seed registry 只读碰撞 guard和旧输出不可覆盖。修复旧 `build` CSV 读取未关闭 stream 的 ResourceWarning，仅改资源关闭方式。

保持不变：

- parent geometry ID、collision/visual XML、actor start/end/conflict/policy/speed/response/gate_delay、0.72m actor shape；50 ms event transport、零补积分、既有 world evidence plugins。
- spawn −5.4、gate −4、goal 5、22 s cap；物理 P05/backend 参数由原 reference几何及 root后续run合同绑定。
- static actor omission、static map/export/raster rectangles、1.0 m guide生成规则；`ReferenceRobot`、`reduced_clearance`、`witness`、`scene_qc`、`static_input`等 AST 与原版本逐项一致。
- camera 原 topic/pose/2560×1440/rate，world name与模型名保持原样；不改视觉命名伪造新布局。

15 个 unittest 覆盖所有 12 原 parent XML，原/新 world 经**仅 plugin/UNBOUND 名称和 seed**规范化后字节一致；scene 经声明 metadata 规范化后一致。每个 block 的两方法绑定均通过 root 新 `event_actor_contract_n05.world_actors`。反例含错误 parent、bool/旧 seed、改 actor policy/path、错误方法/顺序/attempt、改设计字节、actor第二次绑定、registry碰撞先于geometry构造拒绝及已有输出保留。只读 XML/metadata；未跑 901点 witness、地图构建、full geometry 或计时压力。

边界必须由后续 manager承担：actorless world没有 event metadata可写，`bind_gate_identity`可验证 request 属于 exact schedule，但输入 world 属于哪个 block仍须由 scene/parent/world SHA闭包判定；不能凭这个函数单独声称无actor world已内嵌身份或一次性执行。场景 `runtime_identity_bound=False` 与 `execution_authorized=False` 保留。一次性执行、实际 seed registry reservation、N04终态/进程隔离和release均不在此源码权限内。

后续准备完成需重新核对实际输出 source/pre/post、共同geometry/map、per-run identity及 actor metadata 新库闭包，不能用本次轻量 pass代替完整输出审查。六 parent重复开发诊断不构成新独立几何机制晋级、性能确认或学习证据。
