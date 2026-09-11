# sweeper_mcp

NVIDIA 端的 Qt 授权 AI 规划与 MCP 控制包。云端模型只提交一份完整、有序的工具计划；
本地先对计划和全部参数做严格校验，再逐步调用已有 ROS 子系统，并等待当前步骤到达终态后
进入下一步。已识别的步骤失败或显式授权撤销会停止剩余步骤；Qt短时卡顿不再自动取消任务。
2026-09-12已继续修复位姿采集触发、自然停顿合句及视觉误报完成三个问题，详见
[修复与验证](../../docs/AI_CONTROL_REPAIR_20260912.md)。历史失败保留在
[原模拟报告](../../docs/AI_CONTROL_SIMULATION_20260912.md)，不是最终通过项。
最低原话“清扫A区后回到当前点进行视觉清扫”要求先查询记录一次map位姿，再覆盖A、
返回记录点、开启视觉伺服。新增视觉完成协议需要配套部署J6M；未部署时AI拒绝启用视觉，
不能仅重启NVIDIA就进行完整链测试。完整15个工具、参数及控制链已同步至
[项目README的AI章节](../../README.md#ai-语音控制与本地-asr)。

ASR 使用 NVIDIA 本机的 OpenAI Whisper `small`、`medium` 或 `large`（即
`large-v3`），默认 `medium`。`sweeper_ai` 管理一个隔离的
JSON-lines worker 子进程，worker 只接触显式指定的 ALSA 设备、本地 PCM 和 checkpoint，
没有 ROS、云端密钥、Qt 会话令牌或 MCP 控制能力。旧 PyAudio/键盘话题、GPS 仿真和 TCP
清扫逻辑仍保持删除。

## 授权边界

三个授权每次 `sweeper_ai` 启动都为关闭：

1. 语音输入授权：允许本地 ASR；授权后仍须显式开始/停止单次录音，或另行启用智能语音
   连续监听。手工文本不需要它。撤销时立即停止采集/识别、关闭智能监听并丢弃旧
   capture/session 的迟到结果。
2. AI 语义解析授权：关闭时不向云端发送文本、执行结果或总结。
3. AI 控制授权：关闭时仍展示校验后的计划，但所有会改变机器人状态的工具都不执行。

Qt 与节点共享每次启动随机生成的私有会话令牌。MCP 变更类工具还需要节点内部生成的第二个
能力令牌，不能由普通 MCP 客户端绕过。2026-09-11 已删除“Qt 心跳超过3秒就撤销全部
授权并取消任务”的AI层看门狗。原heartbeat接口仅保留为一次性令牌认证握手，Qt确认会话
已建立后不再周期调用。ui_session_alive兼容字段表示会话已认证，不再表示3秒心跳租约。
显式停止/撤销解析或控制授权仍取消AI所属任务；关闭Qt仍由统一生命周期关闭AI节点与
整套程序，AI节点shutdown继续撤权、关闭录音并取消任务。三门每次冷启动仍默认关闭。

AI 授权不替代项目已有的实体急停、定位、CAN、避障、主运动门和 FOD 运动门。此包从不
直接发布 `/cmd_vel`，也没有释放急停或绕过安全门的工具。

## 工具与现有 Qt 功能的对应关系

- `navigate_relative`：读取新鲜 `/Odometry`，按车辆当前朝向换算相对前/左/转角。
  NVIDIA 在提交前生成唯一的 `sweeper-ai-<uuid>` GoalID，经
  `/navigation_goal/action_request` 交给 J6M 安全桥，再由桥在 J6M 本机转发到
  `/move_base/goal`；不会从历史“最近目标”推测所有权。
- `navigate_map_pose`：使用任意有效的 `map` 绝对 x/y/yaw。MCP 只等待一次锁存静态地图，
  不会按接收时间让它过期；每次执行都将本地完整 OccupancyGrid SHA-256 与新鲜的
  `/coverage/status.map_digest` 精确比对，并验证 ICP `LOCALIZED`、地图原点平面旋转、
  目标范围和占用栅格，发布前再复核同一地图代际。
- `start_coverage_cleaning`：调用 `/coverage/start_batch`，只允许引用 Qt 已保存的区域名称
  或 UUID；不接受模型生成的临时多边形。NVIDIA 在调用前生成
  `coverage-batch-<32hex>`，J6M 原样把它作为 `batch_id`，同 ID/同参数重试不会重复启动，
  同 ID/不同参数会拒绝。
- `pause_coverage`、`resume_coverage`、`skip_coverage_region`、`cancel_coverage`：复用覆盖
  管理器服务，并核对任务是否属于当前 AI 会话；取消使用
  `/coverage/cancel_batch` 精确指定 AI 的 batch ID，不会取消 Qt 的其他批次。
- `start_spot_cleaning`、`stop_spot_cleaning`：复用
  `/fod_navigation_mode/set_fod_enabled` 的视觉伺服安全仲裁，要求完成协议1及新会话ID；
  退出只能针对仍匹配本AI会话的新鲜状态。
- 其余五个 `get_...`/`list_...` 工具只读，不需要 AI 控制授权。

普通导航提交后必须同时看到同一 GoalID 的 action 回显和唯一状态，目标 frame 与完整平面
位姿也必须一致；任一闭环缺失都进入撤销收敛阶段，而不是把其他 Qt 目标误关联给 AI。
精确撤销通过 `/navigation_goal/cancel_request` 送到 J6M 本地转发；若撤销先于目标到达，
桥会用 `/navigation_goal/cancel_ack` 明确证明该 ID 从未被转发。除此之外必须等同 ID 安全状态得到
确认前，后端保留句柄、阻止下一目标并持续重试。`LOST` 只表示状态未知，不能当作车辆已经
停止。J6M 还持有 `/navigation_goal/ai_heartbeat` 精确 GoalID 租约；只有 Agent 正在轮询
监督同一目标时 NVIDIA 才会续租，单独存活的 ROS timer 不能延长目标。NVIDIA 控制后端
失联、桥重启、暂停或覆盖接管时，J6M 会持续撤销该 AI GoalID。

覆盖启动响应丢失不再等同于“启动失败”。后端保留预先生成的 batch ID，并立即调用
`/coverage/cancel_batch`：若 J6M 尚未提交，则建立 cancel-before-start tombstone，阻止
迟到线程提交；若已提交，则只取消该批次并持续等待其终态、move_base 精确终态、规划器
恢复、TEB 恢复和 owner 释放。上述闭环完成前 AI 执行锁不会释放，也不会开始下一步。

当前工具总数为 15：5 个只读工具（`get_robot_status`、`get_navigation_status`、
`list_saved_coverage_regions`、`get_coverage_status`、`get_visual_servo_status`）和 10 个
变更类工具（两种导航、取消导航、视觉定点清扫启停、覆盖启动/暂停/恢复/跳区/取消）。
不存在 GPS、经纬度、直接速度或急停释放工具。

不存在 GPS、经纬度、旧清扫设备 TCP、急停释放或直接速度工具。连续语句最多拆成 8 步；
超限会整单拒绝，绝不截断执行。

“地图原点/地图坐标原点”固定表示 `map` 位姿 `(0, 0, 0°)`；其他坐标可由用户明确给出，
例如“导航到地图坐标 x=5.2、y=-3.1、朝向 90 度”。当前没有命名导航点库，因此“基地、
充电点、起点”等词若没有精确坐标，云端计划必须询问而不能自行猜测。

## 区域配置路径

区域库权威位置随当前 map-set 保存：

```text
<STATIC_MAP_SET>/coverage_regions/<STATIC_MAP_SOURCE_MODE>/regions.json
```

例如 A区、B区使用当前地图集下的 `coverage_regions/fused/regions.json`。文件中的地图完整
SHA-256、来源模式和 map-set 规范路径必须全部匹配。旧的
`global_maps/coverage_regions/v1/<digest>/<mode>/regions.json` 只作为 Qt 一次性迁移回退，
不是新的写入位置。

## 本地可切换 Whisper ASR

首次安装仅在 NVIDIA 执行：

```bash
cd /home/slam/robot_j6m_ws_optimized_20260905
bash ./scripts/install_whisper_asr.sh
```

安装器在 Git 忽略的 `runtime/asr` 下创建独立 CUDA venv，将已知 Jetson NVIDIA PyTorch
wheel 校验后复制为安装种子，固定 OpenAI Whisper commit
`5f86d1d86363843179951550570367b37c5d6f78`。官方三套 checkpoint 都先写入 `.partial`，
SHA-256 匹配后才原子改名为：

```text
runtime/asr/models/small.pt
runtime/asr/models/medium.pt
runtime/asr/models/large-v3.pt
```

运行中的 worker 不联网下载；模型文件不存在、哈希不符、CUDA 不可用、`arecord` 不可用、
自动枚举不到实体采集端点、显式设备打不开或模型环境异常时，`asr_available=false`，
Qt 录音按钮保持禁用。`input_device` 为空或为 `auto` 时只读枚举 ALSA capture 端点并
优先选择 USB 麦克风；`auto_null.monitor` 和 Jetson APE fabric 端点不作为实体麦克风。

标准单次交互为：确认语音授权，显式开始录音，说完后显式停止并识别。智能语音模式还需
单独调用 `/sweeper_ai/set_smart_voice` 开启连续监听；它与单次录音互斥。worker 结合能量与
WebRTC 人声判断起句，约 1 秒静音切出音频片段，另以人声占比、平稳宽带噪声、识别置信
信息及重复文本拦截伪语音。低能量复述提示词、NaN、纯标点不作为指令接收。
它不输出 partial token；每段识别后发布本地 `SMART_ASR`，节点先合成草稿，再发布
`SMART_TURN`。查询/澄清补答等待1.5秒；运动/清扫动作句等待3秒；含“先/然后/之后”
等连续动作线索或未完表达等待4秒。完整前半句也保留续句窗口，不是只检查句尾连词。
计时使用采集时钟，并等待后续录制及已采集片段全部识别完毕；无关联短词 6 秒过期，
语气词被忽略，“停止/取消/暂停”等完整短指令不受最小字数限制。可说“说完了”结束草稿。
解析授权关闭时不会发送云端；开启后完整轮次进入最多 8 条的 FIFO，严格等待上一条
云端计划及工具任务完成后再处理下一条。队列中的口令超过 30 秒会作为陈旧指令丢弃；关闭
智能语音、撤销语音/解析授权、节点关闭或取消任务都会清空尚未派发的句子。AI 控制门
每次变化也会清空旧队列，避免先前预览口令在之后获得执行权限。

规划器保留最近 3 轮、60 秒以内的对话，支持回答上一轮的区域问题及解析指代。
待补充计划显式返回 `needs_clarification=true`、空 `steps`，`understood_sequence` 仅为
动作理解草稿，Qt 显示“待补充，未执行”；不能把不支持的条件分支静默改成无条件动作。
“查看/搜索周围有没有垃圾，有垃圾就清扫”不是外部条件分支，而是现有 FOD 视觉伺服
自身的目标发现与接管语义，直接规划一次 `start_spot_cleaning`；无满足条件的新鲜目标时
由视觉后端拒绝或结束。
历史已提交任务不会作为待执行步骤重放。停止监听、撤权、会话失效会清空草稿与上下文；
规划时发现区域目录变化也清空历史。格式错误最多重试一次，只重试云端规划。

“去某区域，在那里清扫”是一条区域覆盖任务，覆盖系统负责入场，不额外拆出导航或视觉
定点清扫；单独“去某区域”也不能从区域名称猜中心/入口坐标。区域自然语言匹配不维护
“前门/侧门”等固定别名。每次规划会重新读取当前地图的完整区域名称/UUID目录，由云端
结合地点主体、明确方位、方位同义表达、建筑/功能词省略和上下文指代做通用候选匹配；
提示词中的具体区域只用于解释推理方法。地点主体和明确方位为硬约束，门/门厅/大厅等
功能词为软约束。唯一候选时计划必须输出目录中的原始精确名称或UUID；存在多个合理候选时
要求操作员确认。本地 `region_catalog.py` 在所有步骤执行前再次核对本轮目录，拒绝编造、
歧义、重复或截断目录，并额外拦截同地点同方位的兄弟候选被按目录顺序任选；目录变化会清空
旧对话候选。因此正常新增区域无需改代码或重启AI，
会从下一次请求起参与匹配。

AI计划中的导航坐标/相对偏移还必须通过用户原文依据校验：支持地图原点、明确分隔的
map X/Y/可选朝向，以及“向前两米”等明确相对表达；单独说“原点/零点”也默认地图原点。
模糊距离、区域名称或模型自行生成的坐标不会作为点位依据。每个新请求（独立短停止/取消
等除外）在进入规划时通过get_robot_status(capture_map_pose=true)尝试只读记录新鲜、
已定位的map->base_link位姿及地图摘要，不依赖返回关键词，也不使用camera_init里程计。
该快照在云端请求和任何动作之前冻结，返回时保留当时朝向，绝不在覆盖完成后重新取点。
语音以合句后本轮请求进入规划时为准，不声称能回溯录音开始前/排队前的历史位置。
澄清补答可保留本轮快照；新独立请求不复用已预览或执行的快照。未保存的“刚才位置”、
“我现在的位置”（人的位置）、未定位或地图不一致仍需澄清，不得用机器人位置冒充。
地图切换会阻止该计划继续；返回导航携带expected_map_digest，在目标提交前再次核对。

例如“先清扫某区域，再返回现在的位置，进行视觉清扫”使用动态区域目录依次规划
start_coverage_cleaning → navigate_map_pose(冻结的返回点) → start_spot_cleaning。
本地检查不得漏掉返回或后续视觉步骤；整份计划通过后才允许执行，拒绝前不执行任何步骤。
执行器仍等待覆盖完整结束并释放所有权、同一导航GoalID成功后才开始下一步。
视觉状态携带`ai_completion_protocol=1`和`visual_session`。启动需确认新session ID，
完成需同ID、比启动前更新的完成序号、COMPLETE、stopped=true、active=false、仲裁恢复导航
模式；缺任何一项均不能放行后续步骤。中止/手动退出/旧回执不能冒充完成。
回执由J6M原停止确认提交点生成并跨DISABLED保留，仅新增结果元数据，不放宽5cm保护。
启动响应丢失时尝试找回本次会话；归属/退出不明则保留占用并阻止新的AI运动，状态缺失或
已换成其他会话时不擅自全局停视觉。泛指“去区域清扫”且无区域依据时整单要求澄清。
关键REQUEST/TRANSCRIPT/PLAN/PLAN_REJECTED/STEP/AUTH/FINAL事件现在按request_id写入
本地AI/ROS日志，含步骤参数与拒绝原因（密钥形态及本地路径脱敏），不记录音频或私有配置。

控制授权关闭时计划仍只预览，开启后也必须通过 MCP 能力令牌和项目原有运动安全门。手工
文字输入完全绕过 ASR 和语音门，但不能绕过解析门或控制门。

## 配置与启动

真实本地配置为忽略 Git 的 `config/sweeper_mcp.yaml`，必须为 `0600`。节点直接读取该文件，
禁止 `rosparam load`，避免把云端密钥上传到网络可见的 J6M ROS master。ASR 默认配置为
`model: medium`、`device: cuda`。Qt 通过 `/sweeper_ai/set_asr_model` 在
`small / medium / large` 间切换；切换会替换隔离 worker，且只允许在录音、监听和识别均
空闲时进行，失败会保留原 worker。`input_device: auto` 或空值启用自动发现，也可填写
稳定 ALSA 标识；worker Python 和模型路径默认指向 `runtime/asr`，可通过 NVIDIA 本地
环境变量覆盖。

新增 `asr.smart_segment_silence_ms`（默认 1000）替代旧 `smart_silence_ms` 的节点配置，
旧字段不再决定新缓冲模式的切段；worker 内部协议仍使用 `smart_silence_ms`。
`turn_quiet_s`/`turn_command_quiet_s`/`turn_incomplete_s`/`turn_fragment_ttl_s`默认1.5/3/4/6。
超过有界续句窗口仍可能分轮，“说完了”可主动提交，停止/取消不增加此动作等待。
ASR 独立环境新增 `webrtcvad==2.0.10`，安装脚本及静态检查已同步；缺依赖时显式报错。

正常 NVIDIA 启动由 `scripts/nvidia_ui.sh` 同时托管 AI 节点和 Qt：

```bash
roslaunch sweeper_mcp ai_control.launch
```

仅验证界面和分解时可使用独立 ROS master，并设置 `SWEEPER_MCP_BACKEND=mock`。mock 后端
只在内存中模拟工具结果，不会连接或控制车辆。ASR、Qt、AI 节点和 MCP 客户端全部运行在
NVIDIA；仅这些文件的修改通常只需本机构建。但本次新增J6M视觉完成回执，必须配套部署
`autolabor_fod_control`后再统一启动。旧板端不支持协议时AI拒绝发视觉启用请求。

## 离线验证

```bash
python3 scripts/test_mcp_protocol.py
python3 scripts/test_agent_planning.py
python3 scripts/test_asr_authorization.py
python3 scripts/test_ros_backend_goal_ownership.py
python3 scripts/test_ros_backend_map_navigation.py
```

测试覆盖工具目录、旧 GPS 工具移除、能力令牌、严格参数范围、计划预览、连续步骤顺序、
失败停步、授权撤销、8 步原子上限，以及导航/覆盖操作 ID 在并发提交、响应丢失和精确取消
时的所有权收敛，均不需要 ROS、硬件、网络或 API 密钥。
