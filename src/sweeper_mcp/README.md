# sweeper_mcp

NVIDIA 端的 Qt 授权 AI 规划与 MCP 控制包。云端模型只提交一份完整、有序的工具计划；
本地先对计划和全部参数做严格校验，再逐步调用已有 ROS 子系统，并等待当前步骤到达终态后
进入下一步。任一步失败、授权撤销或 Qt 心跳丢失都会停止剩余步骤。

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
能力令牌，不能由普通 MCP 客户端绕过。关闭 Qt、心跳超过 3 秒、撤销解析或控制授权都会
撤销控制并请求取消 AI 所有的活动任务。

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
  `/fod_navigation_mode/set_fod_enabled` 的视觉伺服安全仲裁。
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
cd /home/slam/robot_j6m_ws
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
单独调用 `/sweeper_ai/set_smart_voice` 开启连续监听；它与单次录音互斥，每句话先发布本地
`TRANSCRIPT`。worker 以本地能量 VAD 判断起句，在约 `0.8 s` 静音后形成完整句子，再进行
一次当前所选模型的批量识别；它不输出边说边更新的 partial token。解析授权关闭时不会发送
云端；开启后进入最多 8 条的 FIFO，严格等待上一条
云端计划及工具任务完成后再处理下一条。队列中的口令超过 30 秒会作为陈旧指令丢弃；关闭
智能语音、撤销语音/解析授权、Qt 心跳丢失或取消任务都会清空尚未派发的句子。AI 控制门
每次变化也会清空旧队列，避免先前预览口令在之后获得执行权限。

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

正常 NVIDIA 启动由 `scripts/nvidia_ui.sh` 同时托管 AI 节点和 Qt：

```bash
roslaunch sweeper_mcp ai_control.launch
```

仅验证界面和分解时可使用独立 ROS master，并设置 `SWEEPER_MCP_BACKEND=mock`。mock 后端
只在内存中模拟工具结果，不会连接或控制车辆。ASR、Qt、AI 节点和 MCP 客户端全部运行在
NVIDIA；这些文件的修改只需要本机构建，不需要 `deploy_j6m.sh` 或切换 J6M release。

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
