# 智能语音动态区域语义同步与运行记录（2026-09-10）

## 结论

操作员此前确实从 `/home/slam/robot_j6m_ws_navigation_20260907` 启动了 Qt。失效原因不是
终端目录选错，而是当时动态区域语义修复只落在独立视觉工作树；导航工作树的 `0909`
分支仍保留旧提示词和旧计划校验。旧后端因此把“研究院前门”视为无法落到坐标的模糊
导航目标，并追问是定点还是覆盖清扫。

本轮已把智能语音相关修改完整同步到导航工作树并重新构建、完整重启。当前运行的 Qt 和
`/sweeper_ai` 均来自本目录的 `devel`，独立视觉服务已停止。

## 同步内容

- 保留 2026-09-09 已实现的声学伪触发过滤、WebRTC VAD、重复/低置信转写拒绝、自然停顿
  合句、孤立短片段等待补充，以及最近三轮上下文。
- 每次云端规划前通过 `list_saved_coverage_regions` 读取完整区域目录；目录改变时清除旧的
  区域上下文，避免沿用过期候选。
- 提示词描述地点主体、方位、自然同义表达和建筑后缀的通用匹配原则，不维护当前地图的
  固定别名表。提示词中的区域名称只作为可替换推理示例。
- 云端只能把用户口语落到当轮目录里的原始精确名称或 UUID。本地再次校验引用是否真实
  存在；未知、冲突或多候选时返回澄清，不执行计划。
- “到 A 区，在那里清扫”是一个覆盖清扫高层任务，只生成
  `start_coverage_cleaning(regions=[A])`；覆盖管理器自行规划到入口，不再额外生成模糊
  `navigate_map_pose` 或 `navigate_relative`。
- 清扫区域目录不是导航点目录。只导航、不清扫时，区域名称不能被推断成中心、入口或任意
  坐标；点到点导航仍要求用户明确给出 map 坐标/地图原点，或明确相对方向和距离。
- `submit_plan` 继续要求唯一一次完整提交；无工具能力的条件分支或必要参数缺失仍返回空
  steps 和澄清，不能删掉一部分动作后执行剩余计划。

核心实现位于：

- `src/sweeper_mcp/src/sweeper_mcp/agent.py`
- `src/sweeper_mcp/src/sweeper_mcp/region_catalog.py`
- `src/sweeper_mcp/src/sweeper_mcp/navigation_grounding.py`
- `src/sweeper_mcp/src/sweeper_mcp/tools.py`
- `src/sweeper_mcp/scripts/sweeper_ai_node.py`
- `src/sweeper_mcp/src/sweeper_mcp/voice_turns.py`
- `src/sweeper_mcp/scripts/whisper_asr_worker.py`

## 验证

本机 `./scripts/optimized.sh build` Release 构建通过。9 个 nose 文件按 catkin 的隔离方式
逐文件运行，共 164 项通过：8 + 21 + 32 + 34 + 9 + 16 + 5 + 22 + 17。
若把所有 nose 文件强行合并到一个解释器，nose 会把测试路径临时导入成 `sweeper_mcp`
并遮蔽 catkin 生成的 `sweeper_mcp.msg`；逐文件进程隔离后全部通过，属于测试运行器命名
空间限制，不是产品回归。

真实云端、当前真实区域目录做了 10 项计划预览，10/10 通过。关键结果：

- “请你到研究院前门去，在那里清扫一下”只返回一个
  `start_coverage_cleaning(研究院大楼前大厅)`，耗时约 1.17 秒。
- “去研究院前门清扫，然后去研究院侧门清扫”返回两个按序覆盖清扫步骤。
- “请你到研究院前门去，不清扫”不产生导航工具，要求补充精确坐标。
- “先到地图坐标(1,2)，再清扫A区”保留明确点导航和覆盖清扫两个步骤。
- 未知区域、模糊相对位移均返回澄清。

另以内存向目录注入与当前区域无关的新名称，6/6 通过：唯一的“研究院后大厅”和
“研发楼西侧大厅”可由相应自然说法匹配；同时存在“研究院后大厅/研究院后门厅”时要求
二选一；未知楼顶区域和只导航到模糊区域均不生成工具。这里的 `synthetic-*` 只存在于
测试进程构造的数据中，没有创建区域文件，也没有改真实地图配置。

证据：

- `validation/voice_region_cloud_20260910_0909.json`
- `validation/voice_dynamic_region_cloud_20260910_0909.json`

两组云端验证均只检查 plan，不连接 ROS 执行端，也不调用机器人动作工具。

## 当前运行状态

统一入口执行：

```bash
./scripts/optimized.sh restart \
  --map-set "$PWD/global_maps/map_sets/map_20260829_221335_gimp_cleaned"
```

重启退出码为 0，583 项结构化运行检查零失败；J6M、MID360、ZED、ROS 双机节点、Qt 地图
纹理和 AI 状态话题均通过。2026-09-10 18:44 的状态为：

- `autolabor-navigation-20260907.service`: active，MainPID 122771；
- `autolabor-vision-0908.service`: inactive；
- Qt PID 130311，AI PID 130307，均来自本导航工作树；
- 地图 READY，覆盖 IDLE、无活动批次；
- 新会话为 `WAITING_INITIAL_POSE`，尚未定位；
- 语音授权、解析授权、控制授权均为 false，智能监听为 false；
- ASR 默认 `medium`/CUDA，尚未因授权而加载；
- 连续读取 3 条最终 `/cmd_vel`，线速度和角速度均为 0；
- J6M current 仍为 `20260909_003347`，本轮 AI/Qt 修改不需要也没有重新部署 J6M。

本轮没有发送初值、导航目标、覆盖任务或非零速度。要做语义预览时，可在 Qt 中只打开
“语音授权”和“解析授权”；保持“控制授权”关闭即可只看识别文本与云端计划。实际运动前
仍必须由操作员在本次冷启动会话按真实位置设置初值，并重新核对现场安全条件。

本轮修改保存于本地分支 `0909`，未推送远端。
