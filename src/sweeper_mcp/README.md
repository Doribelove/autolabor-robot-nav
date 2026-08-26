# sweeper_mcp —— MCP 服务 + LLM 语音自动执行

面向智慧环卫无人清扫车的 **「语音 → 大模型 → 自动执行」闭环**。

- **MCP 服务端**：把清扫车的真实操作（导航/清扫/急停/状态查询）封装成 MCP 工具，走 `stdio + JSON-RPC` 协议，**纯 Python 3.8 标准库手写**（本机唯一 Python 是 3.8，官方 mcp/fastmcp SDK 需 3.10+，且 server 要操作 ROS 必须用 py3.8 的 rospy）。
- **Agent Host**：语音识别文本 → 云端大模型 DeepSeek（`tools` 参数做 function calling）→ 自主决定调用哪个 MCP 工具 → 工具在 ROS 上执行 → 结果反馈 → 输出最终答复。

**整体替换了旧 `ai_task_decomposition` 的「LLM 输出 JSON 任务清单 → 发布 TaskList」链路**（该链路全仓无执行器消费，是断的）。旧包已完全废弃（2026-08-17）：原 ASR / 键盘模块已迁移合并进本包的 **`voice` 子模块**（见下「目录结构」），不再有跨包引用。

**Agent 两阶段循环（2026-08-19，解决"多指令先后叠加 + 无终端交互"）：**
1. **规划**：大模型把整句指令拆成「按顺序执行的工具调用列表」（`submit_plan`），终端打印拆分结果（`指令1: navigate_pose(x=0,y=0,yaw=0) ← 回到原点` / `指令2: ...`）。
2. **执行**：按顺序逐条调用工具；对异步导航工具自动轮询 `navigation_status` 并在终端实时输出状态变化（`active → succeeded/aborted`），某一步完成后自动执行下一条，全部完成后生成总结。

终端会完整呈现「规划拆分 / 每步工具调用与返回 / 导航状态变化 / 完成总结」——即指令发出后 mcp 端与大模型的双向交互全程可见，导航异常（aborted）或成功到达都主动输出，不再等用户主动询问。

## 1. 架构

```
 语音本地whisper识别
        │
        ▼
┌─────────────────────────────┐   OpenAI 兼容 tools 参数
│ Agent Host (Py3.8)          │ ──/chat/completions──► DeepSeek (deepseek-v4-flash)
│  agent.py + llm.py          │ ◄──────tool_calls─────
└────────────┬────────────────┘
             │ stdio JSON-RPC（每行一条 JSON）
             ▼
┌─────────────────────────────┐  tools/list, tools/call
│ MCP Server (stdio)          │
│  mcp_sweeper_server.py      │
└────────────┬────────────────┘
             ▼ tools 分发
┌────────────┴──────────────────────────────┐
│ ros_backend.py (rospy)    │ sweep_backend.py (TCP) │
│  发布/订阅/服务           │ 复用 SweepDeviceControl│
└────────┬──────────────────────────────────┘
         ▼
  move_base / /m2_driver / gps_module / /fod_navigation_mode / 清扫装置(192.168.1.197:50003)
```

## 2. 目录结构

```
sweeper_mcp/
├── package.xml  CMakeLists.txt  setup.py
├── config/sweeper_mcp.yaml          # 非敏感参数（话题名/TCP/LLM 配置）
├── launch/voice_agent.launch        # 语音 Agent ROS 节点 + 键盘模拟输入
├── src/sweeper_mcp/
│   ├── mcp_jsonrpc.py               # MCP 服务核心：stdio + JSON-RPC 帧与 dispatch（纯 stdlib）
│   ├── tools.py                     # 10 个工具注册表（schema + handler 绑定）
│   ├── mock_backend.py              # 离线测试桩（MCP_BACKEND=mock）
│   ├── ros_backend.py               # 真实 ROS 后端（懒加载 rospy）
│   ├── sweep_backend.py             # 清扫装置 TCP 后端（复用 SweepDeviceControl，屏蔽其 print）
│   ├── voice/                       # AI 语音子模块（2026-08-17 由 ai_task_decomposition 迁移合并）
│   │   ├── __init__.py              # 统一导出（AudioRecorder / WhisperRecognizer / list_input_devices …）
│   │   ├── asr_audio.py             # pyaudio 16k 录音（AudioRecorder / pcm_bytes_to_float32）
│   │   └── asr_recognizer.py        # whisper 离线识别 + OpenCC 简体（WhisperRecognizer）
│   ├── llm.py                       # OpenAI 兼容客户端（含 tools/function calling）
│   ├── mcp_client.py                # stdio 子进程客户端（Agent Host 侧）
│   └── agent.py                     # Agent 循环 run_agent() + AgentRunner
└── scripts/
    ├── mcp_sweeper_server.py        # MCP server 入口（MCP_BACKEND=mock|ros）
    ├── start_mcp_server.sh          # 供外部 MCP host 注册的启动脚本（source 环境 + exec server）
    ├── test_mcp_protocol.py         # M1 协议离线测试（7 项）
    ├── test_deepseek_tools.py       # M3 探针：验证模型支持 tools
    ├── test_agent_planning.py       # 指令拆分+顺序执行+导航监控测试（离线 FakeLLM / --live 真实模型）
    ├── test_sim_voice.py            # 仿真闭环自动测试（发指令→校验移动→停下，含顺序指令用例）
    ├── voice_agent_cli.py           # 语音 CLI：说话→识别→Agent→执行→答复
    ├── voice_agent_sim.py           # 语音联调：语音→ASR→/voice/text→车动→打印回复（不直接调 LLM）
    ├── voice_agent_node.py          # ROS 节点：/voice/text→Agent→/voice/agent_reply + 进度发布到 /voice/agent_progress
    └── keyboard_input.py            # 键盘输入发布 /voice/text（迁移自 ai_task_decomposition，测试用）
```

配套一键脚本（仓库根 `scripts/`）：`start_sim_voice.sh`（清理→启动仿真→等就绪）、`kill_sim_all.sh`（一键清理所有仿真进程）、`test_voice_module.sh`（voice 子模块离线测试）、`test_voice_action.sh`（语音+动作联调测试）、`test_agent_planning.sh`（指令拆分/顺序执行/导航监控测试：`--live` 真实模型规划、`--sim` 真实仿真顺序联调）。

## 3. 工具清单（10 个）

| 工具 | 参数 | 说明 |
|---|---|---|
| `get_robot_status` | 无 | 电量/急停/位姿/清扫开关/模式（话题缺失返回 N/A） |
| `navigate_pose` | `x,y`(必填), `yaw`, `frame_id`(默认`camera_init`) | 本地绝对坐标导航，发布 `/move_base_simple/goal` |
| `navigate_relative` | `dx`,`dy`,`dyaw` | **相对位移导航**：读当前位姿换算目标（"往前走10米"→dx=10）。位姿来源**模式自适应**：TF 查 `camera_init→base_link` 优先（GPS 与 FAST_LIO 两种模式都通），回退读 move_base 的 `TebLocalPlannerROS/odom_topic` 参数，再回退 `pose_source_topics` |
| `navigate_gps` | `latitude`,`longitude`, `altitude?` | GPS 经纬度导航，发布 `/gps/goal_fix` |
| `cancel_navigation` | 无 | 取消当前导航 |
| `navigation_status` | 无 | 查询导航状态（idle/active/succeeded/…） |
| `emergency_stop` | `active`, `reason?` | 急停（最高优先级） |
| `sweep_set` | `action: on/off/toggle` | **定点清扫**开关（TCP 清扫装置，当前唯一已实现清扫能力） |
| `sweep_coverage` | `area/pattern/duration/width`（预留） | **全覆盖清扫【预留接口，尚未实现】**，调用返回未实现提示 |
| `set_fod_mode` | `enabled` | 切换 GPS↔FOD 回收模式（需 FOD 管理器在线，后补） |

> **预留接口说明**：`sweep_coverage`（全覆盖清扫）是后续要补的能力，接口与参数 schema 已按计划定义（area/pattern/duration/width），当前 handler 返回"尚未实现"。**循迹清扫/倾倒/定点抓取等**尚未定义，扩展方式与 `sweep_coverage` 相同：在 `tools.py` 加一条 TOOL_SPEC + 在 `mock_backend`/`ros_backend` 各加一个同名方法即可，无需改协议层。
>
> 坐标系：本项目无 `map` 地图，全局帧是 **`camera_init`**（GPS ENU，仿真与真车一致），工具默认帧已对齐。

## 4. 运行

### 4.1 准备

模型与密钥**默认写在 `config/sweeper_mcp.yaml`**（改完重启即生效），无需手动 export。优先级：启动参数 > 环境变量 > 配置文件。

```bash
# 可选：临时换模型/密钥（覆盖配置文件）
export DEEPSEEK_MODEL="deepseek-chat"      # 或 CLI 启动时加 --llm-model / --api-key
# export DEEPSEEK_API_KEY="sk-xxx"

# 使用前 source 工作区（devel 已编译过）
cd /mnt/storage/robot_project/autorobot/autolabor-robot-nav
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 4.2 离线测试（不碰 ROS/网络）

```bash
cd src/sweeper_mcp
MCP_BACKEND=mock python3 scripts/test_mcp_protocol.py     # 7 项协议测试
python3 scripts/test_deepseek_tools.py                     # 验证当前模型支持 tools（key 读配置文件）
```

### 4.3 直接调用 MCP server（验证工具）

```bash
# 先起 Gazebo 仿真（或真车 bringup），再：
MCP_BACKEND=ros python3 src/sweeper_mcp/scripts/mcp_sweeper_server.py
# 另一终端用任意 MCP 客户端调用，或：
#   printf '{"jsonrpc":"2.0","id":1,"method":"tools/list"}\n' | MCP_BACKEND=ros python3 .../mcp_sweeper_server.py
```

### 4.4 语音 CLI（说话→识别→执行→答复）

```bash
MCP_BACKEND=ros python3 src/sweeper_mcp/scripts/voice_agent_cli.py
# 终端: start 开始录音 → stop 结束并识别 → 自动执行并打印答复
```

### 4.5 ROS 节点形态（常驻）

```bash
roslaunch sweeper_mcp voice_agent.launch
# 另一终端发指令 / 观察答复
rostopic pub -1 /voice/text std_msgs/String "data: '往前走10米并打开清扫'"
rostopic echo /voice/agent_reply
```

### 4.6 Gazebo 仿真启动方式（三种，按需选用）

**① 纯 Gazebo 导航仿真（无语音，滚动目标自动巡航 60m）** —— 验证底盘/GPS/move_base 基线：
```bash
roslaunch robot_bringup gazebo_sim_navigation.launch           # 默认 GUI（RViz + Gazebo）
roslaunch robot_bringup gazebo_sim_navigation.launch headless:=true   # 无窗口（服务器）
roslaunch robot_bringup gazebo_sim_navigation.launch goal_dy:=40.0    # 改滚动目标方向
roslaunch robot_bringup gazebo_sim_navigation.launch start_operator_gui:=true  # 换成生产操作台 GUI
```

**② 仿真语音闭环（标准世界 + 语音 Agent，原地待命等指令）** —— 发中文指令即驱动：
```bash
roslaunch sweeper_mcp sim_voice_demo.launch            # headless 默认；gui:=true 显示 RViz
# 另一终端发指令验证：rostopic pub -1 /voice/text std_msgs/String "data: 往前走5米"
```

**③ 一键脚本（推荐日常用）** —— 清理→启动→等就绪→测试/联调一条龙：
```bash
bash scripts/start_sim_voice.sh            # 拉起 ②，等"仿真就绪"
bash scripts/test_voice_module.sh          # 语音子模块离线检查（不启动仿真）
bash scripts/test_voice_action.sh          # 全自动：验模块→启仿真→发指令→校验车动
```

### 4.7 仿真语音闭环测试（标准世界 + 语音 + 大模型）

一键启动：标准 Gazebo 世界（与 `gazebo_sim_navigation.launch` 相同配置，仅把滚动最终目标设到原点，
机器人启动后原地待命）+ 语音 Agent。无人车**一直等待语音指令，收到就移动，到达目标停下，等下一句**。

```bash
# 终端1：启动（headless 默认；gui:=true 显示 RViz）
roslaunch sweeper_mcp sim_voice_demo.launch

# 终端2：自动化测试（发指令→校验移动→到达停下→下一条）
python3 src/sweeper_mcp/scripts/test_sim_voice.py
# 或手动输入指令：python3 .../test_sim_voice.py --interactive
# 或用一键联调脚本：bash scripts/test_voice_action.sh
```

自动化测试内置 4 个用例（前进5米 / 查询状态 / 左平移3米 / 后退3米），实测 4/4 通过。
**注意**：纯原地转向（"原地左转90度"）不作为自动化用例——本项目 TEB 配置 `yaw_goal_tolerance=6.283`（全圈），
move_base 不强制到达朝向，纯旋转目标会被当作"已到达"。转向请用带位移的指令（如"左前方走4米"）。

### 4.8 语音控制仿真联调（用户语音 → ASR → MCP → 机器人运动）

> 一键启动仿真（标准世界 + 语音 Agent）后，对麦克风说话即可驱动仿真车移动，全程无需敲键盘：

```bash
bash scripts/start_sim_voice.sh                          # 终端1：拉起仿真 + voice_agent，等"仿真就绪"
python3 src/sweeper_mcp/scripts/voice_agent_sim.py       # 终端2：语音联调
#   终端2 内: 输入 start 开始录音 → 对麦克风说"往前走5米" → 输入 stop 结束
#   自动: whisper 识别 → 发布 /voice/text → voice_agent(DeepSeek+MCP) 执行 → 打印回复
```

`voice_agent_sim.py` **不直接调 LLM/MCP**，只负责"识别你说的中文 → 发到 `/voice/text` → 打印
`/voice/agent_reply` 回复"。真正的理解与执行由常驻 `voice_agent` 节点完成，因此前后两端各管一段、
职责清晰。无麦克风时可用 `rostopic pub -1 /voice/text std_msgs/String "data: 往前走5米"` 验证链路仍通。

**进度实时回传**：`voice_agent` 节点把 agent 每行状态（`[规划]`任务分解 / `[执行]`指令N /
`[导航]`状态 / `[完成]`）发布到 `/voice/agent_progress`，CLI 逐行实时打印。发一条多步指令
（如"先往前走2米，然后再往前开3米"）时，你会先看到**任务分解结果**，然后**"执行指令1"**，
接着车才开始动，最后收到最终答复：

```
  ⏳ [规划] 指令拆分完成，共 2 条，按顺序执行:
  ⏳    指令1: navigate_relative(dx=2, dy=0, dyaw=0) ← 先相对当前位置前进2米
  ⏳    指令2: navigate_relative(dx=3, dy=0, dyaw=0) ← 然后再相对当前位置前进3米
  ⏳ [执行] ▶ 指令1/2: navigate_relative(dx=2, dy=0, dyaw=0)
  ⏳ [工具]   navigate_relative → 已发布导航目标: x=1.997 y=-0.119 ...   ← 车开始动
  ⏳ [导航]  状态 → active: ...  →  succeeded: Goal reached.
  ⏳ [执行] ✓ 指令1/2 完成
  ...
🤖 已完成您的指令：先往前开2米，再往前开3米，两步均已成功到达目标点。
```

等待最终答复的超时默认 300s（多步指令单步导航+LLM 可到 1~3 分钟，60s 不够），超过 40s
无进度更新会提示"可能卡住"但不会提前放弃（`--reply-timeout` / `--progress-topic` 可调）。

### 4.9 外部程序连调 Quick Start（外部程序 ↔ 本包）

本包是 **MCP stdio server + ROS 话题节点**双重形态，外部程序可任选一种方式接入：

**方式 A：外部 MCP host（Claude Code / Claude Desktop / 任意 MCP 客户端）→ stdio 调工具**
`mcp_sweeper_server.py` 是标准 stdio MCP server，把它的启动命令注册进 host 即可直接调用 10 个 ROS 工具：
```bash
# 注册命令（host 侧配置 server.command）：
bash -c 'source /opt/ros/noetic/setup.bash \
  && source /mnt/storage/robot_project/autorobot/autolabor-robot-nav/devel/setup.bash \
  && exec /mnt/storage/robot_project/autorobot/autolabor-robot-nav/src/sweeper_mcp/scripts/start_mcp_server.sh'
# 或手动起 server：MCP_BACKEND=ros python3 src/sweeper_mcp/scripts/mcp_sweeper_server.py
```
Claude Desktop 示例（`claude_desktop_config.json` 的 `mcpServers`）：
```json
{
  "mcpServers": {
    "sweeper_mcp": {
      "command": "bash",
      "args": ["-c", "source /opt/ros/noetic/setup.bash && source /mnt/storage/robot_project/autorobot/autolabor-robot-nav/devel/setup.bash && exec /mnt/storage/robot_project/autorobot/autolabor-robot-nav/src/sweeper_mcp/scripts/start_mcp_server.sh"]
    }
  }
}
```
先起仿真/真车（否则工具返回"ROS 不可达"）；`MCP_BACKEND=mock` 可离线验证工具 schema。

**方式 B：ROS 话题连调（语音/文本 → 常驻节点 → 回复）**
```bash
roslaunch sweeper_mcp voice_agent.launch            # 常驻 voice_agent 节点
rostopic pub -1 /voice/text std_msgs/String "data: '往前走10米并打开清扫'"
rostopic echo /voice/agent_reply
```
数据流：`/voice/text` → `voice_agent_node`(DeepSeek function calling + MCP 子进程) → `/voice/agent_reply`。

**方式 C：Python 直接连 MCP server 子进程（包内客户端）**
```python
from sweeper_mcp.mcp_client import MCPClient
c = MCPClient(server_env_backend="ros")   # 自动 spawn 子进程并注入 MCP_BACKEND=ros
c.initialize()
c.list_tools()                            # 10 个工具
c.call_tool("navigate_relative", {"dx": 10})
c.close()
```

**密钥/配置优先级**：启动参数 > 环境变量（`DEEPSEEK_API_KEY` / `DEEPSEEK_MODEL`）> 配置文件 `config/sweeper_mcp.yaml`。

## 5. 配置（config/sweeper_mcp.yaml）

- `agent`：base_url / model / temperature / timeout / max_rounds（**常规循环兜底**上限，规划阶段一次到位不再受轮数限制）等。
  - **指令拆分+顺序执行（2026-08-19 新增）**：`enable_planning=true`（大模型先把整句指令拆成步骤列表再顺序执行）、`max_plan_steps=20`（单条指令最多拆成多少步）、`nav_wait_timeout=240`（单步导航最长等待秒）、`nav_wait_poll=1.0`（导航状态轮询间隔秒）、`continue_on_nav_fail=true`（某步导航失败是否继续后续）、`llm_summary=true`（完成后大模型总结，失败自动退回本地总结）、`verbose=true`（终端打印 规划/执行/导航 全过程）。
- `ros`：话题名（goal_topic / gps_goal_topic / cancel_topic / status_topic / chassis_info_topic / emergency_stop_topic / fod_mode_service）、`goal_frame=camera_init`、`pose_source_topics`（navigate_relative 位姿来源的**最后回退列表**；首选 TF 查 `camera_init→base_link`，其次 move_base 的 `TebLocalPlannerROS/odom_topic` 参数。GPS 模式为 `/gps/odom`，FAST_LIO 模式为 `/Odometry`）。
- `sweep`：清扫装置 TCP ip/port。
- 清扫装置目录可用环境变量 `SWEEP_DEVICE_DIR` 覆盖；设备 ip/port 用 `SWEEP_IP`/`SWEEP_PORT`。

## 6. 验证状态（2026-08-16）

- ✅ M1 最小 MCP server：7 项协议测试通过（版本回显/工具列表/调用/缺参/未知工具-32602/未知方法-32601/stdout 纯净）。
- ✅ M2 真实 ROS 后端：9 工具在 roscore+假 odom 下全部验证（目标话题/消息字段正确、navigate_relative 坐标换算正确、话题缺失优雅降级、清扫无设备时容错返回）。
- ✅ M3 Agent 循环：`deepseek-v4-flash` 探针确认支持 tools；mock 端到端「查询状态」「往前走10米并打开清扫」均正确触发工具并输出答复。
- ✅ M4 节点闭环：`/voice/text`→Agent→`/voice/agent_reply` 跑通。
- ✅ **仿真语音闭环测试**（见 4.7）：标准仿真世界 + 语音 Agent 一键启动，自动化测试 4/4 通过（前进5米 / 查询状态 / 左平移3米 / 后退3米，每个用例到达目标后停下、等待下一条指令）。
- ✅ **目标投递可靠性修复**（2026-08-17）：`_publish_reliable` 由"重复发布一次"升级为"等时钟同步 + 等连接 + 重打时间戳补发 3 次"。修复前 MCP server 每个新进程的**第一个目标带 0 时间戳**被 move_base 秒判"已到达"（车不动却报 succeeded）；修复后可靠性测试三个目标全部被接受并执行。
- ✅ **语音控制仿真联调**（2026-08-17）：新增 `voice_agent_sim.py`（语音 → ASR → `/voice/text` → 车动 → 打印 `/voice/agent_reply`）；同时新增外部 MCP host 连调脚本 `start_mcp_server.sh`（见 4.8/4.9）。实际语音识别联调待有麦克风环境由用户执行，链路已用 `rostopic`/自动测试验证。
- ✅ **AI 语音模块合并**（2026-08-17）：`ai_task_decomposition` 的 ASR / 键盘模块全部迁入 `sweeper_mcp.voice` 子模块（`asr_audio.py` / `asr_recognizer.py`）与 `scripts/keyboard_input.py`；`voice_agent_cli.py` / `voice_agent_sim.py` 改为包内导入（不再 sys.path 硬编码跨包引用）；`voice_agent.launch` 的 keyboard_input 指向本包；`ai_task_decomposition` 保留空壳完全废弃。新增语音联调测试脚本 `test_voice_module.sh`（离线）/ `test_voice_action.sh`（仿真+动作）。
- ✅ **四元数 bug 修复**（2026-08-18）：`navigate_pose` 原先 `orientation.z=yaw; w=0` 生成非单位四元数，move_base 判"目标朝向非法"而 abort（如"导航到（20，-10）"）。已改为标准单位四元数 `z=sin(yaw/2), w=cos(yaw/2)`；回归验证 navigate_pose(20,-10) 与 navigate_pose(15,5,yaw=1.57) 均 SUCCEEDED，test_sim_voice.py 4/4 通过。
- ✅ **指令拆分 + 顺序执行 + 导航监控**（2026-08-19）：Agent 改为两阶段循环——大模型先把整句指令拆成步骤列表（终端打印"指令1/指令2/..."），再按顺序逐条执行；异步导航自动轮询 navigation_status，终端实时输出 `active→succeeded/aborted`，完成后自动执行下一条。实测「先回到原点，然后再往前开10米」被正确拆成 2 条并依次执行（mock 与真实 DeepSeek 均验证）；离线编排测试 4/4 通过；仿真顺序指令用例已加入 test_sim_voice.py。
- ✅ **导航监控"秒判到达" bug 修复**（2026-08-19）：`navigation_status` 原先返回 move_base **全部历史目标**的状态拼接（status_list 按旧→新排列，首段是最早目标），监控循环取首段 → 把旧目标的 `succeeded` 当成当前结果，指令1 未真正到达就执行下一条（顺序指令总位移 5m 只走 3.57m）。已改为：只报**最近目标**（status_list 末尾）的状态；监控循环加 arm 逻辑——必须先见过 `pending/active` 才接受终态，并对"轮询间隙极快完成/被拒绝"用连续 2 次相同终态兜底。修复后顺序指令 5m 实测 4.98m，test_sim_voice.py 5/5 通过，且单步/顺序指令均显示真实 `active→succeeded` 过渡。
- ✅ **进度实时回传 CLI**（2026-08-19）：`voice_agent` 节点新增 `/voice/agent_progress` 话题，把 agent 每行状态（任务分解/执行/导航/完成）实时发布；`voice_agent_sim.py` 逐行打印——先见任务分解结果→"执行指令1"→车才开始动→最终答复。同时修复 CLI 双重发布（同一条指令被 agent 并发执行两遍、耗时翻倍）与 60s 过早超时（改 300s 硬上限 + 40s 无进度提示）。实测多步指令总耗时 14.7s 收到完整答复，进度流 18 行顺序正确；离线协议 7/7、离线编排 4/4、test_sim_voice.py 5/5 均通过。
- ⏳ 真车联调（待安排）。

## 7. 安全与注意事项

- **密钥**：默认 key/model 写在 `config/sweeper_mcp.yaml`（用户要求，方便随时改）。优先级：启动参数 `--api-key`/`--llm-model` > 环境变量 `DEEPSEEK_API_KEY`/`DEEPSEEK_MODEL` > 配置文件。旧 `config/task_decomp.yaml` 的明文 key 已删除，改用本文件或环境变量。
- **导航安全**：工具**禁止裸写 /cmd_vel**，一律走受控链（`/gps/goal_fix` 或 `/move_base_simple/goal`，由 gps_goal_speed_limiter + fod_navigation_mode 仲裁器把关）。
- **SweepDeviceControl 的坑**：它内部 `print()` 打日志，在 MCP server 进程内调用会污染 stdio 协议流，`sweep_backend` 已用 `redirect_stdout` 重定向到 stderr。
- **发布可靠性**：`_publish_reliable` 三步保证送达——等 `/clock` 同步（use_sim_time 下新进程 `Time.now()` 从 0 开始，带 0 时间戳的目标会被 move_base 误判"已到达"）→ 等 publisher 连接建立 → 补发 3 次并重打最新时间戳。**改它时务必保留这三步**（详见 `ros机器人开发文档/清扫车MCP语音控制/move_base导航排错经验.md`）。
- **旧链路**：`ai_task_decomposition` 包已整体废弃（2026-08-17）。原 ASR / 键盘模块已迁移进本包 `voice` 子模块与 `scripts/keyboard_input.py`，旧包保留空壳不再提供代码，避免与新链路重复驱动导航。

## 8. 已知环境问题（与本包无关）

`catkin_make` 全量重构时 `navigation_arena/arena-rosnav-3D/.../arena_traj_planner` 报 `find_package(nlopt)` 失败（nlopt 已装但 cmake config 路径没找到）。**现有 devel 是完整编译过的，可正常运行**；但新增包后想把它编进 devel 需要先解决该问题（例如给 nlopt 设置 `nlopt_DIR` 或修复 arena 的 find_package）。
