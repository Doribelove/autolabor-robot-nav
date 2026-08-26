# -*- coding: utf-8 -*-
"""Agent 循环 —— 语音/文本指令 → 指令拆分 → DeepSeek(规划) → MCP 工具顺序执行 → 最终答复。

与语音/ROS 解耦：run_agent(text, llm, mcp) 只依赖两个接口对象：
  - llm.chat(messages, tools) -> {"content": str|None, "tool_calls": [...]|None}
  - mcp.list_tools() / mcp.call_tool(name, arguments) -> {"text": str, "is_error": bool}

两阶段循环（解决"多条指令无法先后叠加执行" + "无终端交互/不主动监控"两个问题）：

**阶段一 规划（plan）**：让大模型把整句自然语言拆成"按顺序执行的工具调用列表"
（强制只调用 submit_plan 提交计划），终端打印拆分结果，例如：
    指令1: navigate_pose(x=0, y=0, yaw=0)   ← 回到原点，车头对正X轴
    指令2: navigate_relative(dx=10)          ← 再往前开10米

**阶段二 执行（execute）**：按顺序逐条调用工具；对异步导航类工具自动轮询
navigation_status 并实时在终端输出状态变化（active→succeeded/aborted），
某一步完成后自动执行下一步，全部完成后输出总结。

若规划阶段失败（大模型不配合），自动退回旧的单轮 function calling 循环
run_agent_legacy，保证最坏情况下仍能工作。
"""

import json
import logging
import os
import threading
import time

logger = logging.getLogger(__name__)

DEFAULT_SYSTEM_PROMPT = (
    "你是智慧环卫无人清扫车的语音控制大脑。用户的自然语言指令将翻译成机器人工具调用并执行。\n\n"
    "【当前已实现能力】\n"
    "- 本地坐标导航(navigate_pose)：仅当你确切知道本地坐标 x/y 时使用。\n"
    "- 相对位移导航(navigate_relative)：如'往前走10米/后退/左移/左转90度'，dx/dy/dyaw。\n"
    "- GPS 经纬度导航(navigate_gps)：用户给了明确经纬度时使用。\n"
    "- 取消导航(cancel_navigation)、查询导航状态(navigation_status)。\n"
    "- 查询机器人状态(get_robot_status)：电量/位姿/清扫开关/模式。\n"
    "- 急停(emergency_stop)：检测到危险时优先调用。\n"
    "- 清扫装置开关(sweep_set)：on/off/toggle（定点清扫）。\n\n"
    "【尚未实现，请如实告知，不要编造】\n"
    "- 全覆盖清扫/循迹清扫/倾倒/定点抓取等能力尚未实现；用户要求时说明'该功能还在开发中'，\n"
    "  并给出可行的替代（如用 navigate + sweep_set 执行定点清扫）。\n\n"
    "【工具使用规范】\n"
    "- 一条指令含多个连续动作（如'先…再…/然后/最后'）时，Agent 会自动拆分成多步依次执行；\n"
    "  每步导航都会等到达后才执行下一步，无需你手动查询状态。\n"
    "- 用户说'回到原点/回起点'：原点即本地坐标系 (0,0)，用 navigate_pose(x=0, y=0, yaw=0)。\n"
    "- 用户没给坐标/经纬度时，先向用户询问，不要编造数字。\n"
    "- 所有导航目标都经过 move_base（带局部 costmap + TEB 避障），禁止绕过导航直接发速度。\n"
    "- 若导航状态返回 aborted（常见原因：目标点在障碍物内/不可达、被取消），如实告诉用户，\n"
    "  并建议一个可达的目标点或换一种指令，不要强行重试同一目标。\n"
    "- 工具调用失败时，把失败原因如实告诉用户，并提出解决办法。\n"
    "- 全部执行完后，用一两句中文总结完成情况；如无需工具则直接回答。\n"
)

# ---------------- 规划阶段 ----------------

# 规划专用工具：大模型只允许调用它提交计划，不允许直接调用业务工具。
# steps 里每一项都是一个"已定义工具"的调用（tool=工具名，arguments=参数，description=中文说明）。
PLAN_TOOL = {
    "type": "function",
    "function": {
        "name": "submit_plan",
        "description": (
            "把用户指令拆分成按顺序执行的工具调用计划。你只能调用本工具一次，"
            "把完整步骤列表放在 arguments.steps 中；不要直接执行任何业务工具，只输出计划。"
            "若指令是纯查询/闲聊、不需要调用工具，把 steps 置为空数组，并在 answer 里给出直接答复。"
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "steps": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "tool": {"type": "string", "description": "要调用的工具名（必须是提供的工具之一）"},
                            "arguments": {"type": "object", "description": "该工具的参数（字段与工具 schema 一致）"},
                            "description": {"type": "string", "description": "用中文简要描述这一步做什么"},
                        },
                        "required": ["tool", "arguments", "description"],
                    },
                    "description": "按执行先后顺序排列的步骤，至少 1 条",
                },
                "answer": {
                    "type": "string",
                    "description": "可选。仅当 steps 为空（无需工具）时，填写给用户的直接中文答复。",
                },
            },
            "required": ["steps"],
        },
    },
}

# 异步导航类工具：发布即返回，需要轮询 navigation_status 等待终态。
ASYNC_NAV_TOOLS = {"navigate_pose", "navigate_relative", "navigate_gps"}

# 导航终态（move_base GoalStatus，见 ros_backend.GOAL_STATUS_NAMES）
NAV_TERMINAL_STATES = {"succeeded", "aborted", "preempted", "canceled", "rejected", "recalled", "lost"}


# 进度监听者（如 ROS 节点把每行状态发布到 /voice/agent_progress，供 CLI 实时显示）
_PROGRESS_LISTENERS = []
_PROGRESS_LOCK = threading.Lock()


def set_progress_listener(cb):
    """注册进度回调：agent 每输出一行状态（规划拆分/执行/导航监控/完成）都会调用 cb(line)。

    传 None 清空所有监听（幂等）。回调抛异常会被隔离，不影响 agent 主流程。
    """
    global _PROGRESS_LISTENERS
    with _PROGRESS_LOCK:
        if cb is None:
            _PROGRESS_LISTENERS = []
        else:
            _PROGRESS_LISTENERS.append(cb)


def _emit_progress(line):
    """把一行进度转发给所有监听者（不阻塞、异常隔离）。"""
    with _PROGRESS_LOCK:
        cbs = list(_PROGRESS_LISTENERS)
    for cb in cbs:
        try:
            cb(line)
        except Exception:
            pass


def _say(msg, verbose=True):
    """终端输出（带 flush，保证实时呈现），同时记日志、转发给进度监听者。
    verbose=False 时静默。"""
    if not verbose:
        return
    print(msg, flush=True)
    logger.info("%s", msg)
    _emit_progress(msg)


def _build_plan_prompt(tools):
    """规划阶段 system 提示：告诉大模型拆分规则，并把真实工具 schema 内嵌成文本（供规划填参数）。"""
    lines = [
        "你的任务是：把用户指令拆分成一条或多条【按顺序执行】的工具调用，并调用 submit_plan 提交。",
        "规则：",
        "1. 每个动作只对应一个工具调用；出现'先…再…/然后/最后/之后'等词时，务必按执行先后顺序列出，不要合并。",
        "2. 若一条指令本身包含多个连续动作（例如'先回到原点然后再往前开10米'），必须拆成多条步骤。",
        "3. 只规划，不直接执行业务工具；只调用 submit_plan 一次提交完整计划。",
        "4. 纯查询/闲聊（如'你好'、'车在哪'、'还有多少电'）不需要工具时，steps 置空数组，在 answer 给出直接答复。",
        "5. '回到原点/回起点'即本地坐标 (0,0)：navigate_pose(x=0, y=0, yaw=0)。'车头对正X轴'即 yaw=0。",
        "6. 参数必须从用户话里推断或询问，禁止编造。",
        "",
        "可用工具及其参数（供你规划时填 arguments，字段可选性见 schema）：",
    ]
    for t in tools:
        fn = t.get("function", {})
        name = fn.get("name") or t.get("name", "")
        desc = fn.get("description") or ""
        params = fn.get("parameters") or {}
        lines.append("- %s: %s" % (name, desc[:160]))
        lines.append("    参数 schema: %s" % json.dumps(params, ensure_ascii=False)[:300])
    return "\n".join(lines)


def _parse_plan_args(raw):
    """解析 submit_plan 的 arguments JSON，返回 (steps, answer)；格式非法返回 (None, None)。"""
    try:
        args = json.loads(raw or "{}")
    except ValueError:
        return None, None
    if not isinstance(args, dict):
        return None, None
    steps = []
    steps_raw = args.get("steps")
    if steps_raw:
        if not isinstance(steps_raw, list):
            return None, None
        for item in steps_raw:
            if not isinstance(item, dict):
                continue
            tool = item.get("tool")
            if not tool or not isinstance(tool, str):
                continue
            arguments = item.get("arguments") or {}
            if not isinstance(arguments, dict):
                arguments = {}
            steps.append({
                "tool": tool,
                "arguments": arguments,
                "description": item.get("description") or tool,
            })
    answer = args.get("answer")
    if isinstance(answer, str):
        answer = answer.strip() or None
    return steps, answer


def _plan(text, llm, tools, max_steps):
    """规划阶段：大模型只允许调用 submit_plan。

    返回 (steps, answer)：
      - steps: list（可能为空，表示无需工具）；为空且 answer 有值时直接答复。
      - steps is None：模型未返回有效计划（罕见），调用方应退回常规循环。
    """
    prompt = _build_plan_prompt(tools)
    messages = [{"role": "system", "content": prompt}, {"role": "user", "content": text}]
    # 注意：DeepSeek 思考模式不支持强制 tool_choice；且这里只暴露 submit_plan 一个函数，
    # 所以用默认 auto 即可 —— 需要工具时模型只能调 submit_plan，纯查询则直接文字回答。
    msg = llm.chat(messages, tools=[PLAN_TOOL])
    calls = msg.get("tool_calls") or []
    for tc in calls:
        if tc.get("function", {}).get("name") == "submit_plan":
            steps, answer = _parse_plan_args(tc["function"].get("arguments"))
            if steps is not None:
                if len(steps) > max_steps:
                    steps = steps[:max_steps]
                return steps, answer
    # 未调用 submit_plan：视为"无需工具"，采纳模型直接文字答复（如纯查询的回应）。
    content = (msg.get("content") or "").strip()
    return [], content or None


def _direct_answer(text, llm, system_prompt):
    """纯查询场景：让模型直接回答（无工具）。失败返回空串。"""
    messages = [{"role": "system", "content": system_prompt}, {"role": "user", "content": text}]
    try:
        msg = llm.chat(messages)
        return (msg.get("content") or "").strip()
    except Exception:
        return ""


# ---------------- 执行阶段 ----------------

def _fmt_args(args):
    """把参数 dict 转成紧凑文本：x=0.0, y=0.0, yaw=0.0。"""
    if not args:
        return ""
    parts = []
    for k, v in args.items():
        if isinstance(v, (dict, list)):
            v = json.dumps(v, ensure_ascii=False)
        elif isinstance(v, float):
            v = ("%g" % v)
        parts.append("%s=%s" % (k, v))
    return ", ".join(parts)


def _monitor_navigation(mcp, timeout=240.0, poll=1.0, verbose=True):
    """执行异步导航工具后，轮询 navigation_status 直到终态，终端实时输出状态变化。

    返回 (state, text)：
      - state: succeeded/aborted/.../timeout
      - text: 最后一次状态原文（含 move_base 的报错文本，如 "aborted: Goal position is in an obstacle"）

    正确性要点：move_base 的 /move_base/status 记录全部历史目标，navigation_status
    只报"最近目标"的状态。这里用 arm 逻辑避免两种误判：
      - 旧目标的终态（succeeded/preempted）被当成当前结果 → 必须先见过
        pending/active 才接受终态；
      - 目标在轮询间隙极快完成/被拒绝（从未出现 active）→ 同一终态连续 2 次兜底。
    """
    if verbose:
        _say("[导航]  开始监控目标执行状态（每 %g 秒轮询，最长 %g 秒）..." % (poll, timeout))
    t0 = time.time()
    last = None
    stable = 0
    armed = False
    while time.time() - t0 < timeout:
        try:
            res = mcp.call_tool("navigation_status", {}, timeout=10.0)
            text = (res.get("text") or "idle").strip()
        except Exception as exc:
            text = "状态查询失败: %s" % exc
        # "active: 文本" / "idle（当前无导航目标...）" 等 → 取首段为状态
        state = text.split(":")[0].split("（")[0].strip().lower()
        stable = stable + 1 if state == last else 1
        if state != last:
            last = state
            if verbose:
                _say("[导航]  状态 → %s" % text)
        if state in ("pending", "active"):
            armed = True          # 已见到自己目标进入执行，后续终态即为结果
        elif state in NAV_TERMINAL_STATES:
            if armed or stable >= 2:
                return state, text
        time.sleep(poll)
    return "timeout", "等待导航完成超时(%.0fs)" % timeout


def _local_summary(steps, results):
    """本地总结（无 LLM 依赖，最坏情况兜底）。"""
    if not results:
        return "（没有可执行的指令）"
    parts = []
    for i, (s, r) in enumerate(zip(steps, results), 1):
        mark = "✓" if r["ok"] else "✗"
        parts.append("%d.%s %s" % (i, mark, s.get("description") or s["tool"]))
    return "已依次完成 %d 条指令：%s" % (len(results), "；".join(parts))


def _llm_summary(text, steps, results, llm, system_prompt, tool_text_limit):
    """用大模型生成一到两句总结（失败返回空串，调用方退回本地总结）。"""
    lines = []
    for i, (s, r) in enumerate(zip(steps, results), 1):
        lines.append("%d) %s(%s) → %s [%s]" % (
            i, s["tool"], _fmt_args(s.get("arguments")),
            (r.get("detail") or "")[:tool_text_limit],
            "成功" if r["ok"] else "失败"))
    record = "\n".join(lines)
    messages = [
        {"role": "system",
         "content": system_prompt + "\n\n【任务】根据下面的执行记录，用一到两句中文向用户总结完成情况。失败项要如实指出并给出建议。"},
        {"role": "user", "content": "原指令：%s\n\n执行记录：\n%s" % (text, record)},
    ]
    try:
        msg = llm.chat(messages)
        return (msg.get("content") or "").strip()
    except Exception:
        return ""


def _run_planned(text, llm, mcp, tools, system_prompt, tool_text_limit,
                 max_plan_steps, nav_wait_timeout, nav_wait_poll,
                 continue_on_nav_fail, llm_summary, verbose):
    """两阶段执行：规划 → 顺序执行（含导航监控）。规划失败返回 None（退回常规循环）。"""
    if verbose:
        _say("[规划] 收到指令: %s" % text)
    steps, answer = _plan(text, llm, tools, max_plan_steps)
    if steps is None:
        if verbose:
            _say("[规划] 模型未返回有效计划，退回常规循环。")
        return None
    if not steps:
        if not answer:
            answer = _direct_answer(text, llm, system_prompt)
        if verbose:
            _say("[规划] 无需调用工具，直接答复: %s" % (answer or "(无答复)"))
        return (answer or "(无答复)").strip()

    valid = {t.get("function", {}).get("name") or t.get("name") for t in tools}
    if verbose:
        _say("[规划] 指令拆分完成，共 %d 条，按顺序执行:" % len(steps))
        for i, s in enumerate(steps, 1):
            _say("   指令%d: %s(%s) ← %s" % (i, s["tool"], _fmt_args(s.get("arguments")),
                                             s.get("description") or s["tool"]))

    results = []
    total = len(steps)
    for i, s in enumerate(steps, 1):
        name = s.get("tool")
        args = s.get("arguments") or {}
        if name not in valid:
            _say("[执行] ⚠ 指令%d: 未知工具 %r，跳过。" % (i, name), verbose)
            results.append({"tool": name, "ok": False, "detail": "未知工具 %s" % name})
            continue
        if verbose:
            _say("[执行] ▶ 指令%d/%d: %s(%s)" % (i, total, name, _fmt_args(args)))
        try:
            res = mcp.call_tool(name, args)
        except Exception as exc:
            res = {"text": "工具调用失败: %s" % exc, "is_error": True}
        text_ = (res.get("text") or "")[:tool_text_limit]
        step_ok = not res.get("is_error")
        if verbose:
            _say("[工具]   %s → %s" % (name, text_))
        detail = text_

        if name in ASYNC_NAV_TOOLS:
            state, monitor_text = _monitor_navigation(mcp, nav_wait_timeout, nav_wait_poll, verbose)
            detail = monitor_text
            if state != "succeeded":
                step_ok = False
                if verbose:
                    tail = "继续下一条。" if continue_on_nav_fail else "终止后续指令。"
                    _say("[执行]   ⚠ 指令%d 导航未成功（%s），%s" % (i, state, tail))
                if not continue_on_nav_fail:
                    results.append({"tool": name, "ok": False, "detail": detail})
                    break

        results.append({"tool": name, "ok": step_ok, "detail": detail})
        if verbose:
            _say("[执行] ✓ 指令%d/%d 完成" % (i, total))

    summary = ""
    if llm_summary:
        summary = _llm_summary(text, steps, results, llm, system_prompt, tool_text_limit)
    if not summary:
        summary = _local_summary(steps, results)
    if verbose:
        _say("[完成] %s" % summary)
    return summary


# ---------------- 常规（旧）循环 ----------------

def _trim(messages, window_size):
    """上下文控制：保留 system + 首条 user，其余保留最近 window_size 条。"""
    if len(messages) <= window_size + 2:
        return messages
    return messages[:2] + messages[-window_size:]


def run_agent_legacy(text, llm, mcp, tools, system_prompt=DEFAULT_SYSTEM_PROMPT,
                     max_rounds=5, max_tools_per_round=4, tool_text_limit=1500,
                     window_size=16):
    """旧的单轮 function calling 循环（规划失败时的兜底）。"""
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": text},
    ]

    for _ in range(max_rounds):
        msg = llm.chat(messages, tools=tools)
        calls = msg.get("tool_calls") or []
        if not calls:
            return (msg.get("content") or "").strip() or "(模型未给出答复)"

        messages.append({
            "role": "assistant",
            "content": msg.get("content") or "",
            "tool_calls": calls,
        })

        for tc in calls[:max_tools_per_round]:
            name = tc["function"]["name"]
            try:
                arguments = json.loads(tc["function"].get("arguments") or "{}")
                if not isinstance(arguments, dict):
                    arguments = {}
            except ValueError:
                arguments = {}
            logger.info("工具调用: %s(%s)", name, json.dumps(arguments, ensure_ascii=False))
            try:
                res = mcp.call_tool(name, arguments)
                text_ = (res.get("text") or "")[:tool_text_limit]
            except Exception as exc:
                text_ = "工具调用失败: %s" % exc
            messages.append({
                "role": "tool",
                "tool_call_id": tc.get("id"),
                "content": text_,
            })

        messages = _trim(messages, window_size)

    return "(已达到最大工具轮数，请重试或细化指令)"


def run_agent(text, llm, mcp, system_prompt=DEFAULT_SYSTEM_PROMPT,
              max_rounds=5, max_tools_per_round=4, tool_text_limit=1500,
              window_size=16,
              enable_planning=True, max_plan_steps=20,
              nav_wait_timeout=240.0, nav_wait_poll=1.0,
              continue_on_nav_fail=True, llm_summary=True, verbose=True):
    """执行一轮 Agent 循环，返回最终答复文本。

    默认走"规划+顺序执行"两阶段（enable_planning=True）：
      1. 大模型把整句指令拆成步骤列表，终端打印拆分结果；
      2. 按顺序逐条执行；异步导航自动监控到终态（成功/失败都在终端输出）后，
         才自动执行下一条；
      3. 全部完成后生成总结。

    规划失败时自动退回 run_agent_legacy（旧循环）。

    Args:
        text: 用户语音/文本指令。
        llm: OpenAIClient（或接口等价对象）。
        mcp: MCPClient（或接口等价对象）。
        max_plan_steps: 单条指令最多拆成多少步（旧 max_rounds*max_tools_per_round 的概念，
            现在规划一次到位，不再受轮数限制，支持更长的先后指令链）。
        nav_wait_timeout: 单步导航最长等待秒。
        nav_wait_poll: 导航状态轮询间隔秒。
        continue_on_nav_fail: 某步导航失败时是否继续后续步骤。
        llm_summary: 完成后是否用大模型总结（失败自动退回本地总结）。
        verbose: 是否在终端打印 规划/执行/导航 全过程（任务2：双向交互可见）。
    """
    tools = to_openai_tools(mcp.list_tools()["tools"])
    if enable_planning:
        result = _run_planned(text, llm, mcp, tools, system_prompt, tool_text_limit,
                              max_plan_steps, nav_wait_timeout, nav_wait_poll,
                              continue_on_nav_fail, llm_summary, verbose)
        if result is not None:
            return result
    return run_agent_legacy(text, llm, mcp, tools, system_prompt,
                            max_rounds, max_tools_per_round, tool_text_limit, window_size)


def to_openai_tools(schemas):
    """把 MCP tools/list 的 schema 列表转成 OpenAI function calling 的 tools 参数。"""
    return [{
        "type": "function",
        "function": {
            "name": t["name"],
            "description": t["description"],
            "parameters": t["inputSchema"],
        },
    } for t in schemas]


def load_agent_config():
    """读 config/sweeper_mcp.yaml 的 agent 段作为 LLM/Agent 默认参数；yaml 缺失时返回空。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "..", "..", "config", "sweeper_mcp.yaml"),      # 源码树
        os.path.join(here, "..", "..", "..", "share", "sweeper_mcp", "config", "sweeper_mcp.yaml"),  # devel 安装
    ]
    for path in candidates:
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                agent = (yaml.safe_load(f) or {}).get("agent") or {}
                if agent:
                    return dict(agent)
        except Exception:
            continue
    return {}


class AgentRunner:
    """懒加载的 Agent 宿主：首次 run 才连接 MCP server 并构造 LLM 客户端。

    LLM 参数解析优先级：显式传入 > 环境变量(DEEPSEEK_API_KEY/DEEPSEEK_MODEL) > 配置文件
    config/sweeper_mcp.yaml 的 agent 段。因此直接 AgentRunner(backend='mock') 也能用配置默认值。
    """

    # 可从配置文件读取的 Agent 运行参数（enable_planning / 导航监控等）
    AGENT_KWARG_KEYS = ("enable_planning", "max_plan_steps", "nav_wait_timeout",
                        "nav_wait_poll", "continue_on_nav_fail", "llm_summary", "verbose")

    def __init__(self, base_url=None, model=None, api_key=None, temperature=None,
                 timeout_s=None, backend="mock", system_prompt=None, **agent_kwargs):
        cfg = load_agent_config()
        self._llm_kwargs = {
            "base_url": base_url or cfg.get("base_url", "https://api.deepseek.com"),
            "model": model or os.environ.get("DEEPSEEK_MODEL") or cfg.get("model", "deepseek-v4-flash"),
            "api_key": api_key or os.environ.get("DEEPSEEK_API_KEY") or cfg.get("api_key", ""),
            "temperature": temperature if temperature is not None else cfg.get("temperature", 0.2),
            "timeout_s": timeout_s if timeout_s is not None else cfg.get("timeout_s", 20.0),
        }
        self._backend = backend
        self._system_prompt = system_prompt or DEFAULT_SYSTEM_PROMPT
        # 配置文件里的 Agent 运行参数 + 显式传入的合并（显式优先）
        merged = {}
        for k in self.AGENT_KWARG_KEYS:
            if k in cfg:
                merged[k] = cfg[k]
        merged.update(agent_kwargs)
        self._agent_kwargs = merged
        self._llm = None
        self._mcp = None

    def _ensure(self):
        if self._mcp is None:
            from sweeper_mcp.llm import OpenAIClient
            from sweeper_mcp.mcp_client import MCPClient
            self._llm = OpenAIClient(**self._llm_kwargs)
            self._mcp = MCPClient(server_env_backend=self._backend)
            self._mcp.initialize()

    def run(self, text):
        self._ensure()
        return run_agent(text, self._llm, self._mcp,
                         system_prompt=self._system_prompt, **self._agent_kwargs)

    def close(self):
        if self._mcp is not None:
            self._mcp.close()
            self._mcp = None
