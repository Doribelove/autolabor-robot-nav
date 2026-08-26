#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指令拆分+顺序执行+导航监控 测试（任务1/2）。

验证 Agent 两阶段循环：
  1. 大模型把整句指令拆成步骤列表（终端打印拆分结果：指令1/指令2/...）；
  2. 按顺序逐条执行；异步导航自动监控 navigation_status 到终态
     （终端输出 active→succeeded/aborted）后，才自动执行下一条；
  3. 全部完成后生成总结。

两种模式：
  默认（offline）：用 FakeLLM 返回预置计划，不依赖网络/API Key，验证编排逻辑。
  --live：用真实 DeepSeek + mock 后端，跑 "先回到原点然后再往前开10米"，
          打印完整的 规划/执行/导航监控 终端输出（需网络 + API Key）。

用法:
  python3 scripts/test_agent_planning.py          # 离线编排测试
  python3 scripts/test_agent_planning.py --live   # 真实大模型规划测试
"""

import contextlib
import io
import json
import os
import sys

_PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.agent import AgentRunner  # noqa: E402


# ---------------- FakeLLM：离线模式用，返回预置计划 ----------------

class FakeLLM:
    """按用户文本关键词返回预置计划，模拟大模型规划 + 总结。"""

    def __init__(self):
        self.calls = []  # 记录每次 chat 的 user 内容，便于断言

    def chat(self, messages, tools=None, tool_choice=None):
        user = next((m["content"] for m in reversed(messages)
                     if m["role"] == "user"), "")
        self.calls.append(user)
        is_plan = tools and any(
            t.get("function", {}).get("name") == "submit_plan" for t in tools)
        if is_plan:
            if "先" in user and "再" in user:
                steps = [
                    {"tool": "navigate_pose", "arguments": {"x": 0, "y": 0, "yaw": 0},
                     "description": "回到原点，车头对正X轴"},
                    {"tool": "navigate_relative", "arguments": {"dx": 10},
                     "description": "再往前开10米"},
                ]
            elif "你好" in user:
                return {"content": "你好！我是清扫车语音助手，有什么可以帮您？"}
            elif "多少电" in user:
                steps = [{"tool": "get_robot_status", "arguments": {},
                          "description": "查询电量"}]
            else:
                steps = [{"tool": "navigate_relative", "arguments": {"dx": 5},
                          "description": "往前走5米"}]
            return {"tool_calls": [{
                "id": "plan_1", "type": "function",
                "function": {"name": "submit_plan",
                             "arguments": json.dumps({"steps": steps}, ensure_ascii=False)},
            }]}
        # 总结调用（无 tools）
        return {"content": "测试总结：全部指令已按顺序执行完毕。"}


def _run_and_capture(text, **agent_kwargs):
    """用 mock 后端 + FakeLLM 跑一轮 agent，返回 (答复, 终端输出文本)。"""
    import sweeper_mcp.agent as agent_mod
    runner = AgentRunner(backend="mock", **agent_kwargs)
    # 用 FakeLLM 替换真实 LLM（不碰网络）
    runner._ensure()
    fake = FakeLLM()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        reply = agent_mod.run_agent(text, fake, runner._mcp, **agent_kwargs)
    runner.close()
    return reply, buf.getvalue()


def test_multi_command():
    print("测试1: 多指令拆分+顺序执行（'先...然后再...'）")
    reply, out = _run_and_capture(
        "先回到原点，车头对正X轴，然后再往前开10米",
        enable_planning=True, llm_summary=True, nav_wait_timeout=30, nav_wait_poll=0.3)
    ok = True
    for needle in ["指令拆分完成，共 2 条", "指令1:", "指令2:",
                   "[导航]  状态 → active", "[导航]  状态 → succeeded",
                   "✓ 指令1/2 完成", "✓ 指令2/2 完成"]:
        if needle not in out:
            print("  ✗ 终端缺少: %s" % needle)
            ok = False
    if "navigate_pose" not in out or "navigate_relative" not in out:
        print("  ✗ 未按顺序打印两个工具")
        ok = False
    if "测试总结" not in reply:
        print("  ✗ 总结异常: %s" % reply)
        ok = False
    print("  ✓ 终端输出片段:")
    for line in out.splitlines():
        if any(line.startswith(p) for p in ("[规划]", "[执行]", "[导航]", "[工具]", "[完成]")):
            print("     " + line)
    return ok


def test_single_command():
    print("\n测试2: 单条指令（拆成 1 步）")
    reply, out = _run_and_capture("往前走5米", enable_planning=True,
                                  llm_summary=False, nav_wait_timeout=30, nav_wait_poll=0.3)
    ok = "指令拆分完成，共 1 条" in out and "已依次完成 1 条指令" in reply
    print("  %s 答复: %s" % ("✓" if ok else "✗", reply))
    return ok


def test_pure_query():
    print("\n测试3: 纯查询（无需工具，直接答复）")
    reply, out = _run_and_capture("你好", enable_planning=True)
    ok = "无需调用工具" in out and "你好" in reply
    print("  %s 答复: %s" % ("✓" if ok else "✗", reply))
    return ok


def test_query_tool():
    print("\n测试4: 查询类工具（get_robot_status）")
    reply, out = _run_and_capture("车还有多少电", enable_planning=True,
                                  llm_summary=False, nav_wait_timeout=30, nav_wait_poll=0.3)
    ok = "指令拆分完成，共 1 条" in out and "get_robot_status" in out
    print("  %s 答复: %s" % ("✓" if ok else "✗", reply))
    return ok


def main():
    if "--live" in sys.argv:
        return run_live()
    passed = 0
    total = 4
    passed += test_multi_command()
    passed += test_single_command()
    passed += test_pure_query()
    passed += test_query_tool()
    print("\n====================")
    print("离线编排测试: %d/%d 通过" % (passed, total))
    return 0 if passed == total else 1


def run_live():
    """真实 DeepSeek 规划 + mock 后端执行：打印完整 规划/执行/导航监控 终端输出。"""
    print("== 真实大模型规划测试（mock 后端）==")
    print("指令: 先回到原点，然后再往前开10米\n")
    runner = AgentRunner(backend="mock", enable_planning=True,
                         llm_summary=True, nav_wait_timeout=60, nav_wait_poll=0.5)
    reply = runner.run("先回到原点，然后再往前开10米")
    print("\n[回复] %s" % reply)
    runner.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
