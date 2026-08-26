#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M1 协议离线测试 —— mock 后端，不碰 ROS/网络。

验证：initialize 版本回显、tools/list 返回全部工具、tools/call 正常与缺失参数、
未知工具 -32602、未知方法 -32601、stdout 每行都是合法 JSON（无日志污染）。

用法:
  cd <sweeper_mcp 包根>
  MCP_BACKEND=mock python3 scripts/test_mcp_protocol.py
"""

import json
import os
import subprocess
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.mcp_client import MCPClient, MCPClientError

SERVER = os.path.abspath(os.path.join(_THIS, "mcp_sweeper_server.py"))
ENV = dict(os.environ, MCP_BACKEND="mock")

REQUIRED_TOOLS = {
    "get_robot_status", "navigate_pose", "navigate_relative", "navigate_gps",
    "cancel_navigation", "navigation_status", "emergency_stop", "sweep_set",
    "sweep_coverage", "set_fod_mode",
}


def _client():
    return MCPClient(server_cmd=[sys.executable, SERVER], env=ENV)


def test_initialize_echo():
    c = _client()
    try:
        r = c.initialize("2024-11-05")
        assert r["protocolVersion"] == "2024-11-05", "应原样回显 2024-11-05，实际 %s" % r.get("protocolVersion")
        assert r["capabilities"].get("tools") is not None, "capabilities 应声明 tools"
    finally:
        c.close()
    print("✓ initialize 回显版本 2024-11-05，capabilities.tools 已声明")


def test_tools_list():
    c = _client()
    try:
        c.initialize()
        r = c.list_tools()
        tools = r["tools"]
        names = {t["name"] for t in tools}
        assert REQUIRED_TOOLS.issubset(names), "缺少工具: %s" % (REQUIRED_TOOLS - names)
        for t in tools:
            for key in ("name", "description", "inputSchema"):
                assert key in t, "工具 %s 缺字段 %s" % (t.get("name"), key)
        sweep_coverage = next(t for t in tools if t["name"] == "sweep_coverage")
        assert "pattern" in sweep_coverage["inputSchema"].get("properties", {}), "全覆盖工具应预留 pattern 参数"
    finally:
        c.close()
    print("✓ tools/list 返回全部 %d 个工具，含预留接口 sweep_coverage" % len(tools))


def test_tools_call_ok():
    c = _client()
    try:
        c.initialize()
        res = c.call_tool("get_robot_status")
        assert not res["is_error"], res
        assert "battery" in res["text"], "get_robot_status 应含电量字段"
        res = c.call_tool("navigate_relative", {"dx": 10, "dy": 0})
        assert not res["is_error"], res
        assert "目标" in res["text"]
        res = c.call_tool("sweep_coverage")
        assert res["is_error"], "全覆盖清扫应为 isError(尚未实现)"
        assert "未实现" in res["text"]
    finally:
        c.close()
    print("✓ tools/call get_robot_status / navigate_relative / sweep_coverage(预留) 行为正确")


def test_missing_required_param():
    c = _client()
    try:
        c.initialize()
        res = c.call_tool("navigate_pose", {"x": 1.0})
        assert res["is_error"] and "缺少必填参数" in res["text"], res
    finally:
        c.close()
    print("✓ 缺必填参数 → isError 提示")


def test_unknown_tool_error():
    c = _client()
    try:
        c.initialize()
        try:
            c.call_tool("no_such_tool")
            assert False, "应抛出 MCPClientError"
        except MCPClientError as exc:
            assert exc.code == -32602, exc.code
    finally:
        c.close()
    print("✓ 未知工具返回 -32602")


def test_unknown_method_error():
    c = _client()
    try:
        c.initialize()
        try:
            c.request("no_such_method")
            assert False, "应抛出 MCPClientError"
        except MCPClientError as exc:
            assert exc.code == -32601, exc.code
    finally:
        c.close()
    print("✓ 未知方法返回 -32601")


def test_stdout_purity():
    """server 的 stdout 每一行都必须是合法 JSON，且响应条数精确。"""
    p = subprocess.Popen([sys.executable, SERVER], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=ENV)
    msgs = [
        {"jsonrpc": "2.0", "id": 1, "method": "initialize",
         "params": {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "t", "version": "1"}}},
        {"jsonrpc": "2.0", "method": "notifications/initialized"},   # 通知，不应有响应
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
        {"jsonrpc": "2.0", "id": 3, "method": "tools/call",
         "params": {"name": "navigate_pose", "arguments": {"x": 1, "y": 2}}},
    ]
    payload = "\n".join(json.dumps(m) for m in msgs) + "\n"
    out, err = p.communicate(payload.encode("utf-8"), timeout=20)
    lines = [l for l in out.decode("utf-8").splitlines() if l.strip()]
    assert len(lines) == 3, "应恰好 3 条响应(initialize/tools/list/tools/call)，实际 %d" % len(lines)
    for line in lines:
        json.loads(line)
    print("✓ stdout 每行均为合法 JSON，共 %d 条响应（通知未产生响应）" % len(lines))


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n全部 %d 项通过。" % len(tests))
    return 0


if __name__ == "__main__":
    sys.exit(main())
