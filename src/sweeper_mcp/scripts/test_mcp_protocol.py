#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline MCP protocol and safety-boundary tests."""

import json
import os
import subprocess
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.mcp_client import MCPClient, MCPClientError  # noqa: E402

SERVER = os.path.abspath(os.path.join(_THIS, "mcp_sweeper_server.py"))
ENV = dict(os.environ, MCP_BACKEND="mock")
CONTROL_TOKEN = "offline-test-capability"

EXPECTED_TOOLS = {
    "get_robot_status",
    "get_navigation_status",
    "list_saved_coverage_regions",
    "get_coverage_status",
    "get_visual_servo_status",
    "navigate_relative",
    "navigate_map_pose",
    "cancel_navigation",
    "start_spot_cleaning",
    "stop_spot_cleaning",
    "start_coverage_cleaning",
    "pause_coverage",
    "resume_coverage",
    "skip_coverage_region",
    "cancel_coverage",
}
REMOVED_TOOLS = {
    "navigate_gps", "navigate_pose", "emergency_stop", "sweep_set",
    "sweep_coverage", "set_fod_mode", "navigation_status",
}


def _client(token=""):
    return MCPClient(
        server_cmd=[sys.executable, SERVER], env=ENV, control_token=token)


def test_initialize_and_tool_catalogue():
    client = _client()
    try:
        result = client.initialize("2024-11-05")
        assert result["protocolVersion"] == "2024-11-05"
        assert result["capabilities"].get("tools") is not None
        tools = client.list_tools()["tools"]
        names = {item["name"] for item in tools}
        assert names == EXPECTED_TOOLS, (names ^ EXPECTED_TOOLS)
        assert names.isdisjoint(REMOVED_TOOLS)
        by_name = {item["name"]: item for item in tools}
        coverage_properties = by_name[
            "start_coverage_cleaning"]["inputSchema"]["properties"]
        assert coverage_properties["operation_width_m"]["default"] == 1.0
        assert coverage_properties["overlap_percent"]["default"] == 15.0
        assert coverage_properties["max_speed_mps"]["default"] == 0.8
        assert coverage_properties["allow_reverse_transit"]["default"] is True
        assert "只显式取消当前 AI 会话" in by_name[
            "cancel_navigation"]["description"]
        for item in tools:
            assert {"name", "description", "inputSchema"}.issubset(item)
            assert item["inputSchema"].get("additionalProperties") is False
    finally:
        client.close()


def test_read_only_call_needs_no_capability():
    client = _client()
    try:
        client.initialize()
        result = client.call_tool("get_robot_status")
        assert not result["is_error"], result
        assert "battery_percent" in result["text"]
    finally:
        client.close()


def test_mutating_call_is_fail_closed_without_capability():
    client = _client()
    try:
        client.initialize()
        result = client.call_tool("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        })
        assert result["is_error"], result
        assert "授权" in result["text"]
    finally:
        client.close()


def test_mutating_call_accepts_private_capability():
    client = _client(CONTROL_TOKEN)
    try:
        client.initialize()
        result = client.call_tool("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }, authorised=True)
        assert not result["is_error"], result
        assert json.loads(result["text"])["state"] == "active"
    finally:
        client.close()


def test_strict_argument_validation():
    client = _client(CONTROL_TOKEN)
    try:
        client.initialize()
        missing = client.call_tool(
            "navigate_relative", {"forward_m": 1.0}, authorised=True)
        assert missing["is_error"] and "缺少必填参数" in missing["text"]
        extra = client.call_tool("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
            "latitude": 39.0,
        }, authorised=True)
        assert extra["is_error"] and "未定义参数" in extra["text"]
        out_of_range = client.call_tool("navigate_relative", {
            "forward_m": 31.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }, authorised=True)
        assert out_of_range["is_error"] and "不能大于" in out_of_range["text"]
    finally:
        client.close()


def test_unknown_tool_and_method_errors():
    client = _client()
    try:
        client.initialize()
        try:
            client.call_tool("navigate_gps")
            assert False, "removed tool must not be callable"
        except MCPClientError as exc:
            assert exc.code == -32602
        try:
            client.request("no_such_method")
            assert False, "unknown method must fail"
        except MCPClientError as exc:
            assert exc.code == -32601
    finally:
        client.close()


def test_stdout_contains_only_json_rpc_frames():
    process = subprocess.Popen(
        [sys.executable, SERVER], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=ENV)
    requests = [
        {"jsonrpc": "2.0", "id": 1, "method": "initialize",
         "params": {"protocolVersion": "2025-06-18", "capabilities": {},
                    "clientInfo": {"name": "test", "version": "1"}}},
        {"jsonrpc": "2.0", "method": "notifications/initialized"},
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
        {"jsonrpc": "2.0", "id": 3, "method": "tools/call",
         "params": {"name": "get_robot_status", "arguments": {}}},
    ]
    payload = ("\n".join(json.dumps(item) for item in requests) + "\n").encode()
    stdout, _stderr = process.communicate(payload, timeout=20)
    lines = [line for line in stdout.decode().splitlines() if line.strip()]
    assert len(lines) == 3
    assert [json.loads(line)["id"] for line in lines] == [1, 2, 3]


def test_mcp_child_receives_only_its_narrow_control_capability():
    child = (
        "import json,os,sys\n"
        "for line in sys.stdin:\n"
        " r=json.loads(line); keys=['DEEPSEEK_API_KEY',"
        "'SWEEPER_AI_SESSION_TOKEN','SWEEPER_ASR_CAPABILITY',"
        "'SWEEPER_MCP_CONTROL_TOKEN']; "
        "print(json.dumps({'id':r['id'],'result':{k:os.environ.get(k) "
        "for k in keys}}),flush=True)\n"
    )
    env = dict(os.environ)
    env.update({
        "DEEPSEEK_API_KEY": "cloud-secret",
        "SWEEPER_AI_SESSION_TOKEN": "ui-secret",
        "SWEEPER_ASR_CAPABILITY": "asr-secret",
        "SWEEPER_MCP_CONTROL_TOKEN": "inherited-secret",
    })
    client = MCPClient(
        server_cmd=[sys.executable, "-c", child], env=env,
        control_token=CONTROL_TOKEN)
    try:
        observed = client.request("inspect")
        assert observed["DEEPSEEK_API_KEY"] is None
        assert observed["SWEEPER_AI_SESSION_TOKEN"] is None
        assert observed["SWEEPER_ASR_CAPABILITY"] is None
        assert observed["SWEEPER_MCP_CONTROL_TOKEN"] == CONTROL_TOKEN
    finally:
        client.close()


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d MCP tests passed" % len(tests))
