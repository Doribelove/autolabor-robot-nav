# -*- coding: utf-8 -*-
"""MCP stdio 子进程客户端 —— Agent Host 侧连接 MCP server 用。

以子进程 spawn mcp_sweeper_server.py，通过 stdin/stdout 交换 JSON-RPC（每行一条）。
后台读线程按 id 路由响应，支持并发请求；call_tool 遇 isError 不抛异常（返回 is_error 标记），
协议层错误（未知方法/未知工具等）抛 MCPClientError。
"""

import json
import os
import subprocess
import sys
import threading


class MCPClientError(Exception):
    """协议层错误（对应 JSON-RPC error 对象）。"""

    def __init__(self, code, message, data=None):
        super().__init__("%s (code=%s)" % (message, code))
        self.code = code
        self.message = message
        self.data = data


def default_server_cmd():
    """定位 MCP server 脚本：兼容源码树与 catkin 安装(devel)两种布局。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 源码树：<包根>/scripts/mcp_sweeper_server.py
        os.path.join(here, "..", "..", "scripts", "mcp_sweeper_server.py"),
        # devel 安装：devel/lib/sweeper_mcp/mcp_sweeper_server.py
        os.path.join(here, "..", "..", "..", "sweeper_mcp", "mcp_sweeper_server.py"),
    ]
    for cand in candidates:
        path = os.path.abspath(cand)
        if os.path.exists(path):
            return [sys.executable, path]
    return [sys.executable, os.path.abspath(candidates[0])]


class MCPClient:
    def __init__(self, server_cmd=None, env=None, server_env_backend=None,
                 control_token=""):
        """spawn MCP server 子进程。

        Args:
            server_cmd: 启动命令(list)；默认用 default_server_cmd()。
            env: 传给子进程的环境(dict)；缺省继承 os.environ。
            server_env_backend: MCP_BACKEND 值（mock/ros），None 则继承环境。
        """
        self._cmd = server_cmd or default_server_cmd()
        child_env = dict(os.environ if env is None else env)
        # The MCP tool subprocess never calls the cloud model and must not be
        # able to impersonate the Qt authorization session.  Only the narrow
        # MCP capability created below is deliberately passed to it.
        child_env.pop("DEEPSEEK_API_KEY", None)
        child_env.pop("SWEEPER_AI_SESSION_TOKEN", None)
        child_env.pop("SWEEPER_ASR_CAPABILITY", None)
        child_env.pop("SWEEPER_MCP_CONTROL_TOKEN", None)
        if server_env_backend:
            child_env["MCP_BACKEND"] = server_env_backend
        self._control_token = control_token or ""
        if self._control_token:
            child_env["SWEEPER_MCP_CONTROL_TOKEN"] = self._control_token
        self._proc = subprocess.Popen(
            self._cmd,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, encoding="utf-8", bufsize=1, env=child_env,
        )
        self._next_id = 0
        self._lock = threading.Lock()
        self._pending = {}
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        for line in self._proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            rid = msg.get("id")
            with self._lock:
                entry = self._pending.get(rid) if rid is not None else None
            if entry is None:
                continue
            entry["msg"] = msg
            entry["event"].set()

    def request(self, method, params=None, timeout=30.0):
        with self._lock:
            self._next_id += 1
            rid = self._next_id
            entry = {"msg": None, "event": threading.Event()}
            self._pending[rid] = entry
            req = {"jsonrpc": "2.0", "id": rid, "method": method}
            if params is not None:
                req["params"] = params
            line = json.dumps(req, ensure_ascii=False) + "\n"
            try:
                self._proc.stdin.write(line)
                self._proc.stdin.flush()
            except Exception as exc:
                self._pending.pop(rid, None)
                raise MCPClientError(-32000, "写入 MCP server 失败: %s" % exc)

        if not entry["event"].wait(timeout):
            self._pending.pop(rid, None)
            raise MCPClientError(-32000, "请求超时: %s" % method)
        with self._lock:
            self._pending.pop(rid, None)
        msg = entry["msg"]
        if "error" in msg:
            err = msg["error"]
            raise MCPClientError(err.get("code"), err.get("message", ""), err.get("data"))
        return msg.get("result")

    def initialize(self, protocol_version="2025-06-18"):
        """MCP 握手，返回 server 的 initialize 结果。"""
        result = self.request("initialize", {
            "protocolVersion": protocol_version,
            "capabilities": {},
            "clientInfo": {"name": "sweeper_mcp_client", "version": "0.1.0"},
        })
        self.notify("notifications/initialized", {})
        return result

    def notify(self, method, params=None):
        """Send one JSON-RPC notification; notifications have no response."""
        request = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            request["params"] = params
        line = json.dumps(request, ensure_ascii=False) + "\n"
        with self._lock:
            try:
                self._proc.stdin.write(line)
                self._proc.stdin.flush()
            except Exception as exc:
                raise MCPClientError(
                    -32000, "写入 MCP server 通知失败: %s" % exc)

    def list_tools(self):
        """返回 tools/list 结果（{tools:[...]}）。"""
        return self.request("tools/list")

    def call_tool(self, name, arguments=None, timeout=30.0, authorised=False):
        """调用工具。isError 不抛异常，返回 {"text": str, "is_error": bool}。"""
        params = {"name": name, "arguments": arguments or {}}
        if authorised and self._control_token:
            params["_meta"] = {"controlToken": self._control_token}
        res = self.request("tools/call", params, timeout=timeout)
        content = res.get("content") or []
        text = content[0]["text"] if content and content[0].get("type") == "text" else ""
        return {"text": text, "is_error": bool(res.get("isError", False))}

    def close(self):
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.terminate()
            self._proc.wait(timeout=5)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
