# -*- coding: utf-8 -*-
"""MCP server 核心 —— 纯标准库的 stdio + JSON-RPC 2.0 实现。

因本机唯一 Python 是 3.8（官方 mcp/fastmcp SDK 需 3.10+，且 server 要操作 ROS
必须用 py3.8 的 rospy），这里手写最小协议实现，只支持 tools-only 服务：
  - 传输: stdio，每行一条完整 JSON（json.dumps + "\\n"），stdout 只准输出协议帧
  - 必须处理的 method: initialize / notifications/initialized / ping /
    tools/list / tools/call / resources/list / prompts/list
  - 协议版本兼容 2024-11-05 与 2025-06-18（客户端版本在支持集内原样回显）
  - 工具执行失败一律走 result.isError=true（跨版本通用），不做 JSON-RPC error
  - 缺 "id" 的消息一律视为通知，不响应（避免客户端因意外响应断开）

日志一律走 stderr（logging），绝不 print 到 stdout。
"""

import json
import logging
import sys

logger = logging.getLogger(__name__)

SUPPORTED_VERSIONS = {"2024-11-05", "2025-06-18"}
LATEST_VERSION = "2025-06-18"
SERVER_INFO = {"name": "sweeper_mcp", "version": "0.3.0"}

# JSON-RPC 错误码
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


class MCPError(Exception):
    """协议层错误（对应 JSON-RPC error 对象）。"""

    def __init__(self, code, message, data=None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.data = data


def _result(rid, result):
    return {"jsonrpc": "2.0", "id": rid, "result": result}


def _error(rid, code, message, data=None):
    err = {"code": code, "message": message}
    if data is not None:
        err["data"] = data
    return {"jsonrpc": "2.0", "id": rid, "error": err}


def dispatch(req, registry):
    """处理一条请求/通知，返回响应 dict 或 None（通知不响应）。

    Args:
        req: 解析后的 JSON 对象。
        registry: 工具注册表，须提供 get(name) 与 schemas() 接口。
    """
    if not isinstance(req, dict) or req.get("jsonrpc") != "2.0" or "method" not in req:
        raise MCPError(INVALID_REQUEST, "Invalid Request")

    rid = req.get("id")
    if rid is None:
        # 通知：不响应
        return None

    method = req.get("method", "")
    params = req.get("params") or {}

    if method == "initialize":
        version = params.get("protocolVersion", "")
        return _result(rid, {
            "protocolVersion": version if version in SUPPORTED_VERSIONS else LATEST_VERSION,
            "capabilities": {"tools": {}},
            "serverInfo": SERVER_INFO,
        })

    if method == "ping":
        return _result(rid, {})

    if method == "tools/list":
        return _result(rid, {"tools": registry.schemas()})

    if method == "tools/call":
        name = params.get("name")
        arguments = params.get("arguments") or {}
        metadata = params.get("_meta") or {}
        control_token = metadata.get("controlToken", "")
        tool = registry.get(name)
        if tool is None:
            raise MCPError(INVALID_PARAMS, "Unknown tool: %s" % name)
        # 日志走 stderr（stdout 只准协议帧），供终端查看"server 收到的每一次工具调用"
        logger.info("[MCP] tools/call → %s(%s)", name,
                    json.dumps(arguments, ensure_ascii=False)[:300])
        res = tool.run(arguments, authorised=registry.is_authorised(control_token))
        logger.info("[MCP] tools/call ← %s | isError=%s | %s", name, res.is_error,
                    res.text[:200])
        return _result(rid, {
            "content": [{"type": "text", "text": res.text}],
            "isError": res.is_error,
        })

    if method == "resources/list":
        return _result(rid, {"resources": []})

    if method == "prompts/list":
        return _result(rid, {"prompts": []})

    # $/…、notifications/… 及一切未知通知：静默
    if method.startswith("$") or method.startswith("notifications/"):
        return None

    raise MCPError(METHOD_NOT_FOUND, "Method not found: %s" % method)


def serve_stdio(registry):
    """从 stdin 逐行读 JSON-RPC 请求，响应写 stdout（仅协议帧），日志走 stderr。"""
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req = None
        try:
            req = json.loads(line)
            resp = dispatch(req, registry)
        except MCPError as exc:
            resp = _error(req.get("id") if isinstance(req, dict) else None,
                          exc.code, exc.message, exc.data)
        except Exception as exc:
            logger.warning("内部错误: %s", exc)
            resp = _error(req.get("id") if isinstance(req, dict) else None,
                          INTERNAL_ERROR, "Internal error: %s" % exc)
        if resp is None:
            continue
        sys.stdout.write(json.dumps(resp, ensure_ascii=False) + "\n")
        sys.stdout.flush()
