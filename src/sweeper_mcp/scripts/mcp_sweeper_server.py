#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MCP server 入口 —— stdio + JSON-RPC（工具服务清扫车 ROS 操作）。

用法:
  MCP_BACKEND=mock python3 scripts/mcp_sweeper_server.py   # 离线测试（不碰 ROS）
  MCP_BACKEND=ros  python3 scripts/mcp_sweeper_server.py   # 真实 ROS（需已 source 环境）

日志一律走 stderr，stdout 只输出协议帧。
"""

import logging
import os
import sys

# 非安装环境可直接 import 本包（脚本在 scripts/，包在 src/ 下）
_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.mcp_jsonrpc import serve_stdio
from sweeper_mcp.tools import build_registry


def create_backend(name):
    if name == "mock":
        from sweeper_mcp.mock_backend import MockBackend
        return MockBackend()
    if name == "ros":
        from sweeper_mcp.ros_backend import ROSBackend
        return ROSBackend()
    raise ValueError("MCP_BACKEND must be ros or mock")


def main():
    # 日志一律走 stderr（stdout 只输出协议帧），INFO 级：每次 tools/call 都可见
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    backend_name = os.environ.get("MCP_BACKEND", "ros")
    backend = create_backend(backend_name)
    registry = build_registry(
        backend, control_token=os.environ.get("SWEEPER_MCP_CONTROL_TOKEN", ""))
    serve_stdio(registry)
    return 0


if __name__ == "__main__":
    sys.exit(main())
