# -*- coding: utf-8 -*-
"""sweeper_mcp —— 清扫车 MCP 服务 + LLM 语音自动执行。

- mcp_jsonrpc : 纯标准库的 MCP stdio server（JSON-RPC 帧 + dispatch）
- tools       : 工具注册表（10 个工具，schema 与 handler 绑定）
- mock_backend: 离线测试桩（MCP_BACKEND=mock）
- ros_backend : 真实 ROS 后端（MCP_BACKEND=ros，懒加载 rospy）
- sweep_backend: 清扫装置 TCP 后端（复用 SweepDeviceControl）
- voice       : AI 语音子模块（asr_audio 录音 / asr_recognizer whisper 识别，2026-08-17 由
                ai_task_decomposition 迁移合并而来）
- llm         : OpenAI 兼容客户端（含 tools/function calling）
- mcp_client  : stdio 子进程客户端
- agent       : Agent 循环 run_agent(text)
"""

__version__ = "0.2.0"
