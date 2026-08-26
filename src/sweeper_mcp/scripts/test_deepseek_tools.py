#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探针：验证当前模型是否支持 tools(function calling)（M3）。

只调一次 OpenAI 兼容接口，看模型是否返回 tool_calls。不依赖 MCP / ROS。

用法:
  export DEEPSEEK_API_KEY=...
  python3 scripts/test_deepseek_tools.py
  # 也可用 DEEPSEEK_BASE_URL / DEEPSEEK_MODEL 覆盖配置

退出码: 0=支持 tools(返回了 tool_calls)  2=调用失败  3=未返回 tool_calls
"""

import os
import sys

_THIS = os.path.dirname(os.path.abspath(__file__))
_PKG_SRC = os.path.abspath(os.path.join(_THIS, "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.llm import OpenAIClient  # noqa: E402


def _config_api_key():
    """从 config/sweeper_mcp.yaml 读默认密钥（环境变量优先）。"""
    yaml_path = os.path.abspath(os.path.join(_THIS, "..", "config", "sweeper_mcp.yaml"))
    try:
        import yaml
        with open(yaml_path, "r", encoding="utf-8") as f:
            return (yaml.safe_load(f) or {}).get("agent", {}).get("api_key", "")
    except Exception:
        return ""


def main():
    base_url = os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com")
    model = os.environ.get("DEEPSEEK_MODEL", "deepseek-v4-flash")
    api_key = os.environ.get("DEEPSEEK_API_KEY", "") or _config_api_key()
    if not api_key:
        print("未配置 api_key（环境变量 DEEPSEEK_API_KEY 或 config/sweeper_mcp.yaml）。", file=sys.stderr)
        return 1

    client = OpenAIClient(base_url=base_url, model=model, api_key=api_key, temperature=0.0)
    messages = [
        {"role": "system", "content": "测试环境：请调用 get_weather 工具查询北京天气，然后简短汇报。"},
        {"role": "user", "content": "北京天气怎么样？"},
    ]
    tools = [{
        "type": "function",
        "function": {
            "name": "get_weather",
            "description": "查询指定城市的天气",
            "parameters": {
                "type": "object",
                "properties": {"city": {"type": "string"}},
                "required": ["city"],
            },
        },
    }]

    print("探测模型: %s (%s)" % (model, base_url), flush=True)
    try:
        msg = client.chat(messages, tools=tools)
    except Exception as exc:
        print("调用失败: %s" % exc, file=sys.stderr)
        return 2

    calls = msg.get("tool_calls")
    if calls:
        print("✓ %s 支持 tools，返回 %d 个 tool_call：" % (model, len(calls)))
        for tc in calls:
            print("  - %s(%s)" % (tc["function"]["name"], tc["function"].get("arguments")))
        return 0

    print("✗ %s 未返回 tool_calls（可能不支持 tools）。模型答复: %s" % (
        model, (msg.get("content") or "")[:200]))
    return 3


if __name__ == "__main__":
    sys.exit(main())
