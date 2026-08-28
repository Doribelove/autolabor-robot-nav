# -*- coding: utf-8 -*-
"""OpenAI 兼容 LLM 客户端 —— 含 tools/function calling 支持（M3 Agent 循环用）。

在旧 llm_client 的重试/退避思路上新建独立模块（不耦合旧代码）：
- chat(messages, tools=None) → 返回 assistant message dict（含 content / tool_calls）
- 复用：网络异常 / HTTP 429 / 5xx 指数退避重试；HTTP 4xx 不重试直接抛
- function calling 时**不用** response_format=json_object（会与 tools 冲突）

api_key 从构造参数或 DEEPSEEK_API_KEY 读取，不写日志、不进入 ROS 参数。
"""

import json
import logging
import os
import time

import requests

logger = logging.getLogger(__name__)


class LLMError(Exception):
    """LLM 调用相关错误基类。"""


class LLMRequestError(LLMError):
    """网络/HTTP 层错误。"""


class LLMResponseError(LLMError):
    """响应结构异常，取不到 assistant message。"""


class OpenAIClient:
    """OpenAI 兼容聊天补全客户端（支持 tools）。"""

    def __init__(self, base_url="https://api.deepseek.com", model="deepseek-v4-flash",
                 api_key=None, temperature=0.2, timeout_s=20.0, max_retries=3):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key or os.environ.get("DEEPSEEK_API_KEY", "")
        self.temperature = temperature
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self.last_metrics = {
            "rtt_ms": 0.0, "http_status": 0, "request_id": "", "usage": {},
        }

    def chat(self, messages, tools=None, tool_choice="auto"):
        """调用 /v1/chat/completions，返回 assistant message dict。

        返回 dict 含：
          - "content": 最终文本（可能为 None，当模型在调工具时）
          - "tool_calls": [{id,type,function:{name,arguments}}]（可能为 None）

        Raises:
            LLMRequestError / LLMResponseError
        """
        if not self.api_key:
            raise LLMRequestError(
                "未配置 DEEPSEEK_API_KEY（请 export DEEPSEEK_API_KEY=...）。")

        url = "%s/v1/chat/completions" % self.base_url
        headers = {
            "Authorization": "Bearer %s" % self.api_key,
            "Content-Type": "application/json",
        }
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature,
            # DeepSeek V4 defaults to thinking mode.  The control agent uses
            # deterministic non-thinking tool calls as explicitly selected by
            # the operator.
            "thinking": {"type": "disabled"},
        }
        if tools:
            payload["tools"] = tools
            payload["tool_choice"] = tool_choice

        last_err = None
        for attempt in range(1, self.max_retries + 1):
            started = time.monotonic()
            try:
                resp = requests.post(url, headers=headers, json=payload, timeout=self.timeout_s)
            except requests.RequestException as exc:
                self.last_metrics = {
                    "rtt_ms": (time.monotonic() - started) * 1000.0,
                    "http_status": 0, "request_id": "", "usage": {},
                }
                last_err = exc
                logger.warning("LLM 请求异常(第 %d/%d 次): %s", attempt, self.max_retries, exc)
                self._backoff(attempt)
                continue

            try:
                body = resp.json()
            except ValueError:
                body = None
            self.last_metrics = {
                "rtt_ms": (time.monotonic() - started) * 1000.0,
                "http_status": int(resp.status_code),
                "request_id": resp.headers.get("x-request-id", ""),
                "usage": body.get("usage", {}) if isinstance(body, dict) else {},
            }

            if resp.status_code == 429 or resp.status_code >= 500:
                last_err = LLMRequestError("HTTP %s 限流或服务端错误: %s" % (
                    resp.status_code, resp.text[:200]))
                logger.warning("LLM HTTP %s(第 %d/%d 次)", resp.status_code, attempt, self.max_retries)
                self._backoff(attempt)
                continue

            if resp.status_code != 200:
                raise LLMRequestError("HTTP %s: %s" % (resp.status_code, resp.text[:300]))

            if body is None:
                raise LLMResponseError("HTTP 200 但响应不是有效 JSON")
            return self._extract_message(body)

        raise LLMRequestError("LLM 调用重试 %d 次仍失败: %s" % (self.max_retries, last_err))

    @staticmethod
    def _backoff(attempt):
        """指数退避: 0.5s, 1s, 2s ...（封顶 4s）。"""
        time.sleep(min(0.5 * (2 ** (attempt - 1)), 4.0))

    @staticmethod
    def _extract_message(resp_json):
        try:
            choices = resp_json["choices"]
            if not choices:
                raise LLMResponseError("响应 choices 为空")
            msg = choices[0]["message"]
        except (KeyError, TypeError, IndexError) as exc:
            raise LLMResponseError(
                "响应结构异常，无法解析 message: %s | raw=%s" % (
                    exc, json.dumps(resp_json, ensure_ascii=False)[:300]))
        if not isinstance(msg, dict):
            raise LLMResponseError("响应 message 不是对象")
        return msg
