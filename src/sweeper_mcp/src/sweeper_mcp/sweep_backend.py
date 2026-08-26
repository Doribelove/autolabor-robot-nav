# -*- coding: utf-8 -*-
"""清扫装置后端 —— 复用 SweepDeviceControl 的 TCP 客户端（192.168.1.197:50003）。

⚠️ 关键坑：SweepDeviceControl 内部用 print() 打日志；在 MCP server 进程内以库方式
调用会把日志打进 stdout 协议流，破坏 JSON-RPC 帧。这里统一用 contextlib.redirect_stdout
把调用期间的 stdout 重定向到 stderr，保证协议流纯净。

当前能力（与设备协议一致）：
  - sweep_set(action): on/off/toggle 开关（仅启/停，无转速/模式）
  - sweep_status_text(): 查询状态，返回 "on"/"off"/"N/A"
  - sweep_coverage(): 全覆盖清扫【预留接口，尚未实现】
"""

import contextlib
import os
import sys

from sweeper_mcp.tools import ToolResult

DEFAULT_IP = "192.168.1.197"
DEFAULT_PORT = 50003
DEFAULT_TIMEOUT = 5


def _sweep_module_dir():
    """定位 SweepDeviceControl 目录：源码树 / devel 安装环境均可，可用环境变量覆盖。"""
    env = os.environ.get("SWEEP_DEVICE_DIR")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    # 源码树：<工作区>/src/SweepDeviceControl
    cand = os.path.abspath(os.path.join(here, "..", "..", "..", "SweepDeviceControl"))
    if os.path.isdir(cand):
        return cand
    # 安装(devel)环境：向上逐级找 <某工作区>/src/SweepDeviceControl
    d = here
    for _ in range(8):
        d = os.path.dirname(d)
        p = os.path.join(d, "src", "SweepDeviceControl")
        if os.path.isdir(p):
            return p
    return cand


def _client():
    module_dir = _sweep_module_dir()
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)
    from SweepDeviceControl import SweepDeviceTCPClient
    return SweepDeviceTCPClient(
        ip=os.environ.get("SWEEP_IP", DEFAULT_IP),
        port=int(os.environ.get("SWEEP_PORT", DEFAULT_PORT)),
        timeout=int(os.environ.get("SWEEP_TIMEOUT", DEFAULT_TIMEOUT)),
    )


@contextlib.contextmanager
def _redirect_stdout_to_stderr():
    """SweepDeviceControl 的 print 日志 → stderr，避免污染 MCP 协议流。"""
    with contextlib.redirect_stdout(sys.stderr):
        yield


def sweep_status_text():
    """查询清扫装置状态，返回 "on"/"off"/"N/A"（查询失败或设备掉线）。"""
    try:
        client = _client()
        with _redirect_stdout_to_stderr():
            if not client.connect():
                client.close()
                return "N/A"
            state = client.get_device_status()
            client.close()
        return {1: "on", 0: "off"}.get(state, "N/A")
    except Exception:
        return "N/A"


def sweep_set(action):
    """开关清扫装置。action: on/off/toggle。返回 ToolResult。"""
    client = _client()
    try:
        with _redirect_stdout_to_stderr():
            if not client.connect():
                client.close()
                return ToolResult("清扫装置通信失败(%s:%s)，请检查设备电源/网络。" % (DEFAULT_IP, DEFAULT_PORT), True)

            state = client.get_device_status()
            if action == "toggle":
                ok = client.toggle_sweep_device()
                state = client.get_device_status()
                client.close()
                desc = {1: "开启", 0: "关闭"}.get(state, "未知")
                if ok:
                    return ToolResult("清扫装置已切换，当前状态: %s" % desc, False)
                return ToolResult("清扫装置切换操作失败。", True)

            target = 1 if action == "on" else 0
            if state == target:
                client.close()
                return ToolResult("清扫装置已是%s状态。" % ("开启" if target == 1 else "关闭"), False)
            ok = client.toggle_sweep_device()
            client.close()
            desc = "开启" if target == 1 else "关闭"
            return ToolResult("清扫装置已%s。" % desc if ok else "清扫装置操作失败。", not ok)
    except Exception as exc:
        try:
            client.close()
        except Exception:
            pass
        return ToolResult("清扫装置操作异常: %s" % exc, True)


def sweep_coverage(area=None, pattern=None, duration=None, width=None):
    """全覆盖清扫 —— 预留接口，尚未实现。"""
    return ToolResult("全覆盖清扫尚未实现（接口已预留）。当前仅支持定点清扫开关 sweep_set(on/off/toggle)。", True)
