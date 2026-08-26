# -*- coding: utf-8 -*-
"""Mock 后端 —— 离线测试桩（MCP_BACKEND=mock），不碰 ROS/网络。

返回仿真数据，用于 M1 协议测试与 M3 Agent 循环的无 ROS 端到端验证。
与 ros_backend 实现完全相同的 9 个 handler 方法接口。
"""

import json
import threading
import time

from sweeper_mcp.tools import ToolResult


class MockBackend:
    name = "mock"

    def __init__(self, nav_delay=1.5):
        # 模拟状态，便于多次调用有连续感
        self._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._sweep = "off"
        self._nav_status = "idle"
        self._fod = False
        # 模拟异步导航：发布目标后 delay 秒自动"到达"（succeeded），
        # 让"规划+顺序执行+导航监控"在离线 mock 下也能完整走通。
        self._nav_delay = nav_delay

    @staticmethod
    def _json_text(data):
        return json.dumps(data, ensure_ascii=False)

    def _simulate_nav(self, delay=None):
        """后台线程模拟导航完成：active → (delay 秒后) succeeded。"""
        delay = self._nav_delay if delay is None else delay

        def _finish():
            time.sleep(delay)
            if self._nav_status == "active":
                self._nav_status = "succeeded"

        threading.Thread(target=_finish, daemon=True).start()

    def get_robot_status(self):
        data = {
            "battery_percent": 86,
            "emergency": {"hard": False, "soft": False, "gamepad": False, "robot": False},
            "position": self._pose,
            "sweep_state": self._sweep,
            "navigation_mode": "FOD" if self._fod else "GPS",
        }
        return ToolResult(self._json_text(data), False)

    def navigate_pose(self, x, y, yaw=0.0, frame_id="camera_init"):
        self._nav_status = "active"
        self._simulate_nav()
        return ToolResult(
            "已发布导航目标: x=%.3f y=%.3f yaw=%.3f (frame=%s)，异步执行中（用 navigation_status 查询）。"
            % (x, y, yaw, frame_id), False)

    def navigate_relative(self, dx=0.0, dy=0.0, dyaw=0.0):
        # mock：以 (0,0,0) 为当前位姿演示换算
        import math
        cx, cy, cyaw = self._pose["x"], self._pose["y"], self._pose["yaw"]
        tx = cx + dx * math.cos(cyaw) - dy * math.sin(cyaw)
        ty = cy + dx * math.sin(cyaw) + dy * math.cos(cyaw)
        tyaw = cyaw + dyaw
        self._nav_status = "active"
        self._simulate_nav()
        return ToolResult(
            "已按相对位移换算目标并发布: 当前(%s) → 目标 x=%.3f y=%.3f yaw=%.3f"
            % (self._pose, tx, ty, tyaw), False)

    def navigate_gps(self, latitude, longitude, altitude=None):
        self._nav_status = "active"
        self._simulate_nav()
        txt = "已发布 GPS 目标: lat=%.6f lon=%.6f" % (latitude, longitude)
        if altitude is not None:
            txt += " alt=%.1f" % altitude
        return ToolResult(txt, False)

    def cancel_navigation(self):
        self._nav_status = "idle"
        return ToolResult("已取消当前导航目标。", False)

    def navigation_status(self):
        return ToolResult(self._nav_status, False)

    def emergency_stop(self, active, reason=None):
        txt = "已触发急停。" if active else "已解除急停。"
        if reason:
            txt += " 原因: %s" % reason
        return ToolResult(txt, False)

    def sweep_set(self, action):
        if action == "on":
            self._sweep = "on"
            return ToolResult("清扫装置已开启。", False)
        if action == "off":
            self._sweep = "off"
            return ToolResult("清扫装置已关闭。", False)
        # toggle
        self._sweep = "on" if self._sweep == "off" else "off"
        return ToolResult("清扫装置已切换，当前状态: %s" % self._sweep, False)

    def sweep_coverage(self, area=None, pattern=None, duration=None, width=None):
        return ToolResult("全覆盖清扫尚未实现（接口已预留）。当前仅支持定点清扫开关 sweep_set(on/off/toggle)。", True)

    def set_fod_mode(self, enabled):
        self._fod = enabled
        return ToolResult("已切换到 %s 模式。" % ("FOD 视觉回收" if enabled else "GPS"), False)
