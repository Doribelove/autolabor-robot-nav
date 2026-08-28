# -*- coding: utf-8 -*-
"""Deterministic offline backend used by MCP and agent tests."""

import json
import threading
import time

from sweeper_mcp.tools import ToolResult


class MockBackend:
    name = "mock"

    def __init__(self, delay=0.05, fail_tool=""):
        self.delay = delay
        self.fail_tool = fail_tool
        self.calls = []
        self.nav = {"state": "idle", "goal_id": ""}
        self.coverage = {"state": "IDLE", "batch_id": "", "ai_owned": False}
        self.visual = {"state": "RELATIVE_NAV_ACTIVE", "ai_owned": False}

    @staticmethod
    def _result(value, error=False):
        return ToolResult(json.dumps(value, ensure_ascii=False, sort_keys=True), error)

    def _record(self, name, arguments=None):
        self.calls.append((name, arguments or {}))
        if self.fail_tool == name:
            return ToolResult("mock failure: %s" % name, True)
        return None

    def _finish(self, target, terminal):
        def worker():
            time.sleep(self.delay)
            target["state"] = terminal
        threading.Thread(target=worker, daemon=True).start()

    def get_robot_status(self):
        self._record("get_robot_status")
        return self._result({"battery_percent": 86, "position": {"x": 0, "y": 0}})

    def get_navigation_status(self):
        return self._result(self.nav)

    def list_saved_coverage_regions(self):
        return self._result({
            "map_digest": "mock-map",
            "regions": [{"id": "region-a", "name": "A区"},
                        {"id": "region-b", "name": "B区"}],
        })

    def get_coverage_status(self):
        return self._result(self.coverage)

    def get_visual_servo_status(self):
        return self._result(self.visual)

    def navigate_relative(self, forward_m, left_m, delta_yaw_deg):
        failed = self._record("navigate_relative", locals())
        if failed:
            return failed
        self.nav = {"state": "active", "goal_id": "mock-relative"}
        self._finish(self.nav, "succeeded")
        return self._result(self.nav)

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        if failed:
            return failed
        self.nav = {"state": "active", "goal_id": "mock-map-goal"}
        self._finish(self.nav, "succeeded")
        return self._result(self.nav)

    def cancel_navigation(self):
        self.nav["state"] = "preempted"
        return self._result({"canceled": True, "goal_id": self.nav["goal_id"]})

    def start_spot_cleaning(self):
        failed = self._record("start_spot_cleaning")
        if failed:
            return failed
        self.visual = {"state": "FOD_ACTIVE", "ai_owned": True}
        self._finish(self.visual, "RELATIVE_NAV_ACTIVE")
        return self._result(self.visual)

    def stop_spot_cleaning(self):
        self.visual = {"state": "RELATIVE_NAV_ACTIVE", "ai_owned": False}
        return self._result(self.visual)

    def start_coverage_cleaning(self, regions, operation_width_m=1.0,
                                overlap_percent=15.0, max_speed_mps=0.8,
                                allow_reverse_transit=True,
                                reverse_speed_mps=0.3,
                                max_angular_speed_rps=0.6,
                                linear_accel_mps2=2.0,
                                angular_accel_rps2=0.5,
                                direction_change_penalty_sec=1.0,
                                segment_handoff_penalty_sec=0.5):
        failed = self._record("start_coverage_cleaning", locals())
        if failed:
            return failed
        self.coverage = {
            "state": "SWEEPING", "batch_active": True,
            "batch_id": "mock-batch", "ai_owned": True,
        }

        def finish():
            time.sleep(self.delay)
            self.coverage.update(state="COMPLETED", batch_active=False)
        threading.Thread(target=finish, daemon=True).start()
        return self._result(self.coverage)

    def pause_coverage(self):
        self.coverage["state"] = "PAUSED"
        return self._result(self.coverage)

    def resume_coverage(self):
        self.coverage["state"] = "SWEEPING"
        return self._result(self.coverage)

    def skip_coverage_region(self):
        return self._result({"skipped": True})

    def cancel_coverage(self):
        self.coverage.update(state="CANCELED", batch_active=False)
        return self._result(self.coverage)
