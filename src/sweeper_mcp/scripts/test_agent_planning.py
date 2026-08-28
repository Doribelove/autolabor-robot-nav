#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline plan validation, authorization and sequential execution tests."""

import json
import os
import sys
import threading

_PKG_SRC = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

from sweeper_mcp.agent import AgentError, AgentRunner  # noqa: E402
from sweeper_mcp.mock_backend import MockBackend  # noqa: E402
from sweeper_mcp.tools import ToolResult, build_registry  # noqa: E402


class FakeLLM:
    def __init__(self, document):
        self.document = document
        self.last_metrics = {
            "rtt_ms": 12.5, "http_status": 200,
            "request_id": "offline", "usage": {},
        }
        self.calls = 0

    def chat(self, _messages, tools=None, tool_choice=None):
        self.calls += 1
        if tools:
            assert tool_choice["function"]["name"] == "submit_plan"
            return {"tool_calls": [{
                "id": "plan-1", "type": "function",
                "function": {
                    "name": "submit_plan",
                    "arguments": json.dumps(self.document, ensure_ascii=False),
                },
            }]}
        return {"content": "离线执行总结。"}


class LocalMCP:
    def __init__(self, backend=None):
        self.backend = backend or MockBackend(delay=0.005)
        self.registry = build_registry(self.backend, control_token="capability")

    @staticmethod
    def initialize():
        return {}

    def list_tools(self):
        return {"tools": self.registry.schemas()}

    def call_tool(self, name, arguments=None, timeout=30.0, authorised=False):
        del timeout
        tool = self.registry.get(name)
        assert tool is not None
        result = tool.run(arguments or {}, authorised=authorised)
        return {"text": result.text, "is_error": result.is_error}


class ScriptedNavigationBackend(MockBackend):
    """Deterministic navigation handshake without ROS or worker threads."""

    def __init__(self, initial_error=False):
        super().__init__(delay=0.0)
        self.initial_error = bool(initial_error)
        self.navigation_status_calls = 0
        self._navigation_states = ["active", "succeeded"]

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        if self.initial_error:
            return self._result({
                "accepted": False,
                "error": "move_base goal submission failed",
            }, error=True)
        return self._result({
            "accepted": True,
            "state": "pending_goal_id",
        })

    def get_navigation_status(self):
        self.navigation_status_calls += 1
        state = self._navigation_states.pop(0)
        return self._result({
            "state": state,
            "goal_id": "move_base-test-goal" if state != "active" else "",
        })


class UncertainNavigationBackend(MockBackend):
    """A goal was published, but its initial ownership confirmation failed."""

    def __init__(self, states=None):
        super().__init__(delay=0.0)
        self.goal_id = "sweeper-ai-" + "a" * 32
        self.navigation_status_calls = 0
        self.cancel_calls = 0
        self.states = list(states or ["cancel_uncertain", "recalled"])

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        return self._result({
            "accepted": False,
            "error": "explicit GoalID confirmation timed out",
            "goal_id": self.goal_id,
            "cancel_state": "uncertain",
        }, error=True)

    def get_navigation_status(self):
        self.navigation_status_calls += 1
        state = self.states.pop(0) if len(self.states) > 1 else self.states[0]
        return self._result({"state": state, "goal_id": self.goal_id})

    def cancel_navigation(self):
        self.cancel_calls += 1
        return self._result({
            "canceled": False,
            "goal_id": self.goal_id,
            "cancel_state": "uncertain",
        }, error=True)


class LostAfterAcceptanceBackend(UncertainNavigationBackend):
    def __init__(self):
        super().__init__(["lost", "cancel_uncertain", "recalled"])

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        return self._result({
            "accepted": True, "state": "active", "goal_id": self.goal_id,
        })


class RecallingThenRecalledNavigationBackend(UncertainNavigationBackend):
    """Hold cleanup at RECALLING until the test permits a true terminal."""

    def __init__(self):
        super().__init__(["recalling"])
        self.two_recalling_cancels = threading.Event()
        self.allow_recalled = threading.Event()

    def get_navigation_status(self):
        self.navigation_status_calls += 1
        if self.navigation_status_calls <= 2:
            return self._result({
                "state": "recalling", "goal_id": self.goal_id,
            })
        if not self.allow_recalled.wait(2.0):
            raise AssertionError("test did not release the recalled terminal state")
        return self._result({"state": "recalled", "goal_id": self.goal_id})

    def cancel_navigation(self):
        result = super().cancel_navigation()
        if self.cancel_calls >= 2:
            self.two_recalling_cancels.set()
        return result


class StatusErrorAfterAcceptanceBackend(UncertainNavigationBackend):
    """A status transport error must enter exact-ID cleanup, not end the UI."""

    def __init__(self):
        super().__init__(["cancel_uncertain", "recalled"])
        self.raise_once = True

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        return self._result({
            "accepted": True, "state": "active", "goal_id": self.goal_id,
        })

    def get_navigation_status(self):
        if self.raise_once:
            self.raise_once = False
            raise RuntimeError("simulated status transport failure")
        return super().get_navigation_status()


class CancelDuringNavigationBackend(UncertainNavigationBackend):
    def __init__(self):
        super().__init__(["recalled"])
        self.cancel_callback = None
        self.first_status = True

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        return self._result({
            "accepted": True, "state": "active", "goal_id": self.goal_id,
        })

    def get_navigation_status(self):
        self.navigation_status_calls += 1
        if self.first_status:
            self.first_status = False
            result = self._result({"state": "active", "goal_id": self.goal_id})
            self.cancel_callback()
            return result
        return self._result({"state": "recalled", "goal_id": self.goal_id})


class MismatchedNavigationStatusBackend(UncertainNavigationBackend):
    def __init__(self):
        super().__init__(["recalled"])
        self.first_status = True

    def navigate_map_pose(self, x_m, y_m, yaw_deg):
        failed = self._record("navigate_map_pose", locals())
        assert failed is None
        return self._result({
            "accepted": True, "state": "active", "goal_id": self.goal_id,
        })

    def get_navigation_status(self):
        self.navigation_status_calls += 1
        if self.first_status:
            self.first_status = False
            return self._result({
                "state": "succeeded",
                "goal_id": "sweeper-ai-" + "b" * 32,
            })
        return self._result({"state": "recalled", "goal_id": self.goal_id})


class CoverageCleanupBackend(MockBackend):
    """FAILED with active ownership is not a safe batch terminal."""

    def __init__(self):
        super().__init__(delay=0.0)
        self.batch_id = "mock-cleanup-batch"
        self.coverage_status_calls = 0
        self.cancel_calls = 0
        self.coverage = {
            "state": "FAILED", "active": True, "batch_active": True,
            "batch_id": self.batch_id, "ai_owned": True,
        }

    def start_coverage_cleaning(self, regions, operation_width_m=1.0,
                                overlap_percent=15.0, max_speed_mps=0.8,
                                allow_reverse_transit=True):
        failed = self._record("start_coverage_cleaning", locals())
        assert failed is None
        return self._result({
            "accepted": True, "state": "SWEEPING",
            "active": True, "batch_active": True,
            "batch_id": self.batch_id,
        })

    def get_coverage_status(self):
        self.coverage_status_calls += 1
        return self._result(self.coverage)

    def cancel_coverage(self):
        self.cancel_calls += 1
        self.coverage.update(
            state="CANCELED", active=False, batch_active=False)
        return self._result(self.coverage)


class CoverageStartTombstonedBackend(MockBackend):
    """A lost start response was proven canceled before any commit."""

    def __init__(self):
        super().__init__(delay=0.0)
        self.batch_id = "coverage-batch-" + "c" * 32
        self.coverage_status_calls = 0
        self.cancel_calls = 0

    def start_coverage_cleaning(self, regions, operation_width_m=1.0,
                                overlap_percent=15.0, max_speed_mps=0.8,
                                allow_reverse_transit=True):
        failed = self._record("start_coverage_cleaning", locals())
        assert failed is None
        return ToolResult(json.dumps({
            "accepted": False,
            "batch_id": self.batch_id,
            "error": "start response lost",
            "cancel_state": "confirmed_not_started",
            "outcome_uncertain": False,
        }), True)

    def get_coverage_status(self):
        self.coverage_status_calls += 1
        return self._result({"state": "IDLE", "batch_id": ""})

    def cancel_coverage(self):
        self.cancel_calls += 1
        return self._result({"state": "CANCELED"})


class CoverageForeignStatusAfterLossBackend(MockBackend):
    """Exact cancel proof for A wins even while global status displays B."""

    def __init__(self):
        super().__init__(delay=0.0)
        self.batch_id = "coverage-batch-" + "a" * 32
        self.cancel_calls = 0

    def start_coverage_cleaning(self, regions, operation_width_m=1.0,
                                overlap_percent=15.0, max_speed_mps=0.8,
                                allow_reverse_transit=True):
        failed = self._record("start_coverage_cleaning", locals())
        assert failed is None
        return ToolResult(json.dumps({
            "accepted": False,
            "batch_id": self.batch_id,
            "error": "start response lost",
            "cancel_state": "unavailable",
            "outcome_uncertain": True,
        }), True)

    def get_coverage_status(self):
        return self._result({
            "state": "PLANNING",
            "active": False,
            "batch_active": True,
            "batch_id": "coverage-batch-" + "b" * 32,
            "ai_owned": False,
        })

    def cancel_coverage(self):
        self.cancel_calls += 1
        return self._result({
            "success": True,
            "batch_id": self.batch_id,
            "cancel_state": "confirmed_terminal",
            "safe": True,
        })


class CoverageForeignStatusAfterAcceptanceBackend(
        CoverageForeignStatusAfterLossBackend):
    """A successful A must not be replaced by global/latest batch B."""

    def start_coverage_cleaning(self, regions, operation_width_m=1.0,
                                overlap_percent=15.0, max_speed_mps=0.8,
                                allow_reverse_transit=True):
        failed = self._record("start_coverage_cleaning", locals())
        assert failed is None
        return self._result({
            "accepted": True,
            "state": "active",
            "batch_id": self.batch_id,
        })


def _step(tool, arguments, description=None):
    return {
        "tool": tool,
        "arguments": arguments,
        "description": description or tool,
    }


def _runner(document, backend=None, authorised=True, events=None):
    return AgentRunner(
        llm=FakeLLM(document), mcp=LocalMCP(backend),
        control_token="capability", max_plan_steps=8,
        nav_wait_timeout=1.0, coverage_wait_timeout=1.0,
        navigation_cleanup_timeout=0.2,
        visual_wait_timeout=1.0, poll_interval=0.002,
        event_callback=(events.append if events is not None else None),
        authorization_checker=lambda: authorised,
        cloud_authorization_checker=lambda: True,
    )


def test_preview_validates_but_does_not_mutate():
    backend = MockBackend(delay=0.005)
    runner = _runner({"steps": [
        _step("navigate_relative", {
            "forward_m": 2.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }),
    ]}, backend=backend)
    result = runner.run("向前两米", execute=False)
    assert result["state"] == "PREVIEW"
    assert result["results"] == []
    assert backend.calls == []


def test_continuous_sentence_executes_strictly_in_order():
    backend = MockBackend(delay=0.005)
    events = []
    runner = _runner({"steps": [
        _step("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }, "先向前一米"),
        _step("navigate_map_pose", {
            "x_m": 3.0, "y_m": 2.0, "yaw_deg": 90.0,
        }, "再到地图位置"),
    ]}, backend=backend, events=events)
    result = runner.run("先向前一米，再去地图坐标三二朝北", execute=True)
    assert result["state"] == "SUCCEEDED", result
    assert [item["tool"] for item in result["results"]] == [
        "navigate_relative", "navigate_map_pose"]
    assert [item[0] for item in backend.calls] == [
        "navigate_relative", "navigate_map_pose"]
    completed = [event["tool"] for event in events
                 if event["kind"] == "STEP" and
                 event.get("state") == "SUCCEEDED"]
    assert completed == ["navigate_relative", "navigate_map_pose"]


def test_accepted_navigation_polls_active_then_succeeded():
    backend = ScriptedNavigationBackend()
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "SUCCEEDED", result
    assert result["results"][0]["ok"] is True
    assert json.loads(result["results"][0]["detail"])["state"] == "succeeded"
    assert backend.navigation_status_calls == 2
    progress = [event["state"] for event in events
                if event["kind"] == "PROGRESS" and
                event.get("tool") == "navigate"]
    assert progress == ["active", "succeeded"]


def test_initial_navigation_error_fails_without_status_polling():
    backend = ScriptedNavigationBackend(initial_error=True)
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "FAILED", result
    assert result["results"][0]["ok"] is False
    assert backend.navigation_status_calls == 0
    assert not [event for event in events if event["kind"] == "PROGRESS"]


def test_post_publish_navigation_error_waits_for_exact_cancel_confirmation():
    backend = UncertainNavigationBackend()
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["cleanup_state"] == "confirmed"
    assert detail["confirmed_state"] == "recalled"
    assert backend.navigation_status_calls == 2
    assert backend.cancel_calls == 1
    progress = [event["state"] for event in events
                if event["kind"] == "PROGRESS"]
    assert progress == ["cleanup_cancel_uncertain", "cleanup_recalled"]


def test_navigation_cleanup_keeps_run_lock_through_same_goal_recalling():
    backend = RecallingThenRecalledNavigationBackend()
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend)
    outcome = {}

    def execute():
        try:
            outcome["result"] = runner.run(
                "导航到地图坐标八零", execute=True)
        except Exception as exc:  # pragma: no cover - surfaced below
            outcome["error"] = exc

    worker = threading.Thread(target=execute, daemon=True)
    worker.start()
    repeated_cancel = backend.two_recalling_cancels.wait(1.0)
    still_running = worker.is_alive()
    run_lock_held = runner._run_lock.locked()
    returned_early = "result" in outcome or "error" in outcome

    backend.allow_recalled.set()
    worker.join(1.0)

    assert repeated_cancel, "RECALLING must trigger repeated exact cancellation"
    assert still_running, "RECALLING must not let the AI task return"
    assert run_lock_held, "RECALLING must not release the Agent run lock"
    assert not returned_early
    assert not worker.is_alive(), "RECALLED should allow cleanup to finish"
    assert "error" not in outcome, outcome.get("error")
    result = outcome["result"]
    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["confirmed_state"] == "recalled"
    assert backend.navigation_status_calls == 3
    assert backend.cancel_calls == 2


def test_lost_navigation_enters_cleanup_instead_of_releasing_goal():
    backend = LostAfterAcceptanceBackend()
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["cleanup_state"] == "confirmed"
    assert detail["confirmed_state"] == "recalled"
    assert backend.cancel_calls == 1
    progress = [event["state"] for event in events
                if event["kind"] == "PROGRESS"]
    assert progress == [
        "lost", "cleanup_cancel_uncertain", "cleanup_recalled"]


def test_navigation_status_error_waits_for_exact_cancel_confirmation():
    backend = StatusErrorAfterAcceptanceBackend()
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["cleanup_state"] == "confirmed"
    assert detail["confirmed_state"] == "recalled"
    assert backend.cancel_calls == 1
    assert [event["state"] for event in events
            if event["kind"] == "PROGRESS"] == [
                "cleanup_cancel_uncertain", "cleanup_recalled"]


def test_operator_cancel_waits_for_same_goal_safe_state():
    backend = CancelDuringNavigationBackend()
    events = []
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend, events=events)
    backend.cancel_callback = runner.cancel

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "CANCELLED", result
    assert result["results"][0]["ok"] is False
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["confirmed_state"] == "recalled"
    assert backend.cancel_calls == 1


def test_foreign_navigation_status_cannot_complete_current_step():
    backend = MismatchedNavigationStatusBackend()
    runner = _runner({"steps": [
        _step("navigate_map_pose", {
            "x_m": 8.0, "y_m": 0.0, "yaw_deg": 0.0,
        }),
    ]}, backend=backend)

    result = runner.run("导航到地图坐标八零", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["goal_id"] == backend.goal_id
    assert detail["confirmed_state"] == "recalled"


def test_coverage_failed_state_keeps_ui_active_until_owner_is_released():
    backend = CoverageCleanupBackend()
    events = []
    runner = _runner({"steps": [
        _step("start_coverage_cleaning", {"regions": ["A区"]}),
    ]}, backend=backend, events=events)

    result = runner.run("清扫A区", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["batch_id"] == backend.batch_id
    assert detail["cleanup_state"] == "confirmed"
    assert detail["confirmed_state"] == "CANCELED"
    assert backend.cancel_calls == 1
    assert backend.coverage_status_calls >= 2
    progress = [event["state"] for event in events
                if event["kind"] == "PROGRESS"]
    assert progress[:2] == ["FAILED", "cleanup_FAILED"]
    assert progress[-1] == "cleanup_CANCELED"


def test_coverage_start_tombstone_is_authoritative_without_newest_batch_guess():
    backend = CoverageStartTombstonedBackend()
    runner = _runner({"steps": [
        _step("start_coverage_cleaning", {"regions": ["A区"]}),
    ]}, backend=backend)

    result = runner.run("清扫A区", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["batch_id"] == backend.batch_id
    assert detail["cancel_state"] == "confirmed_not_started"
    assert backend.coverage_status_calls == 0
    assert backend.cancel_calls == 0


def test_exact_coverage_cancel_proof_closes_old_id_despite_foreign_status():
    backend = CoverageForeignStatusAfterLossBackend()
    runner = _runner({"steps": [
        _step("start_coverage_cleaning", {"regions": ["A区"]}),
    ]}, backend=backend)

    result = runner.run("清扫A区", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["batch_id"] == backend.batch_id
    assert detail["confirmed_state"] == "confirmed_terminal"
    assert backend.cancel_calls == 1


def test_coverage_poll_preserves_a_when_global_status_switches_to_b():
    backend = CoverageForeignStatusAfterAcceptanceBackend()
    runner = _runner({"steps": [
        _step("start_coverage_cleaning", {"regions": ["A区"]}),
    ]}, backend=backend)

    result = runner.run("清扫A区", execute=True)

    assert result["state"] == "FAILED", result
    detail = json.loads(result["results"][0]["detail"])
    assert detail["batch_id"] == backend.batch_id
    assert detail["confirmed_state"] == "confirmed_terminal"
    assert backend.cancel_calls == 1


def test_failure_stops_remaining_steps():
    backend = MockBackend(delay=0.005, fail_tool="navigate_relative")
    runner = _runner({"steps": [
        _step("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }),
        _step("get_robot_status", {}),
    ]}, backend=backend)
    result = runner.run("先移动，再查状态", execute=True)
    assert result["state"] == "FAILED"
    assert len(result["results"]) == 1
    assert [item[0] for item in backend.calls] == ["navigate_relative"]


def test_revoked_control_fails_closed_before_tool_call():
    backend = MockBackend(delay=0.005)
    runner = _runner({"steps": [
        _step("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
        }),
    ]}, backend=backend, authorised=False)
    result = runner.run("向前", execute=True)
    assert result["state"] == "CANCELLED"
    assert backend.calls == []


def test_more_than_eight_steps_is_rejected_atomically():
    steps = [_step("get_robot_status", {}) for _ in range(9)]
    runner = _runner({"steps": steps})
    try:
        runner.run("九步任务", execute=True)
        assert False, "oversized plan must fail"
    except AgentError as exc:
        assert "超过上限" in str(exc)


def test_unknown_fields_and_gps_arguments_are_rejected():
    runner = _runner({"steps": [
        _step("navigate_relative", {
            "forward_m": 1.0, "left_m": 0.0, "delta_yaw_deg": 0.0,
            "latitude": 39.9,
        }),
    ]})
    try:
        runner.run("旧 GPS 参数", execute=True)
        assert False, "undefined arguments must fail"
    except AgentError as exc:
        assert "未定义参数" in str(exc)


def test_empty_plan_returns_cloud_clarification_without_tools():
    runner = _runner({"steps": [], "answer": "请说明距离和方向。"})
    result = runner.run("去那边", execute=True)
    assert result["state"] == "ANSWERED"
    assert result["answer"] == "请说明距离和方向。"
    assert result["results"] == []


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d agent tests passed" % len(tests))
