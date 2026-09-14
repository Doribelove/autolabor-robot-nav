"""Real visual receipt and ROS adapter methods; all I/O is in-memory."""
import copy
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
for source in ("src/sweeper_mcp/src", "src/application/autolabor_coverage/src",
               "src/application/autolabor_fod_control/src"):
    sys.path.insert(0, str(ROOT / source))
from sweeper_mcp.ros_backend import ROSBackend
from sweeper_mcp.visual_completion import visual_completed
from sweeper_mcp.agent import AgentRunner
from autolabor_fod_control.visual_session import VisualSessionReceipt


class World:
    def __init__(self):
        self.receipt = VisualSessionReceipt()
        self.mode = "GPS_ACTIVE"
        self.state = "DISABLED"
        self.protocol = 1
        self.active = False
        self.calls = []
        self.lost_reply = False
        self.no_start = False
        self.missing = False
        self.mode_reason = "visual controller aborted; GPS remains paused"
        self.visual_reason = ""
        self.service_message = "accepted"
        self.backend = ROSBackend()
        self.backend._ready = True
        self.backend._visual_start_timeout = .01
        self.backend._coverage_busy = lambda: False
        self.backend._snapshot = self.snapshot
        self.backend._rospy = SimpleNamespace(
            is_shutdown=lambda: False,
            wait_for_service=lambda *a, **kw: None,
            ServiceProxy=self.proxy)

    def snapshot(self, key, _age):
        if self.missing:
            return None
        data = (dict(state=self.mode, visual_state=self.state, reason=self.mode_reason) if key == "mode" else
                dict(state=self.state, active=self.active, reason=self.visual_reason, ai_completion_protocol=self.protocol,
                     visual_session=self.receipt.status()))
        return SimpleNamespace(data=json.dumps(data))

    def proxy(self, service, _type):
        assert service == "/fod_navigation_mode/set_fod_enabled"
        return self.enable

    def enable(self, enabled):
        self.calls.append(enabled)
        if enabled and not self.no_start:
            self.receipt.begin()
            self.mode, self.state, self.active = "FOD_ACTIVE", "ACQUIRE", True
        if not enabled:
            self.receipt.disable()
            self.mode, self.state, self.active = "GPS_ACTIVE", "DISABLED", False
        if enabled and self.lost_reply:
            raise RuntimeError("injected service reply loss")
        return SimpleNamespace(success=True, message=self.service_message)

    def complete(self):
        self.receipt.complete()
        self.receipt.disable()
        self.mode, self.state, self.active = "GPS_ACTIVE", "DISABLED", False


def test_visual_normal_receipt_is_frozen_across_disable_and_next_session():
    r = VisualSessionReceipt()
    r.complete()
    assert r.sequence == 0
    r.begin()
    first = r.session_id
    r.complete()
    r.complete()
    assert r.sequence == 1
    r.disable()
    assert r.result == "COMPLETE" and r.last_completion["session_id"] == first
    r.begin()
    assert r.session_id != first and r.sequence == 1
    assert r.last_completion["session_id"] == first


def test_visual_abort_or_disable_never_creates_completion():
    for action in ("abort", "disable"):
        r = VisualSessionReceipt()
        r.begin()
        getattr(r, action)()
        r.complete()
        r.disable()
        assert r.sequence == 0 and not r.last_completion
        assert r.result in ("ABORT", "CANCELLED")


def test_ros_adapter_requires_deployed_receipt_protocol_before_enable():
    w = World()
    w.protocol = None
    assert w.backend.start_spot_cleaning().is_error
    assert w.calls == []


def test_ros_adapter_success_requires_current_session_new_receipt_and_idle():
    w = World()
    initial = json.loads(w.backend.start_spot_cleaning().text)
    assert not visual_completed(json.loads(w.backend.get_visual_servo_status().text),
                                initial["visual_session_id"], initial["completion_baseline"])
    assert w.backend._ai_visual_owned
    w.complete()
    data = json.loads(w.backend.get_visual_servo_status().text)
    assert visual_completed(data, initial["visual_session_id"], initial["completion_baseline"])
    assert not w.backend._ai_visual_owned


def test_visual_receipt_rejects_wrong_identity_stale_counter_incomplete_or_moving():
    w = World()
    initial = json.loads(w.backend.start_spot_cleaning().text)
    w.complete()
    data = json.loads(w.backend.get_visual_servo_status().text)
    for key, value in [("visual_session_id", "f"*32), ("visual_result", "ABORT"),
                       ("visual_active", True), ("visual_active", None), ("visual_state", "ABORT"),
                       ("state", "FOD_ACTIVE"), ("ai_completion_protocol", None),
                       ("ai_completion_protocol", True), ("ai_completion_protocol", 1.0)]:
        bad = dict(data, **{key: value})
        assert not visual_completed(bad, initial["visual_session_id"], 0)
    for key, value in [("sequence", 0), ("sequence", True), ("sequence", 1.5),
                       ("session_id", "f"*32), ("result", "ABORT"), ("stopped", False)]:
        bad = copy.deepcopy(data)
        bad["completion"][key] = value
        assert not visual_completed(bad, initial["visual_session_id"], 0)


def test_ros_adapter_old_success_cannot_complete_new_visual_request():
    w = World()
    w.backend.start_spot_cleaning()
    w.complete()
    w.backend.get_visual_servo_status()
    second = json.loads(w.backend.start_spot_cleaning().text)
    w.enable(False)
    data = json.loads(w.backend.get_visual_servo_status().text)
    assert not visual_completed(data, second["visual_session_id"], second["completion_baseline"])


def test_ros_adapter_abort_then_operator_reset_is_not_success():
    w = World()
    initial = json.loads(w.backend.start_spot_cleaning().text)
    w.receipt.abort()
    w.enable(False)
    data = json.loads(w.backend.get_visual_servo_status().text)
    assert data["state"] == "RELATIVE_NAV_ACTIVE" and data["visual_result"] == "ABORT"
    assert not visual_completed(data, initial["visual_session_id"], initial["completion_baseline"])


def test_ros_adapter_never_stops_foreign_or_missing_visual_session():
    for missing in (False, True):
        w = World()
        w.backend.start_spot_cleaning()
        if missing: w.missing = True
        else: w.receipt.begin()  # another controller/session after a reset
        assert w.backend.stop_spot_cleaning().is_error
        assert w.calls == [True]


def test_lost_start_reply_retains_known_session_for_cleanup():
    w = World()
    w.lost_reply = True
    initial = w.backend.start_spot_cleaning()
    assert initial.is_error and json.loads(initial.text)["visual_session_id"] == w.receipt.session_id
    assert w.backend._ai_visual_owned
    assert not w.backend.stop_spot_cleaning().is_error
    assert not w.backend._ai_visual_owned
    assert w.calls == [True, False]


def test_unknown_start_outcome_blocks_other_ai_motion_without_stopping_foreign_session():
    w = World()
    w.no_start = w.lost_reply = True
    assert w.backend.start_spot_cleaning().is_error
    assert w.backend._ai_visual_owned
    assert w.backend._publish_owned_goal("map", 0, 0, 0).is_error
    assert w.backend.start_coverage_cleaning(["A区"]).is_error
    assert w.backend.stop_spot_cleaning().is_error
    assert w.calls == [True]


def test_no_target_confirmed_noop_is_failure_but_does_not_retain_visual_owner():
    w = World()
    w.no_start = True
    assert w.backend.start_spot_cleaning().is_error
    assert not w.backend._ai_visual_owned
    assert w.calls == [True]


def test_service_unavailable_before_submission_does_not_create_an_unknown_owner():
    w = World()
    def unavailable(*args, **kwargs):
        raise RuntimeError("service not available")
    w.backend._rospy.wait_for_service = unavailable
    assert w.backend.start_spot_cleaning().is_error
    assert not w.backend._ai_visual_owned and w.calls == []


def test_ros_status_preserves_controller_reason_and_mode_reason_separately():
    w = World()
    w.visual_reason = "detections source stamp age is invalid: 0.518s"
    data = json.loads(w.backend.get_visual_servo_status().text)
    assert data["reason"] == data["visual_reason"] == w.visual_reason
    assert data["mode_reason"] == w.mode_reason
    w.visual_reason = ""
    assert json.loads(w.backend.get_visual_servo_status().text)["reason"] == w.mode_reason


def test_no_target_and_far_target_keep_service_reason_and_not_started_evidence():
    for message in (
            "no valid-depth FOD was observed for 1.000s; GPS remains active",
            "nearest FOD 5.500m is not within 5.000m; GPS remains active"):
        w = World()
        w.no_start = True
        w.service_message = message
        result = w.backend.start_spot_cleaning()
        data = json.loads(result.text)
        assert result.is_error and data["execution_state"] == "not_started"
        assert data["service_message"] == message and message in data["message"]
        assert not data.get("visual_session_id") and not w.backend._ai_visual_owned


def test_unknown_enable_outcome_is_not_misreported_as_not_started():
    w = World()
    w.no_start = w.lost_reply = True
    result = w.backend.start_spot_cleaning()
    assert json.loads(result.text)["execution_state"] == "start_unconfirmed"
    assert result.is_error and w.backend._ai_visual_owned


class WorldMCP:
    def __init__(self, world):
        self.world = world

    def initialize(self):
        return {}

    def list_tools(self):
        return {"tools": []}

    def call_tool(self, name, arguments=None, **kwargs):
        result = getattr(self.world.backend, name)()
        return dict(text=result.text, is_error=result.is_error)


class MisleadingSummaryLLM:
    def __init__(self):
        self.calls = 0

    def chat(self, *args, **kwargs):
        self.calls += 1
        return {"content": "未执行清扫，车辆没有动作。"}


def test_abort_reason_and_started_evidence_survive_cleanup_and_summary():
    for reason in (
            "detections source stamp age is invalid: 0.518s",
            "FOD is still visible below the motion-confidence threshold; blind advance is inhibited"):
        w = World()
        initial = w.backend.start_spot_cleaning().text
        w.receipt.abort()
        w.mode, w.state, w.active = "FOD_ABORTED", "ABORT", False
        w.visual_reason = reason
        llm = MisleadingSummaryLLM()
        runner = AgentRunner(mcp=WorldMCP(w), llm=llm)
        ok, detail = runner._poll_visual(initial)
        data = json.loads(detail)
        assert not ok and data["execution_state"] == "started_not_completed"
        assert data["visual_status"]["visual_reason"] == reason
        assert reason in data["message"]
        assert data["cleanup_state"] == "exit_requested"
        assert w.calls == [True, False] and not w.backend._ai_visual_owned
        answer = runner._summarise("视觉清扫后回原点", {"steps": [{}, {}]}, [
            dict(tool="start_spot_cleaning", ok=False, detail=detail)], "FAILED")
        assert "已启动，但未完成" in answer and reason in answer
        assert "后续 1 步未执行" in answer and "未执行清扫" not in answer
        assert llm.calls == 0


def test_summary_distinguishes_not_started_from_unknown_without_claiming_no_litter():
    for lost, expected in ((False, "视觉清扫未启动"), (True, "视觉启动结果未确认")):
        w = World()
        w.no_start, w.lost_reply = True, lost
        w.service_message = "no valid-depth FOD was observed for 1.000s; GPS remains active"
        detail = w.backend.start_spot_cleaning().text
        llm = MisleadingSummaryLLM()
        runner = AgentRunner(mcp=WorldMCP(w), llm=llm)
        answer = runner._summarise("视觉清扫", {"steps": [{}]}, [
            dict(tool="start_spot_cleaning", ok=False, detail=detail)], "FAILED")
        assert expected in answer and "没有垃圾" not in answer and llm.calls == 0


def test_cleanup_failure_preserves_original_abort_and_does_not_claim_stopped():
    w = World()
    initial = w.backend.start_spot_cleaning().text
    w.receipt.abort()
    w.mode, w.state, w.active = "FOD_ABORTED", "ABORT", False
    w.visual_reason = "original controller abort"
    w.backend.stop_spot_cleaning = lambda: SimpleNamespace(text="injected exit failure", is_error=True)
    runner = AgentRunner(mcp=WorldMCP(w), llm=MisleadingSummaryLLM())
    ok, detail = runner._poll_visual(initial)
    data = json.loads(detail)
    assert not ok and data["cleanup_state"] == "exit_unconfirmed"
    assert data["visual_status"]["visual_reason"] == "original controller abort"
    assert "injected exit failure" in data["cleanup_detail"] and w.backend._ai_visual_owned
    answer = runner._summarise("视觉清扫", {"steps": [{}]}, [
        dict(tool="start_spot_cleaning", ok=False, detail=detail)], "FAILED")
    assert "退出未确认" in answer and "已经停车" not in answer


def test_long_visual_error_stays_valid_json_and_keeps_execution_evidence():
    w = World()
    initial = w.backend.start_spot_cleaning().text
    w.receipt.abort()
    w.mode, w.state, w.active = "FOD_ABORTED", "ABORT", False
    w.visual_reason = "sensor failure " * 2000
    runner = AgentRunner(mcp=WorldMCP(w), llm=MisleadingSummaryLLM())
    ok, detail = runner._poll_visual(initial)
    assert not ok and len(detail) < 3000
    assert json.loads(detail)["execution_state"] == "started_not_completed"


def test_lost_reply_with_recovered_session_is_reported_as_started_not_completed():
    w = World()
    w.lost_reply = True
    initial = w.backend.start_spot_cleaning()
    assert initial.is_error
    runner = AgentRunner(mcp=WorldMCP(w), llm=MisleadingSummaryLLM())
    ok, detail = runner._visual_failure(initial.text)
    assert not ok and json.loads(detail)["execution_state"] == "started_not_completed"
    assert w.calls == [True, False]


def test_unknown_or_foreign_status_does_not_trigger_a_foreign_stop():
    for missing in (True, False):
        w = World()
        initial = w.backend.start_spot_cleaning().text
        if missing:
            w.missing = True
        else:
            w.receipt.begin()
        runner = AgentRunner(mcp=WorldMCP(w), llm=MisleadingSummaryLLM())
        ok, detail = runner._poll_visual(initial)
        assert not ok and json.loads(detail)["cleanup_state"] == "exit_unconfirmed"
        assert w.calls == [True] and w.backend._ai_visual_owned


def test_mode_fault_reason_is_not_hidden_by_normal_visual_tracking_reason():
    w = World()
    initial = w.backend.start_spot_cleaning().text
    w.mode = "FAULT_STOP"
    w.mode_reason = "mode arbiter safety fault"
    w.visual_reason = "normal visual tracking"
    runner = AgentRunner(mcp=WorldMCP(w), llm=MisleadingSummaryLLM())
    ok, detail = runner._poll_visual(initial)
    assert not ok and w.mode_reason in json.loads(detail)["message"]
    assert w.visual_reason not in json.loads(detail)["message"]


def poll_receipts(changes):
    """Real agent polling; only its transport/clock-duration fixture is synthetic."""
    w = World()
    initial = json.loads(w.backend.start_spot_cleaning().text)
    w.complete()
    complete = json.loads(w.backend.get_visual_servo_status().text)
    runner = AgentRunner.__new__(AgentRunner)
    runner.visual_wait_timeout = 0.03
    runner.poll_interval = 0.001
    runner._cancel = threading.Event()
    runner._emit = lambda *a, **kw: None
    runner._visual_failure = lambda reason, *args, **kwargs: (False, reason)
    calls = []
    def query(*args, **kwargs):
        data = copy.deepcopy(complete)
        modify = changes[min(len(calls), len(changes)-1)]
        modify(data)
        calls.append(data)
        return dict(text=json.dumps(data), is_error=False)
    runner.mcp = SimpleNamespace(call_tool=query)
    return runner._poll_visual(json.dumps(initial)), calls


def test_agent_waits_for_stop_receipt_after_mode_returns_to_navigation():
    result, calls = poll_receipts([
        lambda data: data["completion"].update(stopped=False),
        lambda data: None])
    assert result[0] and len(calls) == 2


def test_agent_never_accepts_complete_without_physical_stop_receipt():
    result, calls = poll_receipts([
        lambda data: data["completion"].update(stopped=False)])
    assert not result[0] and len(calls) > 1
    assert "超时" in result[1]


def test_agent_never_accepts_an_old_completion_sequence():
    result, calls = poll_receipts([
        lambda data: data["completion"].update(sequence=0)])
    assert not result[0] and len(calls) > 1


def test_agent_abort_still_fails_immediately_after_navigation_handoff():
    result, calls = poll_receipts([
        lambda data: data.update(visual_result="ABORT")])
    assert not result[0] and len(calls) == 1


def test_agent_exit_without_complete_still_fails_immediately():
    result, calls = poll_receipts([
        lambda data: data.update(visual_result="RUNNING")])
    assert not result[0] and len(calls) == 1
