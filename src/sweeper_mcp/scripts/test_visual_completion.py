"""Real visual receipt and ROS adapter methods; all I/O is in-memory."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
for source in ("src/sweeper_mcp/src", "src/application/autolabor_coverage/src",
               "src/application/autolabor_fod_control/src"):
    sys.path.insert(0, str(ROOT / source))
from sweeper_mcp.ros_backend import ROSBackend
from sweeper_mcp.visual_completion import visual_completed
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
        data = (dict(state=self.mode, visual_state=self.state) if key == "mode" else
                dict(state=self.state, active=self.active, ai_completion_protocol=self.protocol,
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
        return SimpleNamespace(success=True, message="accepted")

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
