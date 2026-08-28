#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline authorization/epoch tests for the Qt-to-ASR boundary."""

import importlib.util
import os
import threading
import time
from collections import deque
from types import SimpleNamespace


_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE_PATH = os.path.join(_HERE, "sweeper_ai_node.py")
_SPEC = importlib.util.spec_from_file_location("sweeper_ai_node_tested", _NODE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
SweeperAiNode = _MODULE.SweeperAiNode


class FakeAsrClient:
    def __init__(self, result=None, method_results=None):
        self.result = dict(result or {})
        self.method_results = dict(method_results or {})
        self.calls = []
        self.closed = False

    def request(self, method, params=None, timeout=None):
        params = dict(params or {})
        self.calls.append((method, params, timeout))
        result = self.method_results.get(method, self.result)
        if callable(result):
            result = result(params)
        return dict(result or {})

    def close(self):
        self.closed = True


class FakeRunner:
    def __init__(self, result=None):
        self.cancel_count = 0
        self.result = result or {
            "state": "SUCCEEDED",
            "total_ms": 1.0,
            "answer": "done",
            "plan": {"steps": []},
            "results": [],
        }

    def cancel(self):
        self.cancel_count += 1

    def run(self, _text, execute=False):
        del execute
        return dict(self.result)


def _bare_node():
    node = SweeperAiNode.__new__(SweeperAiNode)
    node._lock = threading.RLock()
    node._session_token = "session-token"
    node._last_heartbeat = time.monotonic()
    node._voice_authorized = False
    node._parse_authorized = False
    node._control_authorized = False
    node._task_active = False
    node._request_id = ""
    node._phase = "IDLE"
    node._current_step = 0
    node._total_steps = 0
    node._last_error = ""
    node._final_text = ""
    node._last_cloud_rtt_ms = 0.0
    node._last_total_latency_ms = 0.0
    node._last_http_status = 0
    node._cloud_configured = True
    node._runner = FakeRunner()
    node._asr_epoch = 0
    node._asr_worker_generation = 1
    node._asr_switching = False
    node._asr_capture_id = ""
    node._asr_client = None
    node._asr_available = True
    node._asr_enabled = True
    node._asr_model = "medium"
    node._asr_device = "cuda"
    node._asr_model_dir = "/offline/models"
    node._asr_model_path = "/offline/models/medium.pt"
    node._asr_common_config = {"device": "cuda"}
    node._asr_config = node._asr_config_for_model("medium")
    node._asr_model_loaded = False
    node._asr_recording = False
    node._asr_phase = "READY"
    node._asr_audio_duration_s = 0.0
    node._asr_latency_ms = 0.0
    node._asr_last_error = ""
    node._smart_voice_enabled = False
    node._smart_voice_listening = False
    node._smart_voice_session_id = ""
    node._smart_voice_epoch = 0
    node._smart_voice_utterance_count = 0
    node._smart_voice_pending = deque()
    node._smart_voice_seen = set()
    node.events = []
    node._emit = lambda kind, **fields: node.events.append((kind, fields))
    node._publish_status = lambda: None
    return node


def _smart_event(session_id, index, transcript="清扫 A 区"):
    return {
        "event": "smart_transcript",
        "result": {
            "session_id": session_id,
            "utterance_id": "utterance-%d" % index,
            "utterance_index": index,
            "transcript": transcript,
            "audio_duration_s": 1.0,
            "rms": 0.1,
            "asr_latency_ms": 25.0,
            "accepted": True,
            "model": "medium",
            "device": "cuda",
            "error": "",
        },
    }


def test_manual_input_does_not_require_voice_authorization():
    node = _bare_node()
    node._parse_authorized = True
    finished = threading.Event()
    observed = {}

    def run_request(request_id, text, execute):
        observed.update(request_id=request_id, text=text, execute=execute)
        finished.set()

    node._run_request = run_request
    accepted, request_id, _message = node._begin_request(
        "只解析这一条手工指令", "MANUAL")
    assert accepted and request_id
    assert finished.wait(1.0)
    assert observed["text"] == "只解析这一条手工指令"
    assert observed["execute"] is False


def test_asr_requires_both_voice_and_parse_before_cloud_submission():
    node = _bare_node()
    accepted, _request_id, message = node._begin_request("向前一米", "ASR")
    assert not accepted and "解析" in message
    node._parse_authorized = True
    accepted, _request_id, message = node._begin_request("向前一米", "ASR")
    assert not accepted and "语音" in message


def test_all_cloud_input_requires_a_fresh_qt_heartbeat():
    node = _bare_node()
    node._parse_authorized = True
    node._last_heartbeat = time.monotonic() - 10.0
    accepted, _request_id, message = node._begin_request("状态", "MANUAL")
    assert not accepted and "心跳" in message


def test_recording_needs_voice_but_not_parse_authorization():
    node = _bare_node()
    node._asr_client = FakeAsrClient({
        "ready": True,
        "recording": True,
        "state": "recording",
        "model": "medium",
        "device": "cuda",
    })
    rejected = node._start_asr_recording()
    assert not rejected.accepted and "语音" in rejected.message
    node._voice_authorized = True
    accepted = node._start_asr_recording()
    assert accepted.accepted
    assert accepted.capture_id
    assert node._asr_recording
    assert not node._parse_authorized


def test_start_rejects_previous_capture_until_stop_result_is_consumed():
    node = _bare_node()
    node._voice_authorized = True
    node._asr_client = FakeAsrClient({
        "ready": True,
        "recording": True,
        "state": "recording",
    })
    node._asr_capture_id = "previous-capture"
    node._asr_phase = "READY"
    rejected = node._start_asr_recording()
    assert not rejected.accepted and "忙" in rejected.message
    assert node._asr_client.calls == []


def test_duplicate_stop_is_rejected_while_transcription_is_pending():
    node = _bare_node()
    node._voice_authorized = True
    node._asr_client = FakeAsrClient()
    node._asr_capture_id = "capture-stop"
    node._asr_recording = True
    node._asr_phase = "STOPPING"
    rejected = node._stop_asr_recording()
    assert not rejected.accepted and "没有录音" in rejected.message
    assert node._asr_client.calls == []


def test_revoke_treats_idle_loaded_model_as_active_for_unload():
    node = _bare_node()
    node._asr_model_loaded = True
    node._asr_phase = "READY"
    with node._lock:
        epoch, active = node._revoke_asr_locked("operator revoke")
    assert epoch == 1 and active
    assert node._asr_phase == "CANCELLING"


def test_local_transcript_is_displayed_without_cloud_parse_authorization():
    node = _bare_node()
    node._voice_authorized = True
    node._asr_epoch = 7
    node._asr_capture_id = "capture-7"
    node._asr_recording = True
    client = FakeAsrClient({
        "capture_id": "capture-7",
        "transcript": "清扫 A 区",
        "ready": True,
        "recording": False,
        "state": "ready",
        "model_loaded": True,
        "audio_duration_s": 1.5,
        "asr_latency_ms": 25.0,
        "rms": 0.1,
    })
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(
        (args, kwargs)) or (True, "unexpected", "")
    node._finish_asr_recording(client, 7, "capture-7")
    transcripts = [fields["description"] for kind, fields in node.events
                   if kind == "TRANSCRIPT"]
    assert transcripts == ["清扫 A 区"]
    assert cloud_calls == []
    assert any(kind == "ASR" and fields.get("state") == "LOCAL_ONLY"
               for kind, fields in node.events)


def test_revoked_epoch_discards_late_transcript_and_never_calls_cloud():
    node = _bare_node()
    node._voice_authorized = True
    node._parse_authorized = True
    node._asr_epoch = 11
    node._asr_capture_id = "capture-11"
    node._asr_recording = True
    with node._lock:
        revoked_epoch, active = node._revoke_asr_locked("operator revoke")
    assert active and revoked_epoch == 12
    client = FakeAsrClient({
        "capture_id": "capture-11",
        "transcript": "这个结果必须丢弃",
    })
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(args)
    node._finish_asr_recording(client, 11, "capture-11")
    assert node.events == []
    assert cloud_calls == []


def test_smart_voice_defaults_off_and_requires_all_start_gates():
    node = _bare_node()
    request = SimpleNamespace(session_token="wrong", enabled=True)
    assert not node._set_smart_voice(request).accepted

    request.session_token = node._session_token
    assert not node._set_smart_voice(request).accepted
    node._voice_authorized = True
    node._last_heartbeat = time.monotonic() - 10.0
    assert not node._set_smart_voice(request).accepted
    node._last_heartbeat = time.monotonic()
    node._asr_available = False
    assert not node._set_smart_voice(request).accepted

    node._asr_available = True
    node._asr_client = FakeAsrClient(method_results={
        "start_smart_listening": lambda params: {
            "ready": True,
            "state": "SMART_LISTENING",
            "smart_listening": True,
            "smart_session_id": params["session_id"],
        },
        "stop_smart_listening": lambda params: {
            "ready": True,
            "state": "SMART_STOPPED",
            "smart_listening": False,
            "smart_session_id": "",
            "session_id": params["session_id"],
        },
    })
    started = node._set_smart_voice(request)
    assert started.accepted and started.session_id
    assert node._smart_voice_enabled and node._smart_voice_listening

    request.enabled = False
    stopped = node._set_smart_voice(request)
    assert stopped.accepted and stopped.session_id == started.session_id
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert [call[0] for call in node._asr_client.calls] == [
        "start_smart_listening", "stop_smart_listening"]


def test_smart_voice_and_manual_recording_are_mutually_exclusive():
    node = _bare_node()
    node._voice_authorized = True
    node._asr_client = FakeAsrClient({"ready": True, "recording": True})
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    rejected = node._start_asr_recording()
    assert not rejected.accepted and "智能语音" in rejected.message
    assert node._asr_client.calls == []

    node = _bare_node()
    node._voice_authorized = True
    node._asr_client = FakeAsrClient()
    node._asr_capture_id = "manual-capture"
    node._asr_recording = True
    rejected = node._start_smart_voice()
    assert not rejected.accepted and "手动录音" in rejected.message
    assert node._asr_client.calls == []


def test_smart_transcript_is_local_only_when_parse_gate_is_off():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "smart-local"
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(
        (args, kwargs)) or (True, "unexpected", "")
    node._handle_smart_transcript(_smart_event("smart-local", 1))
    transcripts = [fields["description"] for kind, fields in node.events
                   if kind == "TRANSCRIPT"]
    assert transcripts == ["清扫 A 区"]
    assert cloud_calls == []
    assert list(node._smart_voice_pending) == []
    assert any(kind == "SMART_VOICE" and fields.get("state") == "LOCAL_ONLY"
               for kind, fields in node.events)


def test_worker_queue_drop_is_shown_only_for_current_authorized_session():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "queue-session"
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(args)

    node._asr_event({
        "event": "smart_queue_drop",
        "result": {
            "session_id": "old-session",
            "utterance_id": "old:1",
            "utterance_index": 1,
            "reason": "full",
        },
    })
    assert node.events == []
    node._asr_event({
        "event": "smart_queue_drop",
        "result": {
            "session_id": "queue-session",
            "utterance_id": "queue-session:2",
            "utterance_index": 2,
            "reason": "full",
        },
    })
    assert any(kind == "SMART_VOICE" and fields.get("state") == "QUEUE_FULL"
               for kind, fields in node.events)
    assert cloud_calls == []


def test_smart_session_and_epoch_discard_late_results():
    node = _bare_node()
    node._voice_authorized = True
    node._parse_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "current-session"
    node._smart_voice_epoch = 4
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(args)

    node._handle_smart_transcript(_smart_event("old-session", 1))
    assert node.events == [] and cloud_calls == []
    with node._lock:
        node._revoke_smart_voice_locked("operator stop")
    node._handle_smart_transcript(_smart_event("current-session", 2))
    assert node.events == [] and cloud_calls == []


def test_current_worker_terminal_event_fails_closed_and_old_event_is_ignored():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "terminal-session"
    node._smart_voice_epoch = 9
    node._smart_voice_pending.append({"transcript": "绝不能迟到执行"})

    node._asr_event({
        "event": "smart_error",
        "result": {
            "session_id": "different-session",
            "state": "ERROR",
            "error": "old failure",
            "smart_listening": False,
        },
    })
    assert node._smart_voice_enabled and node._smart_voice_listening

    node._asr_event({
        "event": "smart_error",
        "result": {
            "session_id": "terminal-session",
            "state": "ERROR",
            "error": "microphone disconnected",
            "smart_listening": False,
        },
    })
    assert node._smart_voice_epoch == 10
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert node._smart_voice_session_id == ""
    assert list(node._smart_voice_pending) == []
    assert node._asr_last_error == "microphone disconnected"
    assert any(kind == "SMART_VOICE" and fields.get("state") == "ERROR"
               for kind, fields in node.events)


def test_worker_eof_status_fails_closed_without_terminal_session_id():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "worker-eof-session"
    node._smart_voice_epoch = 12
    node._smart_voice_pending.append({"transcript": "不能在 worker 死后执行"})
    node._asr_event({
        "event": "status",
        "status": {
            "ready": False,
            "state": "UNAVAILABLE",
            "smart_listening": False,
            "smart_session_id": "",
            "error": "ASR worker is not running",
        },
    })
    assert node._smart_voice_epoch == 13
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert node._smart_voice_session_id == ""
    assert list(node._smart_voice_pending) == []
    assert not node._asr_available
    assert any(kind == "SMART_VOICE" and fields.get("state") == "ERROR"
               for kind, fields in node.events)


def test_smart_queue_is_bounded_fifo_and_cloud_requests_are_serial():
    node = _bare_node()
    node._voice_authorized = True
    node._parse_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "serial-session"
    node._smart_voice_epoch = 2
    node._task_active = True

    for index in range(1, 11):
        node._handle_smart_transcript(_smart_event(
            "serial-session", index, "指令%d" % index))
    assert len(node._smart_voice_pending) == _MODULE.SMART_VOICE_QUEUE_LIMIT
    assert sum(1 for kind, fields in node.events
               if kind == "SMART_VOICE" and
               fields.get("state") == "QUEUE_FULL") == 2

    dispatched = []

    def begin_request(text, source, emit_transcript=True):
        assert not node._task_active
        node._task_active = True
        dispatched.append((text, source, emit_transcript))
        return True, "request-%d" % len(dispatched), "请求已接收"

    node._begin_request = begin_request
    node._task_active = False
    assert node._dispatch_next_smart_voice()
    assert dispatched == [("指令1", "ASR", False)]
    assert not node._dispatch_next_smart_voice()
    assert dispatched == [("指令1", "ASR", False)]
    node._task_active = False
    assert node._dispatch_next_smart_voice()
    assert dispatched[-1] == ("指令2", "ASR", False)


def test_smart_queue_drops_expired_commands_before_dispatch():
    node = _bare_node()
    node._voice_authorized = True
    node._parse_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "ttl-session"
    node._smart_voice_epoch = 3
    node._smart_voice_pending.append({
        "epoch": 3,
        "session_id": "ttl-session",
        "utterance_id": "expired",
        "utterance_index": 1,
        "transcript": "执行已经陈旧的导航",
        "enqueued_at": time.monotonic() -
                       _MODULE.SMART_VOICE_QUEUE_TTL_S - 1.0,
    })
    cloud_calls = []
    node._begin_request = lambda *args, **kwargs: cloud_calls.append(args)
    assert not node._dispatch_next_smart_voice()
    assert cloud_calls == []
    assert any(kind == "SMART_VOICE" and fields.get("state") == "DROPPED"
               for kind, fields in node.events)


def test_parse_revoke_clears_unsubmitted_smart_queue():
    node = _bare_node()
    node._parse_authorized = True
    node._control_authorized = True
    node._smart_voice_pending.append({"transcript": "待处理语音"})
    response = node._set_authorization(SimpleNamespace(
        session_token=node._session_token, gate="parse", enabled=False))
    assert response.accepted
    assert not node._parse_authorized and not node._control_authorized
    assert list(node._smart_voice_pending) == []
    assert any(kind == "SMART_VOICE" and fields.get("state") == "DROPPED"
               for kind, fields in node.events)


def test_control_gate_change_clears_old_queue_without_upgrading_active_preview():
    node = _bare_node()
    node._parse_authorized = True
    node._task_active = True
    node._smart_voice_pending.append({"transcript": "旧的预览口令"})
    response = node._set_authorization(SimpleNamespace(
        session_token=node._session_token, gate="control", enabled=True))
    assert response.accepted and node._control_authorized
    assert node._task_active
    assert node._runner.cancel_count == 0
    assert list(node._smart_voice_pending) == []
    assert any(kind == "SMART_VOICE" and fields.get("state") == "DROPPED"
               for kind, fields in node.events)


def test_voice_revoke_stops_current_smart_session_and_clears_queue():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "revoke-session"
    node._smart_voice_pending.append({"transcript": "撤权后不能执行"})
    node._asr_client = FakeAsrClient(method_results={
        "stop_smart_listening": lambda params: {
            "ready": True,
            "state": "SMART_STOPPED",
            "smart_listening": False,
            "smart_session_id": "",
            "session_id": params["session_id"],
        },
    })
    response = node._set_authorization(SimpleNamespace(
        session_token=node._session_token, gate="voice", enabled=False))
    assert response.accepted
    assert not node._voice_authorized
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert list(node._smart_voice_pending) == []
    assert node._asr_client.calls[0][0] == "stop_smart_listening"


def test_heartbeat_expiry_invalidates_smart_session_before_async_stop():
    node = _bare_node()
    node._voice_authorized = True
    node._parse_authorized = True
    node._control_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "stale-heartbeat"
    node._smart_voice_pending.append({"transcript": "心跳丢失后不能执行"})
    node._last_heartbeat = time.monotonic() - 10.0
    stop_requests = []
    node._stop_smart_worker_async = lambda request, reason: stop_requests.append(
        (request, reason))
    node._watchdog(None)
    assert not node._voice_authorized
    assert not node._parse_authorized
    assert not node._control_authorized
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert list(node._smart_voice_pending) == []
    assert len(stop_requests) == 1
    assert stop_requests[0][0][1] == "stale-heartbeat"


def test_task_completion_automatically_checks_next_smart_utterance():
    node = _bare_node()
    node._request_id = "active-request"
    node._task_active = True
    dispatch_count = []
    node._dispatch_next_smart_voice = lambda: dispatch_count.append(1)
    node._run_request("active-request", "第一条", False)
    assert not node._task_active
    assert dispatch_count == [1]


def test_cancel_closes_smart_microphone_and_clears_queue():
    node = _bare_node()
    node._voice_authorized = True
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    node._smart_voice_session_id = "cancel-session"
    node._smart_voice_pending.append({"transcript": "不能在取消后执行"})
    node._asr_client = FakeAsrClient(method_results={
        "stop_smart_listening": lambda params: {
            "ready": True,
            "state": "SMART_STOPPED",
            "smart_listening": False,
            "smart_session_id": "",
            "session_id": params["session_id"],
        },
    })
    response = node._cancel_task(SimpleNamespace(
        session_token=node._session_token, request_id=""))
    assert response.accepted
    assert not node._smart_voice_enabled
    assert not node._smart_voice_listening
    assert list(node._smart_voice_pending) == []
    assert node._asr_client.calls[0][0] == "stop_smart_listening"


def test_asr_model_switch_replaces_idle_worker_and_closes_old_worker():
    node = _bare_node()
    old_client = FakeAsrClient()
    node._asr_client = old_client
    candidate = FakeAsrClient()
    candidate.start = lambda: {
        "ready": True,
        "state": "IDLE",
        "model": "small",
        "device": "cuda",
        "model_loaded": False,
        "recording": False,
        "error": "",
    }
    node._new_asr_client = lambda config, generation: candidate
    response = node._set_asr_model(SimpleNamespace(
        session_token=node._session_token, model="small"))
    assert response.accepted
    assert response.active_model == "small"
    assert node._asr_model == "small"
    assert node._asr_model_path.endswith("/small.pt")
    assert node._asr_client is candidate
    assert old_client.closed
    assert not node._asr_switching


def test_asr_model_switch_is_rejected_while_listening():
    node = _bare_node()
    node._asr_client = FakeAsrClient()
    node._smart_voice_enabled = True
    node._smart_voice_listening = True
    response = node._set_asr_model(SimpleNamespace(
        session_token=node._session_token, model="large"))
    assert not response.accepted
    assert "停止" in response.message
    assert node._asr_model == "medium"


def test_failed_asr_model_switch_preserves_previous_worker_and_model():
    node = _bare_node()
    old_client = FakeAsrClient()
    node._asr_client = old_client
    candidate = FakeAsrClient()

    def fail_start():
        raise RuntimeError("checkpoint missing")

    candidate.start = fail_start
    node._new_asr_client = lambda config, generation: candidate
    response = node._set_asr_model(SimpleNamespace(
        session_token=node._session_token, model="large"))
    assert not response.accepted
    assert response.active_model == "medium"
    assert node._asr_client is old_client
    assert not old_client.closed
    assert candidate.closed
    assert node._asr_worker_generation == 1
    assert not node._asr_switching


def test_switching_worker_events_cannot_overwrite_public_asr_state():
    node = _bare_node()
    node._asr_switching = True
    node._asr_event_for_generation(1, {
        "event": "status",
        "status": {
            "ready": False,
            "state": "UNAVAILABLE",
            "model": "large",
            "error": "candidate not ready",
        },
    })
    assert node._asr_model == "medium"
    assert node._asr_available is True
    assert node._asr_phase == "READY"


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_")]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d ASR authorization tests passed" % len(tests))
