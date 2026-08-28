#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS-free tests for explicit move_base GoalID ownership.

The final publishers/subscribers are in-memory doubles.  No test connects to a
ROS master or publishes a real navigation target.
"""

import json
import math
import os
import sys
import time
from types import SimpleNamespace


_THIS = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE_SRC = os.path.abspath(os.path.join(_THIS, "..", ".."))
_PKG_SRC = os.path.join(_WORKSPACE_SRC, "sweeper_mcp", "src")
_COVERAGE_SRC = os.path.join(
    _WORKSPACE_SRC, "application", "autolabor_coverage", "src")
for _path in (_PKG_SRC, _COVERAGE_SRC):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from sweeper_mcp.ros_backend import ROSBackend  # noqa: E402


class _Stamp:
    def __init__(self, seconds=0.0):
        seconds = float(seconds)
        self.secs = int(math.floor(seconds))
        self.nsecs = int(round((seconds - self.secs) * 1e9))

    def to_sec(self):
        return self.secs + self.nsecs * 1e-9


class _TimeFactory:
    def __init__(self, now_seconds):
        self._now_seconds = float(now_seconds)

    def __call__(self, seconds=0.0):
        return _Stamp(seconds)

    def now(self):
        return _Stamp(self._now_seconds)


class _OfflineRospy:
    def __init__(self, now_seconds=100.0, shutdown_after=4):
        self.Time = _TimeFactory(now_seconds)
        self._shutdown_after = shutdown_after
        self._shutdown_checks = 0

    def is_shutdown(self):
        self._shutdown_checks += 1
        return (self._shutdown_after is not None and
                self._shutdown_checks > self._shutdown_after)


class _PoseStamped:
    def __init__(self):
        self.header = SimpleNamespace(seq=0, stamp=_Stamp(), frame_id="")
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class _GoalID:
    def __init__(self):
        self.stamp = _Stamp()
        self.id = ""


class _MoveBaseActionGoal:
    def __init__(self):
        self.header = SimpleNamespace(seq=0, stamp=_Stamp(), frame_id="")
        self.goal_id = _GoalID()
        self.goal = SimpleNamespace(target_pose=_PoseStamped())


class _OfflinePublisher:
    def __init__(self, on_publish=None, connections=1):
        self.messages = []
        self._on_publish = on_publish
        self._connections = connections

    def publish(self, message):
        self.messages.append(message)
        if self._on_publish is not None:
            self._on_publish(message)

    def get_num_connections(self):
        return self._connections


def _goal_status(identifier, status=0, text=""):
    goal_id = _GoalID()
    goal_id.id = identifier
    goal_id.stamp = _Stamp(100.0)
    return SimpleNamespace(goal_id=goal_id, status=status, text=text)


def _status_array(*items):
    return SimpleNamespace(status_list=list(items))


def _cancel_ack(identifier):
    return SimpleNamespace(
        goal_id=SimpleNamespace(id=identifier),
        status=8,
        text="not_forwarded",
    )


def _copy_action(request, x_offset=0.0, bridge_stamp=100.1):
    """Model J6M restamping plus rospy top-header sequence rewriting."""
    action = _MoveBaseActionGoal()
    action.header.seq = request.header.seq + 1000
    action.header.stamp = _Stamp(bridge_stamp)
    action.goal_id.id = request.goal_id.id
    action.goal_id.stamp = _Stamp(bridge_stamp)
    source = request.goal.target_pose
    target = action.goal.target_pose
    target.header.seq = source.header.seq + 1000
    target.header.stamp = _Stamp(bridge_stamp)
    target.header.frame_id = source.header.frame_id
    target.pose.position.x = source.pose.position.x + x_offset
    target.pose.position.y = source.pose.position.y
    target.pose.position.z = source.pose.position.z
    target.pose.orientation.x = source.pose.orientation.x
    target.pose.orientation.y = source.pose.orientation.y
    target.pose.orientation.z = source.pose.orientation.z
    target.pose.orientation.w = source.pose.orientation.w
    return action


def _backend(on_publish=None, on_cancel=None, with_status=True, shutdown_after=4):
    backend = ROSBackend()
    backend._ready = True
    backend._rospy = _OfflineRospy(shutdown_after=shutdown_after)
    backend._types = {
        "GoalID": _GoalID,
        "MoveBaseActionGoal": _MoveBaseActionGoal,
    }
    backend._ordinary_navigation_ready = lambda: ""
    backend._goal_confirmation_poll = 0.0
    request_pub = _OfflinePublisher(on_publish)
    cancel_pub = _OfflinePublisher(on_cancel)
    backend._action_request_pub = request_pub
    backend._cancel_request_pub = cancel_pub
    backend._heartbeat_pub = _OfflinePublisher()
    if with_status:
        backend._cache("nav_status")(_status_array(
            _goal_status("old-qt-goal", status=3)))
    return backend, request_pub, cancel_pub


def _publish(backend):
    return backend._publish_owned_goal(
        "map", 4.25, -1.5, math.radians(35.0))


def _assert_failed_and_exactly_canceled(result, backend, request_pub, cancel_pub):
    assert result.is_error, result
    assert len(request_pub.messages) == 1
    request_id = request_pub.messages[0].goal_id.id
    assert request_id.startswith("sweeper-ai-")
    assert backend._ai_goal_id == request_id
    assert backend._ai_goal_cancel_uncertain is True
    assert len(cancel_pub.messages) >= 1
    assert all(message.id == request_id for message in cancel_pub.messages)
    assert all(message.id != "old-qt-goal" for message in cancel_pub.messages)
    assert all(message.stamp.to_sec() == 0.0
               for message in cancel_pub.messages)
    assert json.loads(result.text)["cancel_state"] == "uncertain"


def test_explicit_id_round_trip_succeeds_after_j6m_restamp_and_seq_rewrite():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request))
        backend._cache("nav_status")(_status_array(
            _goal_status("old-qt-goal", status=3),
            _goal_status(request.goal_id.id, status=1, text="active"),
        ))

    backend, request_pub, cancel_pub = _backend(on_publish)
    holder["backend"] = backend
    result = _publish(backend)

    assert not result.is_error, result
    payload = json.loads(result.text)
    request = request_pub.messages[0]
    assert payload["accepted"] is True
    assert payload["state"] == "active"
    assert payload["goal_id"] == request.goal_id.id
    assert backend._ai_goal_id == request.goal_id.id
    assert request.goal_id.id.startswith("sweeper-ai-")
    assert len(request.goal_id.id) == len("sweeper-ai-") + 32
    assert request.header.stamp.to_sec() == request.goal_id.stamp.to_sec()
    assert request.goal_id.stamp.to_sec() == (
        request.goal.target_pose.header.stamp.to_sec())
    assert cancel_pub.messages == []


def test_foreign_action_and_status_can_never_be_inferred_as_ai_goal():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        foreign = _copy_action(request)
        foreign.goal_id.id = "manual-qt-goal"
        backend._action_goal_callback(foreign)
        backend._cache("nav_status")(_status_array(
            _goal_status("manual-qt-goal", status=1)))

    backend, request_pub, cancel_pub = _backend(on_publish, shutdown_after=0)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert cancel_pub.messages[0].id != "manual-qt-goal"


def test_exact_echo_without_exact_status_fails_and_cancels_exact_id():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request))
        backend._cache("nav_status")(_status_array(
            _goal_status("old-qt-goal", status=3)))

    backend, request_pub, cancel_pub = _backend(on_publish, shutdown_after=0)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert "同 ID 状态" in result.text


def test_exact_status_without_exact_echo_fails_and_cancels_exact_id():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._cache("nav_status")(_status_array(
            _goal_status(request.goal_id.id, status=1)))

    backend, request_pub, cancel_pub = _backend(on_publish, shutdown_after=0)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert "同 ID action 回显" in result.text


def test_same_id_with_changed_target_is_rejected_and_exactly_canceled():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request, x_offset=0.25))
        backend._cache("nav_status")(_status_array(
            _goal_status(request.goal_id.id, status=1)))

    backend, request_pub, cancel_pub = _backend(on_publish)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert "目标位姿与请求不一致" in result.text


def test_duplicate_same_id_echoes_are_rejected_and_exactly_canceled():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request))
        backend._action_goal_callback(_copy_action(request))
        backend._cache("nav_status")(_status_array(
            _goal_status(request.goal_id.id, status=1)))

    backend, request_pub, cancel_pub = _backend(on_publish)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert "重复" in result.text


def test_duplicate_same_id_statuses_are_rejected_and_exactly_canceled():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request))
        backend._cache("nav_status")(_status_array(
            _goal_status(request.goal_id.id, status=0),
            _goal_status(request.goal_id.id, status=1),
        ))

    backend, request_pub, cancel_pub = _backend(on_publish)
    holder["backend"] = backend
    result = _publish(backend)

    _assert_failed_and_exactly_canceled(
        result, backend, request_pub, cancel_pub)
    assert "重复的显式 GoalID" in result.text


def test_failed_goal_is_released_only_after_same_id_cancel_confirmation():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._action_goal_callback(_copy_action(request, x_offset=0.5))
        backend._cache("nav_status")(_status_array(
            _goal_status(request.goal_id.id, status=1)))

    def on_cancel(cancel):
        backend = holder["backend"]
        backend._cache("nav_status")(_status_array(
            _goal_status(cancel.id, status=8, text="recalled")))

    backend, request_pub, cancel_pub = _backend(on_publish, on_cancel=on_cancel)
    holder["backend"] = backend
    result = _publish(backend)

    assert result.is_error, result
    assert len(request_pub.messages) == 1
    assert len(cancel_pub.messages) >= 1
    assert all(message.id == request_pub.messages[0].goal_id.id
               for message in cancel_pub.messages)
    assert backend._ai_goal_id == request_pub.messages[0].goal_id.id
    assert backend._ai_goal_cancel_uncertain is False
    assert backend._ai_goal_cancel_confirmed is True
    assert json.loads(backend.get_navigation_status().text)["state"] == "recalled"
    assert "确认为 recalled" in result.text


def test_bridge_not_forwarded_ack_safely_closes_cancel_before_goal_race():
    holder = {}

    def on_publish(request):
        # Model a request delayed on its own TCPROS connection: no move_base
        # echo or status can exist yet.
        holder["request_id"] = request.goal_id.id

    def on_cancel(cancel):
        holder["backend"]._cancel_ack_callback(_cancel_ack(cancel.id))

    backend, request_pub, cancel_pub = _backend(
        on_publish, on_cancel=on_cancel, shutdown_after=None)
    holder["backend"] = backend
    backend._goal_confirmation_timeout = 0.0
    result = _publish(backend)

    assert result.is_error, result
    payload = json.loads(result.text)
    assert payload["cancel_state"] == "confirmed"
    assert payload["confirmed_state"] == "recalled_before_forward"
    assert payload["goal_id"] == holder["request_id"]
    assert backend._ai_goal_cancel_uncertain is False
    assert backend._ai_goal_cancel_confirmed is True
    assert json.loads(backend.get_navigation_status().text)["state"] == (
        "recalled_before_forward")
    assert len(cancel_pub.messages) == 1
    assert cancel_pub.messages[0].id == holder["request_id"]


def test_unconfirmed_cancel_keeps_id_visible_and_blocks_next_navigation():
    holder = {}

    def on_publish(request):
        backend = holder["backend"]
        backend._cache("nav_status")(_status_array(
            _goal_status("manual-qt-goal", status=1)))

    backend, request_pub, cancel_pub = _backend(on_publish, shutdown_after=0)
    holder["backend"] = backend
    first = _publish(backend)
    owned_id = request_pub.messages[0].goal_id.id

    assert first.is_error
    status = json.loads(backend.get_navigation_status().text)
    assert status["state"] == "cancel_uncertain"
    assert status["goal_id"] == owned_id
    second = _publish(backend)
    assert second.is_error
    assert "精确取消尚未确认" in second.text
    assert len(request_pub.messages) == 1
    assert len(cancel_pub.messages) >= 1
    assert all(message.id == owned_id for message in cancel_pub.messages)

    backend._cache("nav_status")(_status_array(
        _goal_status(owned_id, status=8, text="recalled")))
    terminal = json.loads(backend.get_navigation_status().text)
    assert terminal["state"] == "recalled"
    assert backend._ai_goal_id == owned_id


def test_missing_status_stream_rejects_before_publish():
    backend, request_pub, cancel_pub = _backend(
        with_status=False, shutdown_after=0)
    result = _publish(backend)

    assert result.is_error, result
    assert request_pub.messages == []
    assert cancel_pub.messages == []
    assert backend._ai_goal_id == ""


def test_existing_active_ai_goal_rejects_new_request_before_publish():
    backend, request_pub, cancel_pub = _backend()
    backend._ai_goal_id = "sweeper-ai-" + "1" * 32
    backend._cache("nav_status")(_status_array(
        _goal_status(backend._ai_goal_id, status=1)))
    result = _publish(backend)

    assert result.is_error, result
    assert "仍在执行" in result.text
    assert request_pub.messages == []
    assert cancel_pub.messages == []


def test_lost_ai_goal_is_retained_exactly_canceled_and_blocks_replacement():
    backend, request_pub, cancel_pub = _backend(shutdown_after=0)
    backend._ai_goal_id = "sweeper-ai-" + "2" * 32
    backend._cache("nav_status")(_status_array(
        _goal_status(backend._ai_goal_id, status=9, text="lost")))

    status = json.loads(backend.get_navigation_status().text)
    result = _publish(backend)

    assert status["state"] == "cancel_uncertain"
    assert status["move_base_state"] == "lost"
    assert backend._ai_goal_cancel_uncertain is True
    assert result.is_error, result
    assert "精确取消尚未确认" in result.text
    assert request_pub.messages == []
    assert len(cancel_pub.messages) == 1
    assert cancel_pub.messages[0].id == backend._ai_goal_id


def test_duplicate_owned_status_is_cancel_uncertain_not_released():
    backend, request_pub, cancel_pub = _backend(shutdown_after=0)
    backend._ai_goal_id = "sweeper-ai-" + "3" * 32
    backend._cache("nav_status")(_status_array(
        _goal_status(backend._ai_goal_id, status=1),
        _goal_status(backend._ai_goal_id, status=3)))

    status = json.loads(backend.get_navigation_status().text)
    result = _publish(backend)

    assert status["state"] == "cancel_uncertain"
    assert backend._ai_goal_cancel_uncertain is True
    assert result.is_error, result
    assert request_pub.messages == []
    assert len(cancel_pub.messages) == 1
    assert cancel_pub.messages[0].id == backend._ai_goal_id


def test_concurrent_submission_is_rejected_before_publish():
    backend, request_pub, cancel_pub = _backend()
    assert backend._goal_submit_lock.acquire(False)
    try:
        result = _publish(backend)
    finally:
        backend._goal_submit_lock.release()

    assert result.is_error, result
    assert "正在提交" in result.text
    assert request_pub.messages == []
    assert cancel_pub.messages == []


def test_heartbeat_is_exact_goal_id_and_restart_cannot_renew_old_owner():
    backend, _request_pub, _cancel_pub = _backend()
    backend._heartbeat_timer_callback(None)
    assert backend._heartbeat_pub.messages == []

    owned_id = "sweeper-ai-" + "4" * 32
    backend._ai_goal_id = owned_id
    backend._heartbeat_timer_callback(None)
    assert backend._heartbeat_pub.messages == []

    backend._touch_ai_goal_supervision(owned_id)
    backend._heartbeat_timer_callback(None)
    assert len(backend._heartbeat_pub.messages) == 1
    heartbeat = backend._heartbeat_pub.messages[0]
    assert heartbeat.id == owned_id
    assert heartbeat.stamp.to_sec() == 0.0

    backend._ai_goal_supervision_wall = (
        time.monotonic() - backend._ai_goal_supervision_timeout - 0.1)
    backend._heartbeat_timer_callback(None)
    assert len(backend._heartbeat_pub.messages) == 1

    # Agent's normal status poll refreshes supervision for this exact ID.
    backend._cache("nav_status")(_status_array(
        _goal_status(owned_id, status=1)))
    backend.get_navigation_status()
    backend._heartbeat_timer_callback(None)
    assert len(backend._heartbeat_pub.messages) == 2

    restarted, _request_pub, _cancel_pub = _backend()
    restarted._heartbeat_timer_callback(None)
    assert restarted._ai_goal_id == ""
    assert restarted._heartbeat_pub.messages == []


def test_recalling_is_not_a_trusted_terminal_for_owner_transfer():
    assert 7 not in ROSBackend.CANCEL_SAFE_STATUSES
    assert 8 in ROSBackend.CANCEL_SAFE_STATUSES


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items())
             if name.startswith("test_") and callable(value)]
    for test in tests:
        test()
        print("PASS", test.__name__)
    print("%d explicit goal ownership tests passed" % len(tests))
