#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest
from unittest import mock

from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
import rospy
from std_msgs.msg import Bool


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "move_base_pause_bridge_under_test",
    str(PACKAGE_ROOT / "scripts" / "move_base_pause_bridge.py"),
)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class MoveBasePauseBridgeTest(unittest.TestCase):
    def setUp(self):
        self.now = rospy.Time.from_sec(100.0)
        self.time_patch = mock.patch.object(
            BRIDGE.rospy.Time, "now", return_value=self.now)
        self.time_patch.start()

    def tearDown(self):
        self.time_patch.stop()

    @staticmethod
    def _bridge():
        bridge = BRIDGE.MoveBasePauseBridge.__new__(BRIDGE.MoveBasePauseBridge)
        bridge.lock = threading.RLock()
        bridge.paused = False
        bridge.coverage_topic_active = False
        bridge.coverage_owner_token = ""
        bridge.coverage_active = False
        bridge.require_coverage_state = False
        bridge.coverage_state_received = True
        bridge.retained_pose = None
        bridge.retained_goal_id = ""
        bridge.retained_reissue_allowed = False
        bridge.reissue_on_resume = True
        bridge.last_action_request_id = ""
        bridge._explicit_goal_ids = set()
        bridge._explicit_goal_tracking_saturated = False
        bridge._active_explicit_goal_ids = set()
        bridge._orphan_explicit_goal_ids = set()
        bridge._rejected_action_request_ids = set()
        bridge._cancel_tombstone_goal_ids = set()
        bridge._cancel_not_forwarded_ack_ids = set()
        bridge._issued_simple_goal_ids = set()
        bridge._orphan_ordinary_goal_ids = set()
        bridge._simple_goal_tracking_saturated = False
        bridge._active_ordinary_goal_ids = set()
        bridge._cancel_requested_goal_ids = set()
        bridge._lease_revoked_goal_ids = set()
        bridge._lease_expired_goal_ids = set()
        bridge._lease_cancel_last_wall = {}
        bridge._ai_goal_lease_wall = {}
        bridge.ai_heartbeat_timeout_sec = 1.0
        bridge.coverage_claim_cancel_timeout_sec = 0.50
        bridge.last_ai_heartbeat_goal_id = ""
        bridge.action_status_received = True
        bridge.max_action_request_age_sec = 2.0
        bridge.max_action_request_future_sec = 0.5
        bridge.required_action_server_node = "/move_base"
        bridge.action_goal_topic = "/move_base/goal"
        bridge.action_goal_request_topic = "/navigation_goal/action_request"
        bridge.action_status_topic = "/move_base/status"
        bridge.ai_heartbeat_topic = "/navigation_goal/ai_heartbeat"
        bridge.action_cancel_request_topic = "/navigation_goal/cancel_request"
        bridge.action_cancel_ack_topic = "/navigation_goal/cancel_ack"
        bridge.cancel_topic = "/move_base/cancel"
        bridge.coverage_owner_service_name = (
            "/navigation_pause/set_coverage_owner"
        )
        bridge.simple_goal_request_topic = "/move_base_simple/goal"
        bridge.simple_goal_output_topic = (
            "/navigation_goal/legacy_simple_input_disabled"
        )
        bridge._action_output_ready = lambda: ""
        bridge.goal_pub = _Publisher()
        bridge.action_goal_pub = _Publisher()
        bridge.action_cancel_ack_pub = _Publisher()
        bridge.cancel_pub = _Publisher()
        bridge.paused_pub = _Publisher()
        bridge.status_pub = _Publisher()
        return bridge

    @staticmethod
    def _action_request(identifier=None, stamp=99.8):
        request = MoveBaseActionGoal()
        request_stamp = rospy.Time.from_sec(stamp)
        request.header.stamp = request_stamp
        request.goal_id.stamp = request_stamp
        request.goal_id.id = identifier or ("sweeper-ai-" + "a" * 32)
        target = request.goal.target_pose
        target.header.stamp = request_stamp
        target.header.frame_id = "map"
        target.pose.position.x = 4.0
        target.pose.position.y = -1.5
        target.pose.orientation.z = 0.25
        target.pose.orientation.w = (1.0 - 0.25 ** 2) ** 0.5
        return request

    @staticmethod
    def _status(identifier, status):
        array = GoalStatusArray()
        item = GoalStatus()
        item.goal_id.id = identifier
        item.status = status
        array.status_list = [item]
        return array

    def _claim_after_terminal(self, bridge, token, goal_ids):
        responses = []
        worker = threading.Thread(target=lambda: responses.append(
            bridge._set_coverage_owner(SimpleNamespace(
                claim=True, owner_token=token))))
        worker.start()
        deadline = time.monotonic() + 1.0
        while (len(bridge.cancel_pub.messages) < len(goal_ids) and
               time.monotonic() < deadline):
            time.sleep(0.005)
        self.assertGreaterEqual(len(bridge.cancel_pub.messages), len(goal_ids))
        for goal_id in goal_ids:
            bridge._action_status_callback(self._status(
                goal_id, GoalStatus.RECALLED))
        worker.join(1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(1, len(responses))
        return responses[0]

    def test_simple_goal_is_forwarded_only_without_pause_or_coverage(self):
        bridge = self._bridge()
        request = PoseStamped()
        request.header.frame_id = "map"
        request.pose.position.x = 2.0
        bridge._simple_goal_callback(request)
        self.assertEqual([], bridge.goal_pub.messages)
        self.assertEqual(1, len(bridge.action_goal_pub.messages))
        action = bridge.action_goal_pub.messages[0]
        self.assertRegex(action.goal_id.id, r"^sweeper-simple-[0-9a-f]{32}$")
        self.assertEqual(2.0, action.goal.target_pose.pose.position.x)
        self.assertEqual(self.now, action.header.stamp)
        self.assertEqual(self.now, action.goal_id.stamp)
        self.assertEqual(self.now, action.goal.target_pose.header.stamp)
        self.assertEqual(
            {action.goal_id.id}, bridge._active_ordinary_goal_ids)

        bridge.paused = True
        bridge._simple_goal_callback(request)
        bridge.paused = False
        bridge.coverage_active = True
        bridge._simple_goal_callback(request)
        self.assertEqual(1, len(bridge.action_goal_pub.messages))
        self.assertIn("coverage owns move_base", bridge.status_pub.messages[-1].data)

    def test_valid_explicit_request_preserves_id_and_pose_but_uses_j6m_time(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)

        self.assertEqual(1, len(bridge.action_goal_pub.messages))
        forwarded = bridge.action_goal_pub.messages[0]
        self.assertEqual(request.goal_id.id, forwarded.goal_id.id)
        self.assertEqual(4.0, forwarded.goal.target_pose.pose.position.x)
        self.assertEqual(self.now, forwarded.header.stamp)
        self.assertEqual(self.now, forwarded.goal_id.stamp)
        self.assertEqual(self.now, forwarded.goal.target_pose.header.stamp)
        self.assertEqual({request.goal_id.id}, bridge._active_explicit_goal_ids)
        self.assertFalse(bridge.retained_reissue_allowed)

    def test_paused_coverage_missing_move_base_and_active_ai_each_reject(self):
        for configure in (
                lambda bridge: setattr(bridge, "paused", True),
                lambda bridge: setattr(bridge, "coverage_active", True),
                lambda bridge: setattr(
                    bridge, "_action_output_ready",
                    lambda: "AI action goal rejected: /move_base missing"),
                lambda bridge: bridge._active_explicit_goal_ids.add(
                    "sweeper-ai-" + "b" * 32),
        ):
            bridge = self._bridge()
            configure(bridge)
            bridge._action_goal_request_callback(self._action_request())
            self.assertEqual([], bridge.action_goal_pub.messages)

    def test_static_mode_rejects_all_ordinary_goals_until_coverage_latch_arrives(self):
        bridge = self._bridge()
        bridge.require_coverage_state = True
        bridge.coverage_state_received = False

        bridge._simple_goal_callback(PoseStamped())
        bridge._action_goal_request_callback(self._action_request())
        self.assertEqual([], bridge.goal_pub.messages)
        self.assertEqual([], bridge.action_goal_pub.messages)

        # A no-op release is useful for acquire compensation, but must not
        # masquerade as the coverage manager's initial fail-closed state.
        released = bridge._set_coverage_owner(SimpleNamespace(
            claim=False, owner_token="coverage-" + "c" * 32))
        self.assertTrue(released.success)
        self.assertFalse(bridge.coverage_state_received)

        bridge._coverage_callback(Bool(data=False))
        bridge._simple_goal_callback(PoseStamped())
        bridge._action_goal_request_callback(self._action_request())
        self.assertEqual([], bridge.goal_pub.messages)
        self.assertEqual(2, len(bridge.action_goal_pub.messages))

    def test_exact_heartbeat_renews_only_its_id_and_expiry_never_revives(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        identifier = request.goal_id.id
        self.assertIn(identifier, bridge._ai_goal_lease_wall)

        original_lease = bridge._ai_goal_lease_wall[identifier]
        bridge._ai_heartbeat_callback(GoalID(
            id="sweeper-ai-" + "b" * 32))
        self.assertEqual(original_lease, bridge._ai_goal_lease_wall[identifier])

        bridge._ai_goal_lease_wall[identifier] = time.monotonic() - 2.0
        bridge._ai_heartbeat_callback(GoalID(id=identifier))
        self.assertGreater(
            bridge._ai_goal_lease_wall[identifier], time.monotonic() - 1.0)

        bridge._ai_goal_lease_wall[identifier] = time.monotonic() - 2.0
        bridge._ai_lease_watchdog(None)
        self.assertEqual(identifier, bridge.cancel_pub.messages[-1].id)
        self.assertIn(identifier, bridge._lease_expired_goal_ids)
        expired_lease = bridge._ai_goal_lease_wall[identifier]
        bridge._ai_heartbeat_callback(GoalID(id=identifier))
        self.assertEqual(expired_lease, bridge._ai_goal_lease_wall[identifier])
        bridge._lease_cancel_last_wall[identifier] = time.monotonic() - 0.6
        bridge._ai_lease_watchdog(None)
        self.assertEqual(2, len(bridge.cancel_pub.messages))

    def test_duplicate_and_pre_canceled_ids_are_never_forwarded(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        bridge._action_goal_request_callback(request)
        self.assertEqual(1, len(bridge.action_goal_pub.messages))

        second = self._action_request("sweeper-ai-" + "b" * 32)
        cancel = GoalID(id=second.goal_id.id)
        bridge._cancel_callback(cancel)
        bridge._active_explicit_goal_ids.clear()
        bridge._action_goal_request_callback(second)
        self.assertEqual(1, len(bridge.action_goal_pub.messages))
        self.assertIn("already canceled", bridge.status_pub.messages[-1].data)

    def test_cancel_before_action_has_local_not_forwarded_ack_only(self):
        bridge = self._bridge()
        request = self._action_request()
        cancel = GoalID(id=request.goal_id.id)

        bridge._action_cancel_request_callback(cancel)
        self.assertEqual([], bridge.action_cancel_ack_pub.messages)
        bridge._action_goal_request_callback(request)
        self.assertEqual(1, len(bridge.action_cancel_ack_pub.messages))
        ack = bridge.action_cancel_ack_pub.messages[0]
        self.assertEqual(request.goal_id.id, ack.goal_id.id)
        self.assertEqual(GoalStatus.RECALLED, ack.status)
        self.assertEqual("not_forwarded", ack.text)
        self.assertEqual([], bridge.action_goal_pub.messages)

        # Once an ID has passed the bridge's acceptance lock, a dedicated
        # cancel must wait for move_base's genuine terminal status instead of
        # synthesizing another local acknowledgement.
        accepted = self._bridge()
        accepted._action_goal_request_callback(request)
        accepted._action_cancel_request_callback(cancel)
        self.assertEqual([], accepted.action_cancel_ack_pub.messages)
        self.assertEqual(request.goal_id.id, accepted.cancel_pub.messages[-1].id)

    def test_empty_status_never_proves_unknown_cancel_was_not_forwarded(self):
        bridge = self._bridge()
        bridge.action_status_received = False
        identifier = "sweeper-ai-" + "c" * 32
        cancel = GoalID(id=identifier)
        bridge._action_cancel_request_callback(cancel)
        self.assertEqual([], bridge.action_cancel_ack_pub.messages)

        bridge._action_status_callback(GoalStatusArray())
        bridge._action_cancel_request_callback(cancel)
        self.assertEqual([], bridge.action_cancel_ack_pub.messages)
        self.assertNotIn(identifier, bridge._rejected_action_request_ids)

    def test_malformed_stale_and_future_requests_are_rejected(self):
        requests = [
            self._action_request("sweeper-ai-" + "a" * 32 + "\n"),
            self._action_request(stamp=97.9),
            self._action_request(stamp=100.6),
        ]
        invalid_nsec = self._action_request()
        invalid_nsec.header.stamp.nsecs = 1000000000
        invalid_nsec.goal_id.stamp.nsecs = 1000000000
        invalid_nsec.goal.target_pose.header.stamp.nsecs = 1000000000
        requests.append(invalid_nsec)

        for request in requests:
            bridge = self._bridge()
            bridge._action_goal_request_callback(request)
            self.assertEqual([], bridge.action_goal_pub.messages)

    def test_restart_readiness_blocks_new_goals_and_coverage_claim(self):
        bridge = self._bridge()
        bridge.action_status_received = False
        bridge._simple_goal_callback(PoseStamped())
        bridge._action_goal_request_callback(self._action_request())
        response = bridge._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token="coverage-" + "c" * 32))

        self.assertEqual([], bridge.action_goal_pub.messages)
        self.assertFalse(response.success)
        self.assertFalse(response.claimed)
        self.assertIn("status is not ready", response.message)

    def test_safe_terminal_releases_slot_but_lost_keeps_lease_and_slot(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        bridge._lease_expired_goal_ids.add(request.goal_id.id)
        bridge._lease_cancel_last_wall[request.goal_id.id] = time.monotonic()
        bridge._action_status_callback(self._status(
            request.goal_id.id, GoalStatus.SUCCEEDED))
        self.assertEqual(set(), bridge._active_explicit_goal_ids)
        self.assertEqual(set(), bridge._lease_expired_goal_ids)
        self.assertNotIn(request.goal_id.id, bridge._lease_cancel_last_wall)

        second = self._action_request("sweeper-ai-" + "b" * 32)
        bridge._action_goal_request_callback(second)
        bridge._cancel_callback(GoalID(id=second.goal_id.id))
        bridge._action_status_callback(self._status(
            second.goal_id.id, GoalStatus.RECALLING))
        self.assertEqual(
            {second.goal_id.id}, bridge._active_explicit_goal_ids)
        bridge._action_status_callback(self._status(
            second.goal_id.id, GoalStatus.RECALLED))
        self.assertEqual(set(), bridge._active_explicit_goal_ids)

        lost = self._action_request("sweeper-ai-" + "c" * 32)
        bridge._action_goal_request_callback(lost)
        bridge._action_status_callback(self._status(
            lost.goal_id.id, GoalStatus.LOST))
        self.assertEqual({lost.goal_id.id}, bridge._active_explicit_goal_ids)
        bridge._ai_goal_lease_wall[lost.goal_id.id] = time.monotonic() - 2.0
        bridge._ai_lease_watchdog(None)
        self.assertEqual(lost.goal_id.id, bridge.cancel_pub.messages[-1].id)
        self.assertIn(lost.goal_id.id, bridge._lease_expired_goal_ids)

    def test_duplicate_status_never_releases_active_slot(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        duplicate = GoalStatusArray(status_list=[
            self._status(request.goal_id.id, GoalStatus.ACTIVE).status_list[0],
            self._status(request.goal_id.id, GoalStatus.RECALLED).status_list[0],
        ])
        bridge._action_status_callback(duplicate)
        self.assertIn(request.goal_id.id, bridge._active_explicit_goal_ids)
        self.assertIn(request.goal_id.id, bridge._lease_revoked_goal_ids)
        self.assertEqual(request.goal_id.id, bridge.cancel_pub.messages[-1].id)

    def test_restart_nonterminal_ai_goal_is_orphaned_and_canceled_to_terminal(self):
        bridge = self._bridge()
        bridge.action_status_received = False
        identifier = "sweeper-ai-" + "d" * 32

        bridge._action_status_callback(self._status(
            identifier, GoalStatus.ACTIVE))
        self.assertEqual({identifier}, bridge._orphan_explicit_goal_ids)
        self.assertEqual({identifier}, bridge._active_explicit_goal_ids)
        self.assertEqual(identifier, bridge.cancel_pub.messages[-1].id)
        self.assertNotIn(identifier, bridge._ai_goal_lease_wall)

        # Even a syntactically exact heartbeat from a newly restarted backend
        # cannot adopt the orphan discovered from move_base state.
        bridge._ai_heartbeat_callback(GoalID(id=identifier))
        self.assertNotIn(identifier, bridge._ai_goal_lease_wall)
        bridge._lease_cancel_last_wall[identifier] = time.monotonic() - 0.6
        bridge._ai_lease_watchdog(None)
        self.assertEqual(2, len(bridge.cancel_pub.messages))

        bridge._action_status_callback(self._status(
            identifier, GoalStatus.RECALLING))
        self.assertIn(identifier, bridge._orphan_explicit_goal_ids)
        bridge._action_status_callback(self._status(
            identifier, GoalStatus.RECALLED))
        self.assertNotIn(identifier, bridge._orphan_explicit_goal_ids)
        self.assertNotIn(identifier, bridge._active_explicit_goal_ids)

    def test_claim_revokes_heartbeat_and_retries_dropped_exact_cancel(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        identifier = request.goal_id.id
        original_lease = bridge._ai_goal_lease_wall[identifier]
        responses = []
        claim_thread = threading.Thread(target=lambda: responses.append(
            bridge._set_coverage_owner(SimpleNamespace(
                claim=True, owner_token="coverage-" + "9" * 32))))
        claim_thread.start()
        deadline = time.monotonic() + 1.0
        while not bridge.cancel_pub.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(bridge.cancel_pub.messages)
        self.assertTrue(claim_thread.is_alive())
        self.assertIn(identifier, bridge._lease_revoked_goal_ids)

        bridge._ai_heartbeat_callback(GoalID(id=identifier))
        self.assertEqual(original_lease, bridge._ai_goal_lease_wall[identifier])
        bridge._lease_cancel_last_wall[identifier] = time.monotonic() - 0.6
        bridge._ai_lease_watchdog(None)
        self.assertEqual(
            [identifier, identifier],
            [message.id for message in bridge.cancel_pub.messages],
        )
        self.assertTrue(claim_thread.is_alive())

        bridge._action_status_callback(self._status(
            identifier, GoalStatus.RECALLED))
        claim_thread.join(1.0)
        self.assertFalse(claim_thread.is_alive())
        self.assertTrue(responses[0].success)

    def test_pause_emits_cancel_all_and_exact_cancel_and_never_reissues_ai(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)

        response = bridge._set_paused(SimpleNamespace(data=True))
        self.assertTrue(response.success)
        self.assertEqual(2, len(bridge.cancel_pub.messages))
        self.assertEqual("", bridge.cancel_pub.messages[0].id)
        self.assertEqual(request.goal_id.id, bridge.cancel_pub.messages[1].id)
        self.assertEqual(rospy.Time(), bridge.cancel_pub.messages[1].stamp)

        response = bridge._set_paused(SimpleNamespace(data=True))
        self.assertTrue(response.success)
        self.assertIn("reasserted", response.message)
        self.assertEqual(4, len(bridge.cancel_pub.messages))
        self.assertEqual("", bridge.cancel_pub.messages[2].id)
        self.assertEqual(request.goal_id.id, bridge.cancel_pub.messages[3].id)

        response = bridge._set_paused(SimpleNamespace(data=False))
        self.assertTrue(response.success)
        self.assertIn("remains canceled", response.message)
        self.assertEqual([], bridge.goal_pub.messages)

    def test_pause_reissues_simple_target_as_new_exact_action_id(self):
        bridge = self._bridge()
        request = PoseStamped()
        request.header.frame_id = "map"
        request.pose.position.x = 2.0
        bridge._simple_goal_callback(request)
        original_id = bridge.action_goal_pub.messages[-1].goal_id.id

        paused = bridge._set_paused(SimpleNamespace(data=True))
        self.assertTrue(paused.success)
        self.assertEqual("", bridge.cancel_pub.messages[0].id)
        self.assertEqual(original_id, bridge.cancel_pub.messages[1].id)
        bridge._lease_cancel_last_wall[original_id] = time.monotonic() - 0.6
        bridge._ai_lease_watchdog(None)
        self.assertEqual(original_id, bridge.cancel_pub.messages[-1].id)
        resumed = bridge._set_paused(SimpleNamespace(data=False))
        self.assertTrue(resumed.success)
        self.assertEqual(2, len(bridge.action_goal_pub.messages))
        replacement = bridge.action_goal_pub.messages[-1]
        self.assertRegex(
            replacement.goal_id.id, r"^sweeper-simple-[0-9a-f]{32}$")
        self.assertNotEqual(original_id, replacement.goal_id.id)
        self.assertEqual(2.0, replacement.goal.target_pose.pose.position.x)
        self.assertEqual([], bridge.goal_pub.messages)
        response = bridge._set_paused(SimpleNamespace(data=False))
        self.assertEqual("already running", response.message)
        self.assertEqual([], bridge.goal_pub.messages)

    def test_coverage_claim_exactly_cancels_ai_and_clears_retention(self):
        bridge = self._bridge()
        request = self._action_request()
        bridge._action_goal_request_callback(request)
        bridge._coverage_callback(Bool(data=True))

        self.assertEqual(1, len(bridge.cancel_pub.messages))
        self.assertEqual(request.goal_id.id, bridge.cancel_pub.messages[0].id)
        self.assertIsNone(bridge.retained_pose)
        self.assertEqual("", bridge.retained_goal_id)
        self.assertFalse(bridge.retained_reissue_allowed)

    def test_synchronous_coverage_claim_linearizes_both_ai_race_orders(self):
        token = "coverage-" + "c" * 32

        # If AI wins the bridge lock first, the synchronous claim returns only
        # after publishing an exact cancellation for that explicit GoalID.
        ai_first = self._bridge()
        request = self._action_request()
        ai_first._action_goal_request_callback(request)
        response = self._claim_after_terminal(
            ai_first, token, [request.goal_id.id])
        self.assertTrue(response.success)
        self.assertTrue(response.claimed)
        self.assertEqual(token, response.current_owner_token)
        self.assertEqual(request.goal_id.id, ai_first.cancel_pub.messages[-1].id)
        self.assertIn(
            request.goal_id.id, ai_first._cancel_requested_goal_ids)

        # If coverage wins first, the same-node state lock makes the later AI
        # callback observe ownership and reject without forwarding anything.
        coverage_first = self._bridge()
        response = coverage_first._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=token))
        self.assertTrue(response.success)
        coverage_first._action_goal_request_callback(self._action_request())
        self.assertEqual([], coverage_first.action_goal_pub.messages)
        self.assertIn(
            "coverage owns move_base", coverage_first.status_pub.messages[-1].data)

    def test_synchronous_coverage_claim_linearizes_simple_goal_and_exact_cancel(self):
        token = "coverage-" + "e" * 32
        request = PoseStamped()
        request.header.frame_id = "map"

        simple_first = self._bridge()
        simple_first._simple_goal_callback(request)
        simple_id = simple_first.action_goal_pub.messages[-1].goal_id.id
        response = self._claim_after_terminal(
            simple_first, token, [simple_id])
        self.assertTrue(response.success)
        self.assertEqual([simple_id], [
            message.id for message in simple_first.cancel_pub.messages])
        self.assertTrue(all(
            message.id for message in simple_first.cancel_pub.messages))

        # A coverage segment emitted after the claim is never classified as an
        # ordinary bridge goal, so idempotent claim retries cannot cancel it.
        segment = SimpleNamespace(
            goal=SimpleNamespace(target_pose=PoseStamped()),
            goal_id=SimpleNamespace(id="coverage-segment-1"),
        )
        simple_first._goal_callback(segment)
        simple_first._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=token))
        self.assertTrue(all(
            message.id == simple_id for message in simple_first.cancel_pub.messages))

        coverage_first = self._bridge()
        self.assertTrue(coverage_first._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=token)).success)
        coverage_first._simple_goal_callback(request)
        self.assertEqual([], coverage_first.action_goal_pub.messages)

    def test_simple_publish_holds_same_lock_until_claim_can_exactly_cancel(self):
        bridge = self._bridge()
        entered_publish = threading.Event()
        release_publish = threading.Event()

        class BlockingPublisher(_Publisher):
            def publish(self, message):
                super().publish(message)
                entered_publish.set()
                self.assert_released = release_publish.wait(1.0)

        bridge.action_goal_pub = BlockingPublisher()
        request = PoseStamped()
        request.header.frame_id = "map"
        simple_thread = threading.Thread(
            target=bridge._simple_goal_callback, args=(request,))
        claim_responses = []
        claim_thread = threading.Thread(target=lambda: claim_responses.append(
            bridge._set_coverage_owner(SimpleNamespace(
                claim=True, owner_token="coverage-" + "f" * 32))))

        simple_thread.start()
        self.assertTrue(entered_publish.wait(1.0))
        claim_thread.start()
        time.sleep(0.01)
        self.assertTrue(claim_thread.is_alive())
        release_publish.set()
        simple_thread.join(1.0)
        deadline = time.monotonic() + 1.0
        while not bridge.cancel_pub.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(bridge.cancel_pub.messages)
        simple_id = bridge.action_goal_pub.messages[0].goal_id.id
        bridge._action_status_callback(self._status(
            simple_id, GoalStatus.RECALLED))
        claim_thread.join(1.0)

        self.assertFalse(simple_thread.is_alive())
        self.assertFalse(claim_thread.is_alive())
        self.assertTrue(claim_responses[0].success)
        self.assertEqual(simple_id, bridge.cancel_pub.messages[-1].id)

    def test_topic_false_and_old_token_cannot_release_synchronous_owner(self):
        bridge = self._bridge()
        owner = "coverage-" + "c" * 32
        old_owner = "coverage-" + "d" * 32
        self.assertTrue(bridge._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=owner)).success)

        bridge._coverage_callback(Bool(data=False))
        self.assertTrue(bridge.coverage_active)
        self.assertEqual(owner, bridge.coverage_owner_token)
        bridge._action_goal_request_callback(self._action_request())
        self.assertEqual([], bridge.action_goal_pub.messages)

        rejected = bridge._set_coverage_owner(SimpleNamespace(
            claim=False, owner_token=old_owner))
        self.assertFalse(rejected.success)
        self.assertTrue(rejected.claimed)
        self.assertEqual(owner, rejected.current_owner_token)

        released = bridge._set_coverage_owner(SimpleNamespace(
            claim=False, owner_token=owner))
        self.assertTrue(released.success)
        self.assertFalse(released.claimed)
        self.assertFalse(bridge.coverage_active)

    def test_same_owner_claim_and_empty_state_release_are_idempotent(self):
        bridge = self._bridge()
        owner = "coverage-" + "c" * 32
        first = bridge._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=owner))
        second = bridge._set_coverage_owner(SimpleNamespace(
            claim=True, owner_token=owner))
        self.assertTrue(first.success)
        self.assertTrue(second.success)
        self.assertEqual(owner, second.current_owner_token)

        self.assertTrue(bridge._set_coverage_owner(SimpleNamespace(
            claim=False, owner_token=owner)).success)
        repeated = bridge._set_coverage_owner(SimpleNamespace(
            claim=False, owner_token=owner))
        self.assertTrue(repeated.success)
        self.assertFalse(repeated.claimed)

    def test_ordinary_action_remains_reissuable_but_coverage_segment_does_not(self):
        bridge = self._bridge()
        bridge._submit_simple_action_locked(PoseStamped())
        ordinary = bridge.action_goal_pub.messages[-1]
        bridge._goal_callback(ordinary)
        self.assertTrue(bridge.retained_reissue_allowed)

        foreign = SimpleNamespace(
            goal=SimpleNamespace(target_pose=PoseStamped()),
            goal_id=SimpleNamespace(id="sweeper-simple-" + "1" * 32),
        )
        bridge._goal_callback(foreign)
        self.assertEqual(ordinary.goal_id.id, bridge.retained_goal_id)
        self.assertNotIn(
            foreign.goal_id.id, bridge._active_ordinary_goal_ids)

        bridge._coverage_callback(Bool(data=True))
        self.assertIsNone(bridge.retained_pose)
        segment = SimpleNamespace(
            goal=SimpleNamespace(target_pose=PoseStamped()),
            goal_id=SimpleNamespace(id="coverage-segment"),
        )
        bridge._goal_callback(segment)
        self.assertIsNone(bridge.retained_pose)

    def test_status_exposes_explicit_action_protocol(self):
        bridge = self._bridge()
        bridge._publish_status("test")
        payload = json.loads(bridge.status_pub.messages[-1].data)
        self.assertEqual(2, payload["action_request_version"])
        self.assertEqual(
            "/navigation_goal/action_request",
            payload["action_goal_request_topic"],
        )
        self.assertEqual("/move_base", payload["required_action_server_node"])
        self.assertEqual(
            "/navigation_goal/ai_heartbeat", payload["ai_heartbeat_topic"])
        self.assertEqual(
            "actionlib_msgs/GoalID", payload["ai_heartbeat_message_type"])
        self.assertTrue(payload["simple_goal_actionized"])
        self.assertEqual(
            "/navigation_goal/cancel_ack",
            payload["action_cancel_ack_topic"],
        )
        self.assertFalse(payload["coverage_owner_claimed"])
        self.assertEqual("", payload["coverage_owner_token"])
        self.assertEqual(
            "/navigation_pause/set_coverage_owner",
            payload["coverage_owner_service"],
        )


if __name__ == "__main__":
    unittest.main()
