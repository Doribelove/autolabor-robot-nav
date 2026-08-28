#!/usr/bin/env python3
"""Gate ordinary navigation, pause move_base, and retain only safe targets.

Qt/RViz keeps using the familiar ``PoseStamped`` request path.  The bridge
converts it to an explicitly identified action goal instead of relying on
move_base's anonymous simple-goal adapter.  The AI path is separate: NVIDIA
assigns a non-empty GoalID before publishing a
``MoveBaseActionGoal`` to ``/navigation_goal/action_request``.  This bridge
keeps its explicit ID and target, normalizes timestamps to J6M time, then
forwards it so the AI can follow, and if needed cancel, exactly its own
move_base goal.
"""

import copy
import json
import math
import re
import threading
import time
import uuid

import rosgraph
import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from autolabor_coverage.srv import SetCoverageOwner, SetCoverageOwnerResponse
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse


class MoveBasePauseBridge:
    AI_GOAL_ID_RE = re.compile(r"^sweeper-ai-[0-9a-f]{32}$")
    SIMPLE_GOAL_ID_RE = re.compile(r"^sweeper-simple-[0-9a-f]{32}$")
    COVERAGE_OWNER_TOKEN_RE = re.compile(r"^coverage-[0-9a-f]{32}$")
    MAX_TRACKED_AI_GOALS = 10000
    MAX_TRACKED_SIMPLE_GOALS = 10000

    def __init__(self):
        self.action_goal_topic = rospy.resolve_name(rospy.get_param(
            "~action_goal_topic", "/move_base/goal"
        ))
        self.action_goal_request_topic = rospy.resolve_name(rospy.get_param(
            "~action_goal_request_topic", "/navigation_goal/action_request"
        ))
        self.action_status_topic = rospy.resolve_name(rospy.get_param(
            "~action_status_topic", "/move_base/status"
        ))
        self.ai_heartbeat_topic = rospy.resolve_name(rospy.get_param(
            "~ai_heartbeat_topic", "/navigation_goal/ai_heartbeat"
        ))
        self.action_cancel_request_topic = rospy.resolve_name(rospy.get_param(
            "~action_cancel_request_topic", "/navigation_goal/cancel_request"
        ))
        self.action_cancel_ack_topic = rospy.resolve_name(rospy.get_param(
            "~action_cancel_ack_topic", "/navigation_goal/cancel_ack"
        ))
        self.simple_goal_request_topic = rospy.resolve_name(rospy.get_param(
            "~simple_goal_request_topic", "/move_base_simple/goal"
        ))
        self.simple_goal_output_topic = rospy.resolve_name(rospy.get_param(
            "~simple_goal_output_topic",
            "/navigation_goal/legacy_simple_input_disabled"
        ))
        self.cancel_topic = rospy.resolve_name(rospy.get_param(
            "~cancel_topic", "/move_base/cancel"
        ))
        self.coverage_owner_service_name = rospy.resolve_name(rospy.get_param(
            "~coverage_owner_service", "~set_coverage_owner"
        ))
        if len({
                self.action_goal_topic,
                self.action_goal_request_topic,
                self.action_status_topic,
                self.ai_heartbeat_topic,
                self.action_cancel_request_topic,
                self.action_cancel_ack_topic,
                self.simple_goal_request_topic,
                self.simple_goal_output_topic,
                self.cancel_topic,
        }) != 9:
            raise ValueError("navigation request and output topics must be distinct")
        self.reissue_on_resume = self._strict_bool("~reissue_on_resume", True)
        self.require_coverage_state = self._strict_bool(
            "~require_coverage_state", False
        )
        self.max_action_request_age_sec = self._strict_positive_float(
            "~max_action_request_age_sec", 2.0
        )
        self.max_action_request_future_sec = self._strict_positive_float(
            "~max_action_request_future_sec", 0.5
        )
        self.ai_heartbeat_timeout_sec = self._strict_positive_float(
            "~ai_heartbeat_timeout_sec", 1.0
        )
        self.coverage_claim_cancel_timeout_sec = self._strict_positive_float(
            "~coverage_claim_cancel_timeout_sec", 2.0
        )
        self.required_action_server_node = rospy.resolve_name(rospy.get_param(
            "~required_action_server_node", "/move_base"
        ))
        self.lock = threading.RLock()
        self.paused = False
        self.retained_pose = None
        self.retained_goal_id = ""
        self.retained_reissue_allowed = False
        self.coverage_topic_active = False
        self.coverage_owner_token = ""
        self.coverage_active = False
        self.coverage_state_received = not self.require_coverage_state
        self.last_action_request_id = ""
        self._explicit_goal_ids = set()
        self._explicit_goal_tracking_saturated = False
        self._active_explicit_goal_ids = set()
        self._orphan_explicit_goal_ids = set()
        self._rejected_action_request_ids = set()
        self._cancel_tombstone_goal_ids = set()
        self._cancel_not_forwarded_ack_ids = set()
        self._issued_simple_goal_ids = set()
        self._orphan_ordinary_goal_ids = set()
        self._simple_goal_tracking_saturated = False
        self._active_ordinary_goal_ids = set()
        self._cancel_requested_goal_ids = set()
        self._lease_revoked_goal_ids = set()
        self._lease_expired_goal_ids = set()
        self._lease_cancel_last_wall = {}
        self._ai_goal_lease_wall = {}
        self.last_ai_heartbeat_goal_id = ""
        self.action_status_received = False

        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=5)
        self.goal_pub = rospy.Publisher(
            self.simple_goal_output_topic, PoseStamped, queue_size=1
        )
        self.action_goal_pub = rospy.Publisher(
            self.action_goal_topic, MoveBaseActionGoal, queue_size=2
        )
        self.action_cancel_ack_pub = rospy.Publisher(
            self.action_cancel_ack_topic, GoalStatus, queue_size=5
        )
        self.paused_pub = rospy.Publisher("~paused", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber(
            self.action_goal_topic,
            MoveBaseActionGoal,
            self._goal_callback,
            queue_size=10,
        )
        self.simple_goal_sub = rospy.Subscriber(
            self.simple_goal_request_topic,
            PoseStamped,
            self._simple_goal_callback,
            queue_size=10,
        )
        self.action_goal_request_sub = rospy.Subscriber(
            self.action_goal_request_topic,
            MoveBaseActionGoal,
            self._action_goal_request_callback,
            queue_size=10,
        )
        self.action_status_sub = rospy.Subscriber(
            self.action_status_topic,
            GoalStatusArray,
            self._action_status_callback,
            queue_size=10,
        )
        self.cancel_sub = rospy.Subscriber(
            self.cancel_topic,
            GoalID,
            self._cancel_callback,
            queue_size=20,
        )
        self.ai_heartbeat_sub = rospy.Subscriber(
            self.ai_heartbeat_topic,
            GoalID,
            self._ai_heartbeat_callback,
            queue_size=5,
        )
        self.action_cancel_request_sub = rospy.Subscriber(
            self.action_cancel_request_topic,
            GoalID,
            self._action_cancel_request_callback,
            queue_size=20,
        )
        self.coverage_sub = rospy.Subscriber(
            "/coverage/active", Bool, self._coverage_callback, queue_size=5
        )
        self.service = rospy.Service("~set_paused", SetBool, self._set_paused)
        self.coverage_owner_service = rospy.Service(
            self.coverage_owner_service_name,
            SetCoverageOwner,
            self._set_coverage_owner,
        )
        self.ai_lease_timer = rospy.Timer(
            rospy.Duration(0.2), self._ai_lease_watchdog)
        self._publish_status("ready")

    @staticmethod
    def _strict_bool(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not bool:
            raise ValueError("{} must be a YAML boolean".format(name))
        return value

    @staticmethod
    def _strict_positive_float(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("{} must be a number".format(name))
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))
        return value

    def _refresh_coverage_active_locked(self):
        """Combine the synchronous owner with the conservative topic latch."""
        self.coverage_active = bool(
            self.coverage_owner_token or self.coverage_topic_active
        )

    def _activate_coverage_locked(self):
        """Apply fail-closed side effects after either ownership source wins."""
        cancel_ids = sorted(
            self._active_explicit_goal_ids |
            self._active_ordinary_goal_ids
        )
        self._cancel_requested_goal_ids.update(cancel_ids)
        revoked_ai_ids = set(cancel_ids) & self._active_explicit_goal_ids
        self._lease_revoked_goal_ids.update(revoked_ai_ids)
        self._lease_expired_goal_ids.update(revoked_ai_ids)
        # Coverage is a new mission owner.  Never reissue either a previous
        # ordinary goal or a coverage segment endpoint after a later safety
        # pause/resume cycle.
        self.retained_pose = None
        self.retained_goal_id = ""
        self.retained_reissue_allowed = False
        return cancel_ids

    def _wait_for_exact_cancellation_terminal(self, goal_ids):
        """Wait outside the lock until every pre-claim goal is truly terminal."""
        pending = set(goal_ids)
        deadline = time.monotonic() + self.coverage_claim_cancel_timeout_sec
        while pending and time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                pending &= (
                    self._active_explicit_goal_ids |
                    self._active_ordinary_goal_ids
                )
            if pending:
                time.sleep(0.02)
        return not pending, sorted(pending)

    def _goal_callback(self, message):
        with self.lock:
            goal_id = str(message.goal_id.id)
            is_known_ai = (
                self.AI_GOAL_ID_RE.fullmatch(goal_id) is not None and
                goal_id in self._explicit_goal_ids
            )
            is_bridge_simple = (
                self.SIMPLE_GOAL_ID_RE.fullmatch(goal_id) is not None and
                goal_id in self._issued_simple_goal_ids
            )
            if not (is_known_ai or is_bridge_simple):
                # Never retain or later replay an action published by another
                # owner.  Coverage segments are also foreign here and remain
                # solely under coverage_manager's lifecycle.
                return
            # Both accepted paths install their retained target and active ID
            # before publishing.  The /move_base/goal echo may arrive after a
            # terminal status, so it must never resurrect that state or make a
            # completed target reissuable again.
        self._publish_status("owned navigation action observed")

    @staticmethod
    def _stamp_tuple(stamp):
        try:
            secs, nsecs = int(stamp.secs), int(stamp.nsecs)
        except (AttributeError, TypeError, ValueError):
            return None
        if secs < 0 or nsecs < 0 or nsecs >= 1000000000:
            return None
        return secs, nsecs

    def _validate_action_request(self, message):
        try:
            goal_id = str(message.goal_id.id)
            if self.AI_GOAL_ID_RE.fullmatch(goal_id) is None:
                return "AI action goal rejected: invalid explicit goal ID"

            stamps = (
                self._stamp_tuple(message.header.stamp),
                self._stamp_tuple(message.goal_id.stamp),
                self._stamp_tuple(message.goal.target_pose.header.stamp),
            )
            if any(stamp is None for stamp in stamps):
                return "AI action goal rejected: invalid ROS timestamp"
            if any(stamp == (0, 0) for stamp in stamps):
                return "AI action goal rejected: zero ROS timestamp"
            if stamps[0] != stamps[1] or stamps[1] != stamps[2]:
                return "AI action goal rejected: inconsistent ROS timestamps"
            request_sec = stamps[0][0] + stamps[0][1] * 1e-9
            now_sec = rospy.Time.now().to_sec()
            if request_sec < now_sec - self.max_action_request_age_sec:
                return "AI action goal rejected: stale ROS timestamp"
            if request_sec > now_sec + self.max_action_request_future_sec:
                return "AI action goal rejected: future ROS timestamp"

            target = message.goal.target_pose
            if not str(target.header.frame_id).strip():
                return "AI action goal rejected: empty target frame"
            position = target.pose.position
            orientation = target.pose.orientation
            values = (
                position.x, position.y, position.z,
                orientation.x, orientation.y,
                orientation.z, orientation.w,
            )
            if not all(math.isfinite(float(value)) for value in values):
                return "AI action goal rejected: non-finite target pose"
            if abs(float(orientation.x)) > 1e-6 or abs(float(orientation.y)) > 1e-6:
                return "AI action goal rejected: target is not planar"
            quaternion_norm = math.sqrt(sum(
                float(value) * float(value)
                for value in (
                    orientation.x, orientation.y,
                    orientation.z, orientation.w,
                )
            ))
            if abs(quaternion_norm - 1.0) > 1e-3:
                return "AI action goal rejected: target quaternion is not normalized"
        except (AttributeError, TypeError, ValueError):
            return "AI action goal rejected: malformed action request"
        return ""

    def _remember_explicit_goal_id_locked(self, goal_id):
        if goal_id in self._explicit_goal_ids:
            return False
        # Never evict IDs during this bridge process: an evicted ID could be
        # replayed after actionlib's short status retention expires.  Hitting
        # the generous cap is fail-closed and requires a controlled restart.
        if len(self._explicit_goal_ids) >= self.MAX_TRACKED_AI_GOALS:
            self._explicit_goal_tracking_saturated = True
            return False
        self._explicit_goal_ids.add(goal_id)
        return True

    def _remember_rejected_action_request_locked(self, goal_id):
        """Remember a request this bridge process proved it never forwarded."""
        if goal_id in self._rejected_action_request_ids:
            return True
        if len(self._rejected_action_request_ids) >= self.MAX_TRACKED_AI_GOALS:
            self._explicit_goal_tracking_saturated = True
            return False
        self._rejected_action_request_ids.add(goal_id)
        return True

    def _publish_not_forwarded_ack_locked(self, goal_id):
        """Acknowledge only a current-process request rejected under this lock."""
        if (self._explicit_goal_tracking_saturated or
                goal_id in self._explicit_goal_ids or
                goal_id not in self._rejected_action_request_ids or
                goal_id not in self._cancel_tombstone_goal_ids or
                goal_id in self._cancel_not_forwarded_ack_ids):
            return False
        acknowledgement = GoalStatus()
        acknowledgement.goal_id.stamp = rospy.Time()
        acknowledgement.goal_id.id = goal_id
        acknowledgement.status = GoalStatus.RECALLED
        acknowledgement.text = "not_forwarded"
        self._cancel_not_forwarded_ack_ids.add(goal_id)
        self.action_cancel_ack_pub.publish(acknowledgement)
        return True

    def _action_output_ready(self):
        """Require the configured move_base node on both action endpoints."""
        try:
            _publishers, subscribers, _services = rosgraph.Master(
                rospy.get_name()).getSystemState()
        except Exception as exc:
            return "AI action goal rejected: ROS master query failed: {}".format(exc)
        subscriber_map = dict(subscribers)
        for topic in (self.action_goal_topic, self.cancel_topic):
            if self.required_action_server_node not in subscriber_map.get(topic, []):
                return (
                    "AI action goal rejected: {} is not subscribed by {}".format(
                        topic, self.required_action_server_node)
                )
        return ""

    def _action_goal_request_callback(self, message):
        validation_error = self._validate_action_request(message)
        output_error = "" if validation_error else self._action_output_ready()
        accepted = None
        not_forwarded_ack = False
        with self.lock:
            goal_id = str(getattr(message.goal_id, "id", ""))
            if validation_error:
                reason = validation_error
            elif output_error:
                reason = output_error
            elif not self.action_status_received:
                reason = "AI action goal rejected: move_base action status is not ready"
            elif not self.coverage_state_received:
                reason = "AI action goal rejected: coverage ownership state is not ready"
            elif self.coverage_active:
                reason = "AI action goal rejected: coverage owns move_base"
            elif self.paused:
                reason = "AI action goal rejected while navigation is paused"
            elif goal_id in self._cancel_requested_goal_ids:
                reason = "AI action goal rejected: this goal ID was already canceled"
            elif self._active_explicit_goal_ids:
                reason = "AI action goal rejected: another AI goal is still active"
            elif not self._remember_explicit_goal_id_locked(goal_id):
                reason = "AI action goal rejected: duplicate explicit goal ID"
            else:
                accepted = copy.deepcopy(message)
                # NVIDIA and J6M clocks are checked for freshness above, then
                # all action timestamps are normalized to J6M time.  This
                # prevents a slightly future remote stamp from making a later
                # local coverage goal look older to SimpleActionServer.
                bridge_stamp = rospy.Time.now()
                accepted.header.stamp = bridge_stamp
                accepted.goal_id.stamp = bridge_stamp
                accepted.goal.target_pose.header.stamp = bridge_stamp
                self.last_action_request_id = goal_id
                self.retained_pose = copy.deepcopy(accepted.goal.target_pose)
                self.retained_goal_id = goal_id
                self.retained_reissue_allowed = False
                self._active_explicit_goal_ids.add(goal_id)
                # The lease is born with this exact accepted ID.  No global
                # heartbeat sent by a later NVIDIA process can extend it.
                self._ai_goal_lease_wall[goal_id] = time.monotonic()
                # Publish while holding the state lock.  A pause/coverage
                # callback can therefore only run either before this gate or
                # after the action has been submitted, in which case its
                # existing cancel path applies.
                self.action_goal_pub.publish(accepted)
                reason = "AI action goal accepted: {}".format(goal_id)
            if (accepted is None and
                    self.AI_GOAL_ID_RE.fullmatch(goal_id) is not None and
                    goal_id not in self._explicit_goal_ids):
                self._remember_rejected_action_request_locked(goal_id)
                not_forwarded_ack = self._publish_not_forwarded_ack_locked(
                    goal_id)
                if not_forwarded_ack:
                    reason = (
                        "AI action goal rejected after exact cancel tombstone: {}"
                    ).format(goal_id)
        self._publish_status(reason)

    def _action_status_callback(self, message):
        terminal = {
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
        }
        cancel_ids = []
        with self.lock:
            self.action_status_received = True
            statuses_by_id = {}
            for item in message.status_list:
                goal_id = str(item.goal_id.id)
                statuses_by_id.setdefault(goal_id, []).append(int(item.status))
            for goal_id, statuses in statuses_by_id.items():
                ambiguous = len(statuses) != 1
                status = statuses[0]
                if self.AI_GOAL_ID_RE.fullmatch(goal_id) is not None:
                    known_to_process = goal_id in self._explicit_goal_ids
                    if not known_to_process:
                        remembered = self._remember_explicit_goal_id_locked(goal_id)
                        if remembered and (ambiguous or status not in terminal):
                            # A nonterminal AI ID which this bridge process did
                            # not accept may belong to a backend that died or to
                            # the preceding bridge process.  It is an orphan:
                            # no heartbeat may adopt it, and exact cancel is
                            # retried until move_base reports a true terminal.
                            self._orphan_explicit_goal_ids.add(goal_id)
                            self._lease_expired_goal_ids.add(goal_id)
                            self._cancel_requested_goal_ids.add(goal_id)
                            cancel_ids.append(goal_id)
                    if goal_id in self._explicit_goal_ids:
                        if ambiguous:
                            # Duplicate entries for one GoalID cannot prove a
                            # terminal transition even if one item says DONE.
                            # Retain the slot and cancel until a later unique,
                            # trusted terminal status resolves the ambiguity.
                            self._active_explicit_goal_ids.add(goal_id)
                            self._lease_revoked_goal_ids.add(goal_id)
                            self._lease_expired_goal_ids.add(goal_id)
                            self._cancel_requested_goal_ids.add(goal_id)
                            cancel_ids.append(goal_id)
                        elif status in terminal:
                            self._active_explicit_goal_ids.discard(goal_id)
                            self._orphan_explicit_goal_ids.discard(goal_id)
                            self._lease_expired_goal_ids.discard(goal_id)
                            self._lease_cancel_last_wall.pop(goal_id, None)
                            self._ai_goal_lease_wall.pop(goal_id, None)
                        else:
                            self._active_explicit_goal_ids.add(goal_id)
                elif self.SIMPLE_GOAL_ID_RE.fullmatch(goal_id) is not None:
                    issued_here = goal_id in self._issued_simple_goal_ids
                    if not issued_here:
                        # The namespace is reserved to this bridge, but after a
                        # restart there is no retained target/owner proof.  Do
                        # not adopt or replay it: quarantine a nonterminal (or
                        # ambiguous) old ID and drive only that ID to terminal.
                        if (goal_id in self._orphan_ordinary_goal_ids and
                                not ambiguous and status in terminal):
                            self._orphan_ordinary_goal_ids.discard(goal_id)
                            self._active_ordinary_goal_ids.discard(goal_id)
                            self._lease_cancel_last_wall.pop(goal_id, None)
                        elif ambiguous or status not in terminal:
                            self._orphan_ordinary_goal_ids.add(goal_id)
                            self._active_ordinary_goal_ids.add(goal_id)
                            self._cancel_requested_goal_ids.add(goal_id)
                            cancel_ids.append(goal_id)
                        continue
                    if ambiguous:
                        self._active_ordinary_goal_ids.add(goal_id)
                        self._cancel_requested_goal_ids.add(goal_id)
                        if goal_id == self.retained_goal_id:
                            self.retained_reissue_allowed = False
                        cancel_ids.append(goal_id)
                    elif status in terminal:
                        self._active_ordinary_goal_ids.discard(goal_id)
                        self._orphan_ordinary_goal_ids.discard(goal_id)
                        if (goal_id == self.retained_goal_id and
                                not self.paused):
                            self.retained_reissue_allowed = False
                    else:
                        self._active_ordinary_goal_ids.add(goal_id)
                        if (self.coverage_active and
                                goal_id not in self._cancel_requested_goal_ids):
                            self._cancel_requested_goal_ids.add(goal_id)
                            cancel_ids.append(goal_id)
        for goal_id in sorted(set(cancel_ids)):
            self._publish_cancel_goal_id(goal_id)
        self._publish_status("move_base action status changed")

    def _cancel_callback(self, message):
        goal_id = str(message.id)
        if self.AI_GOAL_ID_RE.fullmatch(goal_id) is None:
            return
        with self.lock:
            self._cancel_requested_goal_ids.add(goal_id)
            self._lease_revoked_goal_ids.add(goal_id)
            if goal_id in self._active_explicit_goal_ids:
                self._lease_expired_goal_ids.add(goal_id)
        self._publish_status("explicit AI goal cancellation observed")

    def _action_cancel_request_callback(self, message):
        goal_id = str(message.id)
        stamp = self._stamp_tuple(message.stamp)
        if (self.AI_GOAL_ID_RE.fullmatch(goal_id) is None or
                stamp != (0, 0)):
            self._publish_status("AI cancel request rejected: invalid exact GoalID")
            return
        not_forwarded_ack = False
        with self.lock:
            # Record before local publication.  If this cancel request races
            # ahead of the action request on another TCPROS connection, the
            # later action request is rejected and actionlib also remembers the
            # exact cancel independently.
            self._cancel_requested_goal_ids.add(goal_id)
            self._lease_revoked_goal_ids.add(goal_id)
            if goal_id in self._active_explicit_goal_ids:
                self._lease_expired_goal_ids.add(goal_id)
            self._cancel_tombstone_goal_ids.add(goal_id)
            # A status snapshot, including an empty one, cannot prove an
            # unknown ID was never forwarded by a bridge process that died.
            # Synthesize not_forwarded only after this process has actually
            # seen and rejected the matching action request under this lock.
            not_forwarded_ack = self._publish_not_forwarded_ack_locked(goal_id)
        self._publish_cancel_goal_id(goal_id)
        self._publish_status(
            "AI canceled request proven not forwarded"
            if not_forwarded_ack else
            "AI exact cancel request forwarded locally"
        )

    def _ai_heartbeat_callback(self, message):
        goal_id = str(getattr(message, "id", ""))
        stamp = self._stamp_tuple(getattr(message, "stamp", None))
        renewed = False
        if (self.AI_GOAL_ID_RE.fullmatch(goal_id) is None or
                stamp != (0, 0)):
            self._publish_status("AI heartbeat rejected: invalid exact GoalID")
            return
        with self.lock:
            if (goal_id in self._active_explicit_goal_ids and
                    goal_id not in self._orphan_explicit_goal_ids and
                    goal_id not in self._lease_revoked_goal_ids and
                    goal_id not in self._lease_expired_goal_ids):
                self._ai_goal_lease_wall[goal_id] = time.monotonic()
                self.last_ai_heartbeat_goal_id = goal_id
                renewed = True
        if not renewed:
            self._publish_status(
                "AI heartbeat ignored: GoalID is not a renewable local lease")

    def _ai_lease_watchdog(self, _event):
        cancel_ids = []
        now = time.monotonic()
        with self.lock:
            for goal_id in self._active_explicit_goal_ids:
                lease_wall = self._ai_goal_lease_wall.get(goal_id, 0.0)
                if (goal_id in self._orphan_explicit_goal_ids or
                        goal_id in self._lease_revoked_goal_ids or
                        lease_wall <= 0.0 or
                        now - lease_wall > self.ai_heartbeat_timeout_sec):
                    self._lease_expired_goal_ids.add(goal_id)
                    self._lease_revoked_goal_ids.add(goal_id)
                    self._cancel_requested_goal_ids.add(goal_id)
            retry_ids = self._cancel_requested_goal_ids & (
                self._active_explicit_goal_ids |
                self._active_ordinary_goal_ids
            )
            for goal_id in sorted(retry_ids):
                if now - self._lease_cancel_last_wall.get(goal_id, 0.0) >= 0.5:
                    self._lease_cancel_last_wall[goal_id] = now
                    self._cancel_requested_goal_ids.add(goal_id)
                    cancel_ids.append(goal_id)
        for goal_id in cancel_ids:
            self._publish_cancel_goal_id(goal_id)
        if cancel_ids:
            self._publish_status(
                "AI exact GoalID lease expired or orphaned; cancellation reasserted")

    def _submit_simple_action_locked(self, message):
        """Actionize one Qt/RViz pose and publish it inside the ownership lock."""
        if len(self._issued_simple_goal_ids) >= self.MAX_TRACKED_SIMPLE_GOALS:
            self._simple_goal_tracking_saturated = True
            return ""
        action = MoveBaseActionGoal()
        stamp = rospy.Time.now()
        goal_id = "sweeper-simple-{}".format(uuid.uuid4().hex)
        action.header.stamp = stamp
        action.goal_id.stamp = stamp
        action.goal_id.id = goal_id
        action.goal.target_pose = copy.deepcopy(message)
        action.goal.target_pose.header.stamp = stamp
        self.retained_pose = copy.deepcopy(action.goal.target_pose)
        self.retained_goal_id = goal_id
        self.retained_reissue_allowed = True
        self._issued_simple_goal_ids.add(goal_id)
        self._active_ordinary_goal_ids.add(goal_id)
        # Publishing under the same lock as coverage claim establishes the two
        # possible orders: claim first rejects this request; request first makes
        # this exact ID visible for claim's token-scoped cancellation.
        self.action_goal_pub.publish(action)
        return goal_id

    def _simple_goal_callback(self, message):
        with self.lock:
            if not self.action_status_received:
                reason = (
                    "simple navigation goal rejected: move_base action status "
                    "is not ready"
                )
            elif not self.coverage_state_received:
                reason = "simple navigation goal rejected: coverage state is not ready"
            elif self.coverage_active:
                reason = "simple navigation goal rejected: coverage owns move_base"
            elif self.paused:
                reason = "simple navigation goal rejected while navigation is paused"
            else:
                goal_id = self._submit_simple_action_locked(message)
                reason = (
                    "simple navigation action accepted: {}".format(goal_id)
                    if goal_id else
                    "simple navigation goal rejected: tracking cache is saturated"
                )
        self._publish_status(reason)

    def _set_coverage_owner(self, request):
        """Synchronously linearize coverage ownership against AI requests.

        The request token is a mission generation, not an authorization
        secret.  An exact token is nevertheless required on release so a late
        cleanup from an older task can never open the gate for a newer task.
        """
        token = str(getattr(request, "owner_token", ""))
        claim = getattr(request, "claim", None)
        cancel_ids = []
        with self.lock:
            if type(claim) is not bool:
                success = False
                message = "coverage owner request rejected: claim must be boolean"
            elif self.COVERAGE_OWNER_TOKEN_RE.fullmatch(token) is None:
                success = False
                message = "coverage owner request rejected: invalid owner token"
            elif claim and not self.action_status_received:
                success = False
                message = (
                    "coverage owner request rejected: move_base action status "
                    "is not ready"
                )
            elif claim:
                if (self.coverage_owner_token and
                        self.coverage_owner_token != token):
                    success = False
                    message = (
                        "coverage owner request rejected: another generation "
                        "still owns navigation"
                    )
                else:
                    self.coverage_owner_token = token
                    self.coverage_state_received = True
                    self._refresh_coverage_active_locked()
                    cancel_ids = self._activate_coverage_locked()
                    success = True
                    message = "coverage navigation ownership claimed"
            elif not self.coverage_owner_token:
                # This makes compensation after an uncertain acquire
                # idempotent without granting an empty-token wildcard release.
                self._refresh_coverage_active_locked()
                success = True
                message = "coverage navigation ownership is already released"
            elif self.coverage_owner_token != token:
                success = False
                message = (
                    "coverage owner release rejected: owner token does not match"
                )
            else:
                self.coverage_owner_token = ""
                self._refresh_coverage_active_locked()
                success = True
                message = "coverage navigation ownership released"
            claimed = bool(self.coverage_owner_token)
            current_owner_token = self.coverage_owner_token

        # Publish every exact cancellation before returning the synchronous
        # claim response.  If the AI request won the same-node lock first, its
        # action was already forwarded; this path deterministically revokes it.
        # If the claim won first, the AI callback observes coverage_active and
        # rejects without forwarding.
        for goal_id in cancel_ids:
            self._publish_cancel_goal_id(goal_id)
        if success and claim and cancel_ids:
            canceled, pending_ids = self._wait_for_exact_cancellation_terminal(
                cancel_ids)
            if not canceled:
                success = False
                message = (
                    "coverage navigation ownership retained fail-closed; "
                    "previous goals have no trusted terminal status: {}"
                ).format(",".join(pending_ids))
        self._publish_status(message)
        return SetCoverageOwnerResponse(
            success=success,
            claimed=claimed,
            current_owner_token=current_owner_token,
            message=message,
        )

    def _coverage_callback(self, message):
        cancel_ids = []
        with self.lock:
            self.coverage_state_received = True
            self.coverage_topic_active = bool(message.data)
            self._refresh_coverage_active_locked()
            if self.coverage_active:
                cancel_ids = self._activate_coverage_locked()
        for goal_id in cancel_ids:
            self._publish_cancel_goal_id(goal_id)
        self._publish_status("coverage activity changed")

    def _publish_cancel_all(self):
        cancel = GoalID()
        cancel.stamp = rospy.Time()
        cancel.id = ""
        self.cancel_pub.publish(cancel)

    def _publish_cancel_goal_id(self, goal_id):
        cancel = GoalID()
        cancel.stamp = rospy.Time()
        cancel.id = goal_id
        self.cancel_pub.publish(cancel)

    def _set_paused(self, request):
        cancel_ids = []
        with self.lock:
            requested = bool(request.data)
            if requested == self.paused:
                if requested:
                    # Reasserting pause is intentionally idempotent at the
                    # action server, not a no-op at the transport boundary.
                    # It lets safety callers retry after uncertain delivery.
                    cancel_ids = sorted(
                        self._active_explicit_goal_ids |
                        self._active_ordinary_goal_ids
                    )
                    self._cancel_requested_goal_ids.update(cancel_ids)
                    revoked_ai_ids = (
                        set(cancel_ids) & self._active_explicit_goal_ids)
                    self._lease_revoked_goal_ids.update(revoked_ai_ids)
                    self._lease_expired_goal_ids.update(revoked_ai_ids)
                    self._publish_cancel_all()
                    message = "already paused; move_base cancellation reasserted"
                else:
                    return SetBoolResponse(success=True, message="already running")
            else:
                self.paused = requested
                if requested:
                    cancel_ids = sorted(
                        self._active_explicit_goal_ids |
                        self._active_ordinary_goal_ids
                    )
                    self._cancel_requested_goal_ids.update(cancel_ids)
                    revoked_ai_ids = (
                        set(cancel_ids) & self._active_explicit_goal_ids)
                    self._lease_revoked_goal_ids.update(revoked_ai_ids)
                    self._lease_expired_goal_ids.update(revoked_ai_ids)
                    self._publish_cancel_all()
                    message = "move_base goal canceled and retained"
                else:
                    # Coverage owns its segment state and must reissue the exact
                    # enforced swath after a safety pause.  Republishing only the
                    # retained endpoint here would race that state machine and can
                    # briefly produce an unconstrained shortest path.
                    if (self.reissue_on_resume and not self.coverage_active and
                            self.retained_pose is not None and
                            self.retained_reissue_allowed):
                        goal_id = self._submit_simple_action_locked(
                            self.retained_pose)
                        message = (
                            "retained move_base target reissued as exact action {}"
                        ).format(goal_id) if goal_id else (
                            "navigation resumed but retained target was not reissued: "
                            "simple goal tracking cache is saturated"
                        )
                    elif (self.retained_pose is not None and
                          not self.retained_reissue_allowed):
                        message = "navigation resumed; explicit AI target remains canceled"
                    else:
                        message = "navigation resumed without a retained target"
        for goal_id in cancel_ids:
            # An ID-specific cancel is remembered by actionlib even if it is
            # delivered before the corresponding /goal message.  It closes the
            # cross-topic ordering race that cancel-all alone cannot close.
            self._publish_cancel_goal_id(goal_id)
        self._publish_status(message)
        return SetBoolResponse(success=True, message=message)

    def _publish_status(self, reason):
        with self.lock:
            active_lease_ids = sorted(self._active_explicit_goal_ids)
            lease_goal_id = (
                self.last_ai_heartbeat_goal_id
                if self.last_ai_heartbeat_goal_id in self._active_explicit_goal_ids
                else (active_lease_ids[0] if active_lease_ids else "")
            )
            lease_wall = self._ai_goal_lease_wall.get(lease_goal_id, 0.0)
            payload = {
                "paused": self.paused,
                "has_retained_goal": self.retained_pose is not None,
                "retained_goal_id": self.retained_goal_id,
                "retained_reissue_allowed": self.retained_reissue_allowed,
                "reason": reason,
                "reissue_on_resume": self.reissue_on_resume,
                "coverage_active": self.coverage_active,
                "coverage_topic_active": self.coverage_topic_active,
                "coverage_owner_claimed": bool(self.coverage_owner_token),
                "coverage_owner_token": self.coverage_owner_token,
                "coverage_owner_service": self.coverage_owner_service_name,
                "coverage_state_received": self.coverage_state_received,
                "require_coverage_state": self.require_coverage_state,
                "action_request_version": 2,
                "last_action_request_id": self.last_action_request_id,
                "active_explicit_goal_ids": sorted(
                    self._active_explicit_goal_ids),
                "orphan_explicit_goal_ids": sorted(
                    self._orphan_explicit_goal_ids),
                "issued_simple_goal_ids": sorted(
                    self._issued_simple_goal_ids),
                "orphan_ordinary_goal_ids": sorted(
                    self._orphan_ordinary_goal_ids),
                "active_ordinary_goal_ids": sorted(
                    self._active_ordinary_goal_ids),
                "explicit_goal_tracking_saturated": (
                    self._explicit_goal_tracking_saturated),
                "simple_goal_tracking_saturated": (
                    self._simple_goal_tracking_saturated),
                "cancel_requested_goal_ids": sorted(
                    self._cancel_requested_goal_ids),
                "lease_revoked_goal_ids": sorted(
                    self._lease_revoked_goal_ids),
                "lease_expired_goal_ids": sorted(
                    self._lease_expired_goal_ids),
                "action_goal_request_topic": self.action_goal_request_topic,
                "action_goal_output_topic": self.action_goal_topic,
                "action_status_topic": self.action_status_topic,
                "action_status_received": self.action_status_received,
                "ai_heartbeat_topic": self.ai_heartbeat_topic,
                "ai_heartbeat_message_type": "actionlib_msgs/GoalID",
                "ai_heartbeat_goal_id": lease_goal_id,
                "action_cancel_request_topic": self.action_cancel_request_topic,
                "action_cancel_ack_topic": self.action_cancel_ack_topic,
                "ai_heartbeat_age_sec": (
                    None if lease_wall <= 0.0 else
                    max(0.0, time.monotonic() - lease_wall)
                ),
                "ai_heartbeat_timeout_sec": self.ai_heartbeat_timeout_sec,
                "coverage_claim_cancel_timeout_sec": (
                    self.coverage_claim_cancel_timeout_sec),
                "max_action_request_age_sec": self.max_action_request_age_sec,
                "max_action_request_future_sec": self.max_action_request_future_sec,
                "required_action_server_node": self.required_action_server_node,
                "simple_goal_request_topic": self.simple_goal_request_topic,
                "simple_goal_output_topic": self.simple_goal_output_topic,
                "simple_goal_actionized": True,
            }
        self.paused_pub.publish(Bool(data=payload["paused"]))
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("navigation_pause", anonymous=False)
    MoveBasePauseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
