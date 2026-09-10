#!/usr/bin/env python3
"""Isolated pending-litter mission coordinator.

This node never publishes velocity.  It submits one ordinary move_base goal to
the existing ownership bridge, waits for that exact action to succeed at a
stand-off pose, stages the selected map target in the visual controller, and
then asks the existing FOD mode manager for the protected command handoff.
"""

from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Optional

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from autolabor_fod_msgs.srv import (
    SetFodCleaningTarget,
    SetFodCleaningTargetRequest,
    StartPendingLitterCleaning,
    StartPendingLitterCleaningResponse,
)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, UInt64
from std_srvs.srv import SetBool, Trigger, TriggerResponse

from autolabor_fod_control.pending_litter import (
    matches_cleaning_completion, newest_goal_id, stand_off_pose,
)


IDLE = "IDLE"
NAVIGATING = "NAVIGATING"
HANDOFF = "HANDOFF"
VISUAL_SERVO = "VISUAL_SERVO"
COMPLETE = "COMPLETE"
ABORT = "ABORT"


@dataclass(frozen=True)
class Mission:
    generation: int
    object_id: int
    target_x: float
    target_y: float
    observed_at: rospy.Time
    label: str


class PendingLitterCleaner:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.simple_goal_topic = rospy.get_param(
            "~simple_goal_topic", "/move_base_simple/goal"
        )
        self.move_base_status_topic = rospy.get_param(
            "~move_base_status_topic", "/move_base/status"
        )
        self.navigation_status_topic = rospy.get_param(
            "~navigation_status_topic", "/navigation_pause/status"
        )
        self.mode_state_topic = rospy.get_param(
            "~mode_state_topic", "/fod_navigation_mode/state"
        )
        self.visual_target_service = rospy.get_param(
            "~visual_target_service", "/fod_visual_servo/set_cleaning_target"
        )
        self.visual_status_topic = rospy.get_param(
            "~visual_status_topic", "/fod_visual_servo/status"
        )
        self.mode_service = rospy.get_param(
            "~mode_service", "/fod_navigation_mode/set_fod_enabled"
        )
        self.stand_off_m = float(rospy.get_param("~stand_off_m", 1.50))
        self.max_visual_start_distance_m = float(
            rospy.get_param("~max_visual_start_distance_m", 2.20)
        )
        self.max_visual_start_bearing_deg = float(
            rospy.get_param("~max_visual_start_bearing_deg", 35.0)
        )
        self.max_target_distance_m = float(
            rospy.get_param("~max_target_distance_m", 50.0)
        )
        self.tf_timeout_sec = float(rospy.get_param("~tf_timeout_sec", 0.50))
        self.status_timeout_sec = float(
            rospy.get_param("~status_timeout_sec", 1.00)
        )
        self.goal_accept_timeout_sec = float(
            rospy.get_param("~goal_accept_timeout_sec", 2.00)
        )
        self.navigation_timeout_sec = float(
            rospy.get_param("~navigation_timeout_sec", 120.0)
        )
        self.handoff_timeout_sec = float(
            rospy.get_param("~handoff_timeout_sec", 20.0)
        )
        self.visual_timeout_sec = float(
            rospy.get_param("~visual_timeout_sec", 100.0)
        )
        self._validate_parameters()

        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.generation = 0
        self.active: Optional[Mission] = None
        self.state = IDLE
        self.reason = "waiting for a selected pending-litter point"
        self.navigation_status = None
        self.navigation_status_receipt = None
        self.mode_state = ""
        self.mode_state_receipt = None
        self.visual_status = None
        self.visual_status_receipt = None
        self.action_statuses = {}
        self.action_status_receipt = None
        self.goal_id = ""
        self.last_completed_object_id = 0
        self.visual_completion_floor = None
        self.visual_completion_seen = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal_pub = rospy.Publisher(
            self.simple_goal_topic, PoseStamped, queue_size=1
        )
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=2)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.completed_pub = rospy.Publisher(
            "~completed_object_id", UInt64, queue_size=5, latch=False
        )
        self.navigation_status_sub = rospy.Subscriber(
            self.navigation_status_topic,
            String,
            self._navigation_status_cb,
            queue_size=10,
        )
        self.mode_state_sub = rospy.Subscriber(
            self.mode_state_topic, String, self._mode_state_cb, queue_size=10
        )
        self.visual_status_sub = rospy.Subscriber(
            self.visual_status_topic,
            String,
            self._visual_status_cb,
            queue_size=10,
        )
        self.action_status_sub = rospy.Subscriber(
            self.move_base_status_topic,
            GoalStatusArray,
            self._action_status_cb,
            queue_size=10,
        )
        self.start_service = rospy.Service(
            "~start_cleaning",
            StartPendingLitterCleaning,
            self._start_cb,
        )
        self.cancel_service = rospy.Service("~cancel", Trigger, self._cancel_cb)
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._status_tick)
        rospy.on_shutdown(self._shutdown)
        self._publish_status()

    def _validate_parameters(self):
        for name in (
            "map_frame",
            "base_frame",
        ):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError("{} must be a non-empty frame".format(name))
        for name in (
            "simple_goal_topic",
            "move_base_status_topic",
            "navigation_status_topic",
            "mode_state_topic",
            "visual_status_topic",
            "visual_target_service",
            "mode_service",
        ):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError("{} must be an absolute ROS name".format(name))
        numeric = (
            self.stand_off_m,
            self.max_visual_start_distance_m,
            self.max_visual_start_bearing_deg,
            self.max_target_distance_m,
            self.tf_timeout_sec,
            self.status_timeout_sec,
            self.goal_accept_timeout_sec,
            self.navigation_timeout_sec,
            self.handoff_timeout_sec,
            self.visual_timeout_sec,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError("pending-litter timing and geometry must be positive")
        if not 0.80 <= self.stand_off_m <= 3.0:
            raise ValueError("stand_off_m must be between 0.80 and 3.0")
        if self.max_visual_start_distance_m < self.stand_off_m:
            raise ValueError("visual start distance must include the stand-off pose")
        if self.max_visual_start_bearing_deg > 45.0:
            raise ValueError("visual start bearing exceeds the camera capture envelope")

    def _navigation_status_cb(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
        except (TypeError, ValueError):
            return
        with self.lock:
            self.navigation_status = payload
            self.navigation_status_receipt = time.monotonic()

    def _mode_state_cb(self, message):
        with self.lock:
            self.mode_state = str(message.data)
            self.mode_state_receipt = time.monotonic()

    def _visual_status_cb(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("not an object")
        except (TypeError, ValueError):
            return
        with self.lock:
            self.visual_status = payload
            self.visual_status_receipt = time.monotonic()
            mission = self.active
            if mission is not None and matches_cleaning_completion(
                payload.get("last_cleaning_completion"), self.visual_completion_floor,
                mission.object_id, mission.target_x, mission.target_y,
            ):
                self.visual_completion_seen = True

    def _action_status_cb(self, message):
        statuses = {}
        for item in message.status_list:
            statuses.setdefault(str(item.goal_id.id), []).append(int(item.status))
        with self.lock:
            self.action_statuses = statuses
            self.action_status_receipt = time.monotonic()

    def _fresh(self, receipt):
        return receipt is not None and time.monotonic() - receipt <= self.status_timeout_sec

    def _preflight_locked(self):
        if self.active is not None:
            return "another pending-litter mission is already active"
        if not self._fresh(self.navigation_status_receipt):
            return "navigation ownership status is missing or stale"
        status = self.navigation_status
        if status.get("paused"):
            return "ordinary navigation is paused"
        if status.get("coverage_active") or status.get("coverage_owner_claimed"):
            return "coverage cleaning owns move_base"
        if status.get("active_explicit_goal_ids"):
            return "an AI navigation goal is active"
        if status.get("active_ordinary_goal_ids"):
            return "another ordinary navigation goal is active"
        if not status.get("action_status_received"):
            return "move_base action status is not ready"
        if not self._fresh(self.mode_state_receipt) or self.mode_state != "GPS_ACTIVE":
            return "FOD command arbiter is not in GPS_ACTIVE"
        if not self._fresh(self.visual_status_receipt):
            return "visual-servo status is missing or stale"
        if not self.visual_status.get("allow_motion"):
            return "FOD visual motion authorization is disabled"
        if self.visual_status.get("external_estop_override"):
            return "pending-litter cleaning refuses external-estop override"
        if self.visual_status.get("state") != "DISABLED":
            return "visual servo is not in DISABLED standby"
        if self.goal_pub.get_num_connections() != 1:
            return "move_base simple-goal bridge is not the unique subscriber"
        return ""

    @staticmethod
    def _yaw_from_quaternion(q):
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("map pose quaternion is non-finite")
        norm = math.sqrt(sum(float(value) * float(value) for value in values))
        if abs(norm - 1.0) > 0.01:
            raise ValueError("map pose quaternion is not normalized")
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout_sec),
            )
        except Exception as exc:
            raise RuntimeError("map-to-base transform unavailable: {}".format(exc))
        stamp = transform.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if not math.isfinite(stamp) or stamp <= 0.0 or age < -0.20 or age > self.tf_timeout_sec:
            raise RuntimeError("map-to-base transform is stale or invalid")
        translation = transform.transform.translation
        if not all(math.isfinite(value) for value in (translation.x, translation.y)):
            raise RuntimeError("map-to-base translation is non-finite")
        return (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )

    def _start_cb(self, request):
        try:
            object_id = int(request.object_id)
            target_x = float(request.target_position.x)
            target_y = float(request.target_position.y)
            label = str(request.label).strip()
            if object_id <= 0:
                raise ValueError("object_id must be positive")
            if request.target_frame != self.map_frame:
                raise ValueError("selected target must be in map frame")
            if not all(math.isfinite(value) for value in (target_x, target_y)):
                raise ValueError("selected target coordinate is non-finite")
            if request.observed_at.is_zero():
                raise ValueError("selected target has no observation timestamp")
            if not label or len(label) > 32:
                raise ValueError("selected target label is invalid")
            robot_x, robot_y, _robot_yaw = self._map_pose()
            distance = math.hypot(target_x - robot_x, target_y - robot_y)
            if distance > self.max_target_distance_m:
                raise ValueError(
                    "selected target is %.2fm away, above %.2fm limit"
                    % (distance, self.max_target_distance_m)
                )
        except (TypeError, ValueError, RuntimeError) as exc:
            return StartPendingLitterCleaningResponse(False, str(exc))

        with self.lock:
            error = self._preflight_locked()
            if error:
                return StartPendingLitterCleaningResponse(False, error)
            self.generation += 1
            mission = Mission(
                self.generation,
                object_id,
                target_x,
                target_y,
                request.observed_at,
                label,
            )
            self.active = mission
            self.goal_id = ""
            self.visual_completion_floor = None
            self.visual_completion_seen = False
            self.cancel_event.clear()
            self.state = HANDOFF if distance <= self.stand_off_m else NAVIGATING
            self.reason = "pending-litter mission accepted"
        threading.Thread(
            target=self._run_mission,
            args=(mission,),
            name="pending-litter-cleaning-{}".format(mission.generation),
            daemon=True,
        ).start()
        self._publish_status()
        return StartPendingLitterCleaningResponse(
            True,
            "mission accepted for pending litter {} at map ({:.3f},{:.3f})".format(
                label, target_x, target_y
            ),
        )

    def _still_active(self, mission):
        with self.lock:
            return self.active == mission and not self.cancel_event.is_set()

    def _set_state(self, mission, state, reason):
        with self.lock:
            if self.active != mission:
                return False
            self.state = state
            self.reason = str(reason)
        self._publish_status()
        return True

    def _wait(self, mission, predicate, timeout, failure):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if not self._still_active(mission):
                raise RuntimeError("mission canceled")
            result = predicate()
            if result is not None:
                return result
            self.cancel_event.wait(0.05)
        raise RuntimeError(failure)

    def _submit_stand_off_goal(self, mission):
        if not self._still_active(mission):
            raise RuntimeError("mission canceled before stand-off submission")
        robot_x, robot_y, _robot_yaw = self._map_pose()
        goal_x, goal_y, goal_yaw, _distance = stand_off_pose(
            robot_x,
            robot_y,
            mission.target_x,
            mission.target_y,
            self.stand_off_m,
        )
        with self.lock:
            previous_ids = tuple(self.navigation_status.get("issued_simple_goal_ids", []))
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.pose.position.x = goal_x
        message.pose.position.y = goal_y
        message.pose.orientation.z = math.sin(0.5 * goal_yaw)
        message.pose.orientation.w = math.cos(0.5 * goal_yaw)
        self.goal_pub.publish(message)

        # Resolve the exact ID even if cancellation arrives in the tiny window
        # between PoseStamped publication and the bridge's status update. That
        # ID must be canceled explicitly; a broad move_base cancel is forbidden.
        deadline = time.monotonic() + self.goal_accept_timeout_sec
        goal_id = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if not self._fresh(self.navigation_status_receipt):
                    current = tuple()
                else:
                    current = tuple(
                        self.navigation_status.get("issued_simple_goal_ids", [])
                    )
            goal_id = newest_goal_id(previous_ids, current)
            if goal_id is not None:
                break
            self.cancel_event.wait(0.05)
        if goal_id is None:
            raise RuntimeError(
                "move_base ownership bridge did not accept exactly one new goal"
            )
        with self.lock:
            self.goal_id = goal_id
        if self.cancel_event.is_set() or not self._still_active(mission):
            self._cancel_exact_goal(goal_id)
            raise RuntimeError("mission canceled after move_base goal acceptance")
        self._set_state(
            mission,
            NAVIGATING,
            "moving to {:.2f}m stand-off pose with exact goal {}".format(
                self.stand_off_m, goal_id
            ),
        )

        terminal_failure = {
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }

        def finished():
            with self.lock:
                statuses = tuple(self.action_statuses.get(goal_id, ()))
                fresh = self._fresh(self.action_status_receipt)
            if not fresh or len(statuses) != 1:
                return None
            status = statuses[0]
            if status == GoalStatus.SUCCEEDED:
                return True
            if status in terminal_failure:
                raise RuntimeError(
                    "stand-off move_base goal ended with status {}".format(status)
                )
            return None

        self._wait(
            mission,
            finished,
            self.navigation_timeout_sec,
            "stand-off navigation exceeded its time limit",
        )

        # The bridge disables retained-target replay only after it consumes the
        # terminal action status. Wait for that ownership update before the FOD
        # manager pauses/resumes navigation, closing a cross-subscriber race
        # that could otherwise reissue the already-completed stand-off goal.
        def bridge_released():
            with self.lock:
                if not self._fresh(self.navigation_status_receipt):
                    return None
                status = dict(self.navigation_status)
            active = set(status.get("active_ordinary_goal_ids", []))
            if goal_id in active:
                return None
            if status.get("retained_goal_id") != goal_id:
                raise RuntimeError("navigation bridge lost the exact stand-off goal")
            if status.get("retained_reissue_allowed"):
                return None
            return True

        self._wait(
            mission,
            bridge_released,
            self.goal_accept_timeout_sec,
            "navigation bridge did not retire stand-off replay ownership",
        )

    def _stage_visual_target(self, mission):
        if not self._still_active(mission):
            raise RuntimeError("mission canceled before visual-target staging")
        with self.lock:
            if not self._fresh(self.visual_status_receipt):
                raise RuntimeError("visual status became stale before target staging")
            self.visual_completion_floor = int(
                self.visual_status.get("last_cleaning_completion", {}).get("sequence", 0)
            )
            self.visual_completion_seen = False
        robot_x, robot_y, robot_yaw = self._map_pose()
        dx = mission.target_x - robot_x
        dy = mission.target_y - robot_y
        distance = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(
            -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy,
            math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy,
        ))
        if distance > self.max_visual_start_distance_m:
            raise RuntimeError(
                "stand-off finished but target remains %.2fm away" % distance
            )
        if abs(bearing) > self.max_visual_start_bearing_deg:
            raise RuntimeError(
                "stand-off heading misses target by %.1fdeg" % bearing
            )
        rospy.wait_for_service(self.visual_target_service, timeout=2.0)
        request = SetFodCleaningTargetRequest()
        request.clear = False
        request.object_id = mission.object_id
        request.target_position.x = mission.target_x
        request.target_position.y = mission.target_y
        request.target_position.z = 0.0
        request.target_frame = self.map_frame
        request.observed_at = mission.observed_at
        request.label = mission.label
        response = rospy.ServiceProxy(
            self.visual_target_service, SetFodCleaningTarget
        )(request)
        if not response.success:
            raise RuntimeError("visual target rejected: {}".format(response.message))

    def _enable_visual_mode(self, mission):
        if not self._still_active(mission):
            raise RuntimeError("mission canceled before FOD handoff")
        rospy.wait_for_service(self.mode_service, timeout=2.0)
        response = rospy.ServiceProxy(self.mode_service, SetBool)(True)
        if not response.success:
            raise RuntimeError("FOD mode handoff rejected: {}".format(response.message))

        def active():
            with self.lock:
                state = self.mode_state if self._fresh(self.mode_state_receipt) else ""
            if state == "FOD_ACTIVE":
                return True
            if state in ("FOD_ABORTED", "FAULT_STOP"):
                raise RuntimeError("FOD mode entered {}".format(state))
            return None

        self._wait(
            mission,
            active,
            self.handoff_timeout_sec,
            "FOD mode did not enter FOD_ACTIVE after handoff",
        )

    def _wait_visual_complete(self, mission):
        returned_without_evidence_at = None

        def completed():
            nonlocal returned_without_evidence_at
            with self.lock:
                state = self.mode_state if self._fresh(self.mode_state_receipt) else ""
                confirmed = self.visual_completion_seen
            if state == "GPS_ACTIVE" and confirmed:
                return True
            if state == "GPS_ACTIVE":
                if returned_without_evidence_at is None:
                    returned_without_evidence_at = time.monotonic()
                elif time.monotonic() - returned_without_evidence_at > self.goal_accept_timeout_sec:
                    raise RuntimeError("visual mode exited without selected-target coverage completion")
            else:
                returned_without_evidence_at = None
            if state in ("FOD_ABORTED", "FAULT_STOP"):
                raise RuntimeError("visual cleaning ended in {}".format(state))
            return None

        self._wait(
            mission,
            completed,
            self.visual_timeout_sec,
            "visual cleaning exceeded its time limit",
        )

    def _clear_visual_target(self):
        try:
            request = SetFodCleaningTargetRequest()
            request.clear = True
            rospy.ServiceProxy(
                self.visual_target_service, SetFodCleaningTarget
            )(request)
        except Exception:
            pass

    def _cancel_exact_goal(self, goal_id):
        if not goal_id:
            return
        cancel = GoalID()
        cancel.id = str(goal_id)
        self.cancel_pub.publish(cancel)

    def _disable_visual_mode(self):
        try:
            rospy.ServiceProxy(self.mode_service, SetBool)(False)
        except Exception:
            pass

    def _run_mission(self, mission):
        try:
            if not self._still_active(mission):
                raise RuntimeError("mission canceled before execution")
            robot_x, robot_y, _robot_yaw = self._map_pose()
            distance = math.hypot(
                mission.target_x - robot_x, mission.target_y - robot_y
            )
            if distance > self.stand_off_m:
                self._submit_stand_off_goal(mission)
            self._set_state(
                mission,
                HANDOFF,
                "stand-off reached; validating selected target for visual servo",
            )
            self._stage_visual_target(mission)
            self._enable_visual_mode(mission)
            self._set_state(
                mission,
                VISUAL_SERVO,
                "visual servo owns final approach; completion requires a forward 0.10m coordinate hit",
            )
            self._wait_visual_complete(mission)
            if not self._still_active(mission):
                raise RuntimeError("mission canceled before completion commit")
            with self.lock:
                if self.active != mission:
                    return
                if self.cancel_event.is_set():
                    raise RuntimeError("mission canceled at completion commit")
                self.state = COMPLETE
                self.reason = "selected litter was covered by the forward cleaning trajectory"
                self.last_completed_object_id = mission.object_id
                self.active = None
                self.goal_id = ""
            self.completed_pub.publish(UInt64(data=mission.object_id))
            self._publish_status()
        except Exception as exc:
            with self.lock:
                goal_id = self.goal_id if self.active == mission else ""
            self._cancel_exact_goal(goal_id)
            self._disable_visual_mode()
            self._clear_visual_target()
            with self.lock:
                if self.active != mission:
                    return
                self.state = ABORT
                self.reason = str(exc)
                self.active = None
                self.goal_id = ""
            self._publish_status()

    def _cancel_cb(self, _request):
        with self.lock:
            mission = self.active
            goal_id = self.goal_id
            if mission is None:
                return TriggerResponse(True, "no pending-litter mission is active")
            self.cancel_event.set()
            self.state = ABORT
            self.reason = "pending-litter cancellation requested; retiring exact ownership"
        if goal_id:
            self._cancel_exact_goal(goal_id)
        self._disable_visual_mode()
        self._clear_visual_target()
        self._publish_status()
        return TriggerResponse(True, "pending-litter mission cancellation requested")

    def _status_tick(self, _event):
        self._publish_status()

    def _publish_status(self):
        with self.lock:
            mission = self.active
            payload = {
                "state": self.state,
                "reason": self.reason,
                "active": mission is not None,
                "object_id": mission.object_id if mission is not None else 0,
                "label": mission.label if mission is not None else "",
                "target_frame": self.map_frame if mission is not None else "",
                "target_x": mission.target_x if mission is not None else None,
                "target_y": mission.target_y if mission is not None else None,
                "goal_id": self.goal_id,
                "stand_off_m": self.stand_off_m,
                "last_completed_object_id": self.last_completed_object_id,
                "last_completed_object_id_text": str(self.last_completed_object_id),
                "publishes_velocity": False,
            }
            state = self.state
        self.state_pub.publish(String(data=state))
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _shutdown(self):
        self.cancel_event.set()


def main():
    rospy.init_node("pending_litter_cleaner", anonymous=False)
    PendingLitterCleaner()
    rospy.spin()


if __name__ == "__main__":
    main()
