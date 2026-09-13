#!/usr/bin/env python3
"""Drive long GPS routes through bounded rolling move_base action goals."""

from collections import OrderedDict
import copy
import json
import math
import threading

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

from gps_module.long_range import (
    RollingGoalRoute,
    gps_to_xy,
    make_route_goal_id,
    rotate_xy,
    validate_latitude_longitude,
    validate_route_distances,
)


TERMINAL_FAILURE_STATES = frozenset(
    (
        GoalStatus.PREEMPTED,
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    )
)


def time_is_fresh(now, sample_time, timeout):
    """Return whether a local callback receipt time is fresh."""
    if sample_time is None:
        return False
    age = (now - sample_time).to_sec()
    return 0.0 <= age <= timeout


def cancel_matches_goal(cancel, goal_id, goal_stamp):
    """Apply actionlib GoalID cancellation semantics to one managed goal."""
    if not goal_id:
        return False
    if cancel.id:
        return cancel.id == goal_id
    if cancel.stamp.secs == 0 and cancel.stamp.nsecs == 0:
        return True
    return goal_stamp is not None and goal_stamp <= cancel.stamp


class GpsLongRangeGoalManager:
    """Convert one final WGS84 point into local rolling action goals."""

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "camera_init").lstrip("/")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.goal_fix_topic = rospy.get_param("~goal_fix_topic", "/gps/goal_fix")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.action_goal_topic = rospy.get_param(
            "~action_goal_topic", "/move_base/goal"
        )
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.move_base_status_topic = rospy.get_param(
            "~move_base_status_topic", "/move_base/status"
        )
        self.final_goal_topic = rospy.get_param(
            "~final_goal_topic", "/gps/long_range/final_goal"
        )
        self.subgoal_topic = rospy.get_param(
            "~subgoal_topic", "/gps/long_range/subgoal"
        )
        self.route_status_topic = rospy.get_param(
            "~route_status_topic", "/gps/long_range/status"
        )
        self.route_active_topic = rospy.get_param(
            "~route_active_topic", "/gps/long_range/active"
        )
        self.pause_service_name = rospy.get_param(
            "~pause_service", "/gps/long_range/set_paused"
        )

        self.lookahead_distance = float(
            rospy.get_param("~lookahead_distance", 15.0)
        )
        self.advance_distance = float(
            rospy.get_param("~advance_distance", 5.0)
        )
        self.max_lookahead_distance = float(
            rospy.get_param("~max_lookahead_distance", 18.0)
        )
        self.max_final_goal_distance = float(
            rospy.get_param("~max_final_goal_distance", 1000.0)
        )
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.move_base_status_timeout = float(
            rospy.get_param("~move_base_status_timeout", 2.0)
        )
        self.update_rate = float(rospy.get_param("~update_rate", 10.0))
        self.yaw_offset = float(rospy.get_param("~yaw_offset", 0.0))
        self.yaw_offset += math.radians(
            float(rospy.get_param("~yaw_offset_deg", 0.0))
        )

        (
            self.lookahead_distance,
            self.advance_distance,
        ) = validate_route_distances(
            self.lookahead_distance,
            self.advance_distance,
        )
        positive_parameters = (
            self.max_lookahead_distance,
            self.max_final_goal_distance,
            self.odom_timeout,
            self.move_base_status_timeout,
            self.update_rate,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_parameters):
            raise ValueError(
                "distance limits, timeouts and update_rate must be finite and positive"
            )
        if self.lookahead_distance > self.max_lookahead_distance:
            raise ValueError(
                "lookahead_distance must not exceed max_lookahead_distance"
            )
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if (
            not isinstance(self.pause_service_name, str)
            or not self.pause_service_name.startswith("/")
        ):
            raise ValueError("pause_service must be an absolute ROS service name")
        if not math.isfinite(self.yaw_offset):
            raise ValueError("yaw offset must be finite")

        self.origin_lat = self._optional_float_param("~origin_lat")
        self.origin_lon = self._optional_float_param("~origin_lon")
        if self.origin_lat is not None or self.origin_lon is not None:
            if self.origin_lat is None or self.origin_lon is None:
                raise ValueError("origin_lat and origin_lon must be provided together")
            self.origin_lat, self.origin_lon = validate_latitude_longitude(
                self.origin_lat, self.origin_lon
            )

        self.lock = threading.RLock()
        self.latest_odom = None
        self.latest_odom_receipt = None
        self.latest_move_base_status_receipt = None
        self.route = None
        self.route_token = None
        self.route_counter = 0
        self.route_active = False
        self.paused = False
        self.route_state = "IDLE"
        self.route_reason = "waiting for a final GPS goal"
        self.final_latitude = None
        self.final_longitude = None
        self.current_goal_id = None
        self.current_goal_stamp = None
        self.current_goal_is_final = False
        self.advance_requested = False
        self.owned_goal_ids = OrderedDict()

        self.action_goal_pub = rospy.Publisher(
            self.action_goal_topic,
            MoveBaseActionGoal,
            queue_size=5,
        )
        self.cancel_pub = rospy.Publisher(
            self.cancel_topic,
            GoalID,
            queue_size=5,
        )
        self.final_goal_pub = rospy.Publisher(
            self.final_goal_topic,
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.subgoal_pub = rospy.Publisher(
            self.subgoal_topic,
            PoseStamped,
            queue_size=1,
            latch=True,
        )
        self.route_status_pub = rospy.Publisher(
            self.route_status_topic,
            String,
            queue_size=1,
            latch=True,
        )
        self.route_active_pub = rospy.Publisher(
            self.route_active_topic,
            Bool,
            queue_size=1,
            latch=True,
        )

        self.fix_sub = rospy.Subscriber(
            self.fix_topic,
            NavSatFix,
            self.fix_callback,
            queue_size=5,
        )
        self.goal_fix_sub = rospy.Subscriber(
            self.goal_fix_topic,
            NavSatFix,
            self.goal_fix_callback,
            queue_size=5,
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20,
        )
        self.action_goal_sub = rospy.Subscriber(
            self.action_goal_topic,
            MoveBaseActionGoal,
            self.action_goal_callback,
            queue_size=10,
        )
        self.cancel_sub = rospy.Subscriber(
            self.cancel_topic,
            GoalID,
            self.cancel_callback,
            queue_size=10,
        )
        self.move_base_status_sub = rospy.Subscriber(
            self.move_base_status_topic,
            GoalStatusArray,
            self.move_base_status_callback,
            queue_size=10,
        )
        self.pause_service = rospy.Service(
            self.pause_service_name,
            SetBool,
            self.set_paused_callback,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.update_rate),
            self.timer_callback,
        )
        rospy.on_shutdown(self.shutdown)

        with self.lock:
            self._publish_route_status_locked()
        rospy.loginfo(
            "GPS long-range goal manager: final %s -> rolling %s; "
            "lookahead=%.1f m advance=%.1f m max_final=%.1f m",
            self.goal_fix_topic,
            self.action_goal_topic,
            self.lookahead_distance,
            self.advance_distance,
            self.max_final_goal_distance,
        )

    @staticmethod
    def _optional_float_param(name):
        value = rospy.get_param(name, None)
        if value in (None, ""):
            return None
        return float(value)

    def _refresh_origin_from_shared_params(self):
        shared_lat = rospy.get_param("/gps/origin_lat", None)
        shared_lon = rospy.get_param("/gps/origin_lon", None)
        if shared_lat is None or shared_lon is None:
            return False
        try:
            shared_lat, shared_lon = validate_latitude_longitude(
                shared_lat, shared_lon
            )
        except ValueError:
            return False
        with self.lock:
            self.origin_lat = shared_lat
            self.origin_lon = shared_lon
        return True

    def fix_callback(self, msg):
        with self.lock:
            origin_known = (
                self.origin_lat is not None and self.origin_lon is not None
            )
        if origin_known or self._refresh_origin_from_shared_params():
            return
        try:
            latitude, longitude = validate_latitude_longitude(
                msg.latitude, msg.longitude
            )
        except ValueError:
            return
        with self.lock:
            if self.origin_lat is None and self.origin_lon is None:
                self.origin_lat = latitude
                self.origin_lon = longitude
                rospy.loginfo(
                    "Long-range manager adopted GPS origin lat=%.8f lon=%.8f",
                    latitude,
                    longitude,
                )

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = copy.deepcopy(msg)
            self.latest_odom_receipt = rospy.Time.now()

    def _odom_xy_locked(self, now):
        if not time_is_fresh(
            now,
            self.latest_odom_receipt,
            self.odom_timeout,
        ):
            raise ValueError("GPS odometry is missing or stale")
        frame_id = self.latest_odom.header.frame_id.lstrip("/")
        if frame_id and frame_id != self.frame_id:
            raise ValueError(
                "GPS odometry frame {} differs from {}".format(
                    frame_id, self.frame_id
                )
            )
        x = self.latest_odom.pose.pose.position.x
        y = self.latest_odom.pose.pose.position.y
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("GPS odometry position is non-finite")
        return x, y

    def _move_base_is_fresh_locked(self, now):
        return time_is_fresh(
            now,
            self.latest_move_base_status_receipt,
            self.move_base_status_timeout,
        )

    def _pose(self, x, y, current_x, current_y, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = math.atan2(y - current_y, x - current_x)
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _remember_owned_goal_locked(self, identifier):
        self.owned_goal_ids[identifier] = None
        self.owned_goal_ids.move_to_end(identifier)
        while len(self.owned_goal_ids) > 200:
            self.owned_goal_ids.popitem(last=False)

    def _send_next_segment_locked(self, current_x, current_y, stamp):
        segment = self.route.next_segment(current_x, current_y)
        goal_id = make_route_goal_id(
            self.route_token,
            segment.index,
            segment.is_final,
        )
        pose = self._pose(
            segment.x,
            segment.y,
            current_x,
            current_y,
            stamp,
        )
        action_goal = MoveBaseActionGoal()
        action_goal.header.stamp = stamp
        action_goal.goal_id.stamp = stamp
        action_goal.goal_id.id = goal_id
        action_goal.goal.target_pose = copy.deepcopy(pose)

        self.current_goal_id = goal_id
        self.current_goal_stamp = stamp
        self.current_goal_is_final = segment.is_final
        self.advance_requested = False
        self.route_state = "ACTIVE_FINAL" if segment.is_final else "ACTIVE_INTERMEDIATE"
        self.route_reason = (
            "final goal is inside rolling horizon"
            if segment.is_final
            else "tracking rolling subgoal"
        )
        self._remember_owned_goal_locked(goal_id)

        # State and identity are committed before publishing the action goal,
        # so callbacks for our own message cannot mistake it for a foreign goal.
        self.subgoal_pub.publish(pose)
        self._publish_route_status_locked(
            current_x=current_x,
            current_y=current_y,
        )
        self.action_goal_pub.publish(action_goal)
        rospy.loginfo(
            "GPS route segment %d (%s): x=%.3f y=%.3f, "
            "final remaining=%.2f m, id=%s",
            segment.index,
            "final" if segment.is_final else "intermediate",
            segment.x,
            segment.y,
            segment.final_distance,
            goal_id,
        )

    def _publish_route_status_locked(self, current_x=None, current_y=None):
        payload = {
            "active": self.route_active,
            "paused": self.paused,
            "state": self.route_state,
            "reason": self.route_reason,
            "lookahead_distance_m": self.lookahead_distance,
            "advance_distance_m": self.advance_distance,
            "goal_id": self.current_goal_id or "",
            "segment_is_final": self.current_goal_is_final,
        }
        if self.route is not None:
            payload["final_x"] = self.route.final_x
            payload["final_y"] = self.route.final_y
            payload["segment_index"] = self.route.segment_index
            if self.route.current_segment is not None:
                payload["subgoal_x"] = self.route.current_segment.x
                payload["subgoal_y"] = self.route.current_segment.y
            if current_x is not None and current_y is not None:
                payload["final_distance_m"] = self.route.distance_to_final(
                    current_x, current_y
                )
                payload["subgoal_distance_m"] = self.route.distance_to_segment(
                    current_x, current_y
                )
        if self.final_latitude is not None and self.final_longitude is not None:
            payload["final_latitude"] = self.final_latitude
            payload["final_longitude"] = self.final_longitude
        self.route_status_pub.publish(
            String(data=json.dumps(payload, sort_keys=True))
        )
        self.route_active_pub.publish(Bool(data=self.route_active))

    def _deactivate_route_locked(self, state, reason, current_x=None, current_y=None):
        self.route_active = False
        self.route_state = state
        self.route_reason = reason
        self.advance_requested = False
        self._publish_route_status_locked(
            current_x=current_x,
            current_y=current_y,
        )

    def _cancel_message_locked(self):
        if not self.current_goal_id:
            return None
        cancel = GoalID()
        cancel.id = self.current_goal_id
        cancel.stamp = rospy.Time()
        return cancel

    def _reject_new_goal(self, reason):
        rospy.logerr("Rejected final GPS goal: %s", reason)
        with self.lock:
            payload = {
                "active": self.route_active,
                "state": "REJECTED_NEW_GOAL",
                "reason": reason,
                "retained_route_state": self.route_state,
            }
            self.route_status_pub.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )

    def set_paused_callback(self, request):
        """Pause move_base while retaining the final GPS route."""
        cancel = None
        if request.data:
            with self.lock:
                if self.paused:
                    return SetBoolResponse(
                        success=True,
                        message="GPS long-range manager is already paused",
                    )
                self.paused = True
                cancel = self._cancel_message_locked()
                if self.route_active:
                    self.route_state = "PAUSED"
                    self.route_reason = (
                        "route retained while another controller owns the chassis"
                    )
                else:
                    self.route_state = "IDLE_PAUSED"
                    self.route_reason = "GPS goal manager paused with no active route"
                self.advance_requested = False
                self._publish_route_status_locked()
            if cancel is not None:
                self.cancel_pub.publish(cancel)
            rospy.logwarn("GPS long-range goal manager paused; final route retained")
            return SetBoolResponse(
                success=True,
                message="GPS navigation paused; final route retained",
            )

        now = rospy.Time.now()
        with self.lock:
            if not self.paused:
                return SetBoolResponse(
                    success=True,
                    message="GPS long-range manager is already active",
                )
            if self.route_active:
                try:
                    current_x, current_y = self._odom_xy_locked(now)
                except ValueError as exc:
                    return SetBoolResponse(
                        success=False,
                        message="cannot resume GPS route: {}".format(exc),
                    )
                if not self._move_base_is_fresh_locked(now):
                    return SetBoolResponse(
                        success=False,
                        message=(
                            "cannot resume GPS route: move_base status is "
                            "missing or stale"
                        ),
                    )
                self.paused = False
                self._send_next_segment_locked(current_x, current_y, now)
            else:
                self.paused = False
                self.route_state = "IDLE"
                self.route_reason = "waiting for a final GPS goal"
                self._publish_route_status_locked()
        rospy.logwarn("GPS long-range goal manager resumed")
        return SetBoolResponse(
            success=True,
            message="GPS navigation resumed from the current vehicle position",
        )

    def goal_fix_callback(self, msg):
        try:
            latitude, longitude = validate_latitude_longitude(
                msg.latitude, msg.longitude
            )
        except ValueError as exc:
            self._reject_new_goal(str(exc))
            return

        with self.lock:
            if self.paused:
                self._reject_new_goal(
                    "GPS manager is paused for FOD recovery; resend after GPS resumes"
                )
                return
            origin_known = (
                self.origin_lat is not None and self.origin_lon is not None
            )
        if not origin_known and not self._refresh_origin_from_shared_params():
            self._reject_new_goal("GPS origin is unavailable; resend after GPS is ready")
            return

        now = rospy.Time.now()
        with self.lock:
            try:
                current_x, current_y = self._odom_xy_locked(now)
            except ValueError as exc:
                self._reject_new_goal(str(exc))
                return
            if not self._move_base_is_fresh_locked(now):
                self._reject_new_goal(
                    "move_base status is missing or stale; resend after bringup is ready"
                )
                return
            origin_lat = self.origin_lat
            origin_lon = self.origin_lon

        try:
            gps_x, gps_y = gps_to_xy(
                latitude,
                longitude,
                origin_lat,
                origin_lon,
            )
            final_x, final_y = rotate_xy(gps_x, gps_y, self.yaw_offset)
        except ValueError as exc:
            self._reject_new_goal(str(exc))
            return

        final_distance = math.hypot(final_x - current_x, final_y - current_y)
        if final_distance > self.max_final_goal_distance:
            self._reject_new_goal(
                "final distance {:.1f} m exceeds configured {:.1f} m".format(
                    final_distance,
                    self.max_final_goal_distance,
                )
            )
            return

        with self.lock:
            self.route_counter += 1
            self.route_token = "{}-{}".format(now.to_nsec(), self.route_counter)
            self.route = RollingGoalRoute(
                final_x,
                final_y,
                lookahead_distance=self.lookahead_distance,
                advance_distance=self.advance_distance,
            )
            self.route_active = True
            self.final_latitude = latitude
            self.final_longitude = longitude
            self.current_goal_id = None
            self.current_goal_stamp = None
            self.current_goal_is_final = False
            self.advance_requested = False

            final_pose = self._pose(
                final_x,
                final_y,
                current_x,
                current_y,
                now,
            )
            self.final_goal_pub.publish(final_pose)
            self._send_next_segment_locked(current_x, current_y, now)

        rospy.loginfo(
            "Accepted final GPS goal lat=%.8f lon=%.8f as x=%.3f y=%.3f "
            "(%.1f m from current pose)",
            latitude,
            longitude,
            final_x,
            final_y,
            final_distance,
        )

    def action_goal_callback(self, msg):
        identifier = msg.goal_id.id
        with self.lock:
            if self.paused:
                return
            if identifier in self.owned_goal_ids:
                return
            if not self.route_active:
                return
            self._deactivate_route_locked(
                "SUPERSEDED",
                "a non-route move_base goal took control",
            )
        rospy.logwarn(
            "GPS long-range route yielded to foreign move_base goal id=%s",
            identifier or "<empty>",
        )

    def cancel_callback(self, msg):
        with self.lock:
            if self.paused:
                return
            if not self.route_active:
                return
            if not cancel_matches_goal(
                msg,
                self.current_goal_id,
                self.current_goal_stamp,
            ):
                return
            self._deactivate_route_locked(
                "CANCELED",
                "current long-range route was canceled",
            )
        rospy.loginfo("GPS long-range route canceled")

    def move_base_status_callback(self, msg):
        now = rospy.Time.now()
        with self.lock:
            self.latest_move_base_status_receipt = now
            if self.paused:
                return
            if not self.route_active or not self.current_goal_id:
                return
            matching = next(
                (
                    status
                    for status in msg.status_list
                    if status.goal_id.id == self.current_goal_id
                ),
                None,
            )
            if matching is None:
                return
            if matching.status == GoalStatus.SUCCEEDED:
                if self.current_goal_is_final:
                    self._deactivate_route_locked(
                        "COMPLETE",
                        "move_base reached the final GPS goal",
                    )
                    rospy.loginfo("GPS long-range route complete")
                else:
                    self.advance_requested = True
                    self.route_state = "ADVANCING"
                    self.route_reason = "intermediate goal completed; selecting next"
                    self._publish_route_status_locked()
                return
            if matching.status in TERMINAL_FAILURE_STATES:
                self._deactivate_route_locked(
                    "ABORTED",
                    "current move_base segment ended with status {}".format(
                        matching.status
                    ),
                )
                rospy.logerr(
                    "GPS long-range route aborted by move_base status %d",
                    matching.status,
                )

    def timer_callback(self, _event):
        cancel = None
        now = rospy.Time.now()
        with self.lock:
            if self.paused:
                return
            if not self.route_active:
                return
            try:
                current_x, current_y = self._odom_xy_locked(now)
            except ValueError as exc:
                cancel = self._cancel_message_locked()
                self._deactivate_route_locked("ABORTED", str(exc))
            else:
                if not self._move_base_is_fresh_locked(now):
                    cancel = self._cancel_message_locked()
                    self._deactivate_route_locked(
                        "ABORTED",
                        "move_base status is missing or stale",
                        current_x,
                        current_y,
                    )
                elif not self.current_goal_is_final:
                    final_inside_horizon = self.route.final_is_within_horizon(
                        current_x,
                        current_y,
                    )
                    if (
                        self.advance_requested
                        or final_inside_horizon
                        or self.route.should_advance(current_x, current_y)
                    ):
                        self._send_next_segment_locked(
                            current_x,
                            current_y,
                            now,
                        )
                    else:
                        self._publish_route_status_locked(
                            current_x=current_x,
                            current_y=current_y,
                        )
                else:
                    self._publish_route_status_locked(
                        current_x=current_x,
                        current_y=current_y,
                    )
        if cancel is not None:
            self.cancel_pub.publish(cancel)
            rospy.logerr("Canceled GPS route because a required runtime input failed")

    def shutdown(self):
        cancel = None
        try:
            with self.lock:
                if self.route_active:
                    cancel = self._cancel_message_locked()
                    self._deactivate_route_locked(
                        "CANCELED",
                        "long-range goal manager is shutting down",
                    )
            if cancel is not None:
                self.cancel_pub.publish(cancel)
        except rospy.ROSException:
            pass


def main():
    rospy.init_node("gps_long_range_goal_manager")
    try:
        GpsLongRangeGoalManager()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid GPS long-range goal configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
