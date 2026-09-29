#!/usr/bin/env python3
"""GPS command safety relay with an optional final-goal forward-speed cap."""

from collections import OrderedDict
import copy
import math
import threading

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry

from gps_module.long_range import (
    is_contiguous_route_goal_transition,
    is_intermediate_route_goal_id,
)


STOP_GOAL_STATES = frozenset(
    (
        GoalStatus.PREEMPTED,
        GoalStatus.SUCCEEDED,
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.PREEMPTING,
        GoalStatus.RECALLING,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    )
)

TRACKING_GOAL_STATES = frozenset((GoalStatus.PENDING, GoalStatus.ACTIVE))


def boolean_parameter(value, name):
    """Parse a ROS boolean parameter without treating the string 'false' as true."""
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError("{} must be a boolean".format(name))


def goal_speed_cap(distance, comfortable_decel, hard_stop_distance, min_approach_speed,
                   approach_time=0.0):
    """Combine braking distance with optional gradual terminal approach."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("distance must be a finite non-negative value")
    if not math.isfinite(comfortable_decel) or comfortable_decel <= 0.0:
        raise ValueError("comfortable_decel must be a finite positive value")
    if not math.isfinite(hard_stop_distance) or hard_stop_distance < 0.0:
        raise ValueError("hard_stop_distance must be a finite non-negative value")
    if not math.isfinite(min_approach_speed) or min_approach_speed < 0.0:
        raise ValueError("min_approach_speed must be a finite non-negative value")
    if not math.isfinite(approach_time) or approach_time < 0.0:
        raise ValueError("approach_time must be a finite non-negative value")

    remaining = distance - hard_stop_distance
    if remaining <= 0.0:
        return 0.0

    braking_cap = math.sqrt(2.0 * comfortable_decel * remaining)
    if approach_time > 0.0:
        # The square-root brake curve alone slows too late for pickup: the
        # vehicle's front can already reach the FOD while base_link is still
        # 0.8 m away. A proportional approach starts earlier, while retaining
        # the existing low-speed floor and hard-stop behavior.
        braking_cap = min(braking_cap, remaining / approach_time)
    return max(min_approach_speed, braking_cap)


def angular_velocity_at_limited_speed(original_speed, limited_speed, angular_velocity):
    """Scale yaw rate with forward speed so Ackermann curvature is unchanged."""
    if not math.isfinite(original_speed) or original_speed <= 0.0:
        raise ValueError("original_speed must be a finite positive value")
    if not math.isfinite(limited_speed) or limited_speed < 0.0:
        raise ValueError("limited_speed must be a finite non-negative value")
    if limited_speed > original_speed:
        raise ValueError("limited_speed must not exceed original_speed")
    if not math.isfinite(angular_velocity):
        raise ValueError("angular_velocity must be finite")
    return angular_velocity * limited_speed / original_speed


def validate_goal_distances(hard_stop_distance, planner_xy_goal_tolerance):
    """Validate that the safety stop lies strictly inside planner success radius."""
    if not math.isfinite(hard_stop_distance) or hard_stop_distance < 0.0:
        raise ValueError("hard_stop_distance must be finite and non-negative")
    if (
        not math.isfinite(planner_xy_goal_tolerance)
        or planner_xy_goal_tolerance <= 0.0
    ):
        raise ValueError("planner_xy_goal_tolerance must be finite and positive")
    if hard_stop_distance >= planner_xy_goal_tolerance:
        raise ValueError(
            "hard_stop_distance must be smaller than planner_xy_goal_tolerance"
        )


def is_zero_stamp(stamp):
    """Return whether a ROS time value has the actionlib zero-stamp meaning."""
    return stamp.secs == 0 and stamp.nsecs == 0


def cancel_request_matches_goal(cancel_request, goal_id):
    """Implement actionlib GoalID cancellation matching for one active goal."""
    if goal_id is None:
        return False
    if cancel_request.id:
        return cancel_request.id == goal_id.id
    if is_zero_stamp(cancel_request.stamp):
        return True
    return goal_id.stamp <= cancel_request.stamp


class GpsGoalSpeedLimiter:
    def __init__(self):
        self.input_cmd_topic = rospy.get_param("~input_cmd_topic", "/cmd_vel_navigation")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.action_goal_topic = rospy.get_param("~action_goal_topic", "/move_base/goal")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.status_topic = rospy.get_param("~status_topic", "/move_base/status")
        self.speed_cap_enabled = boolean_parameter(
            rospy.get_param("~speed_cap_enabled", False),
            "speed_cap_enabled",
        )
        self.comfortable_decel = float(rospy.get_param("~comfortable_decel", 0.4))
        self.approach_time = float(rospy.get_param("~approach_time", 0.0))
        self.hard_stop_distance = float(
            rospy.get_param("~hard_stop_distance", 0.2)
        )
        self.planner_xy_goal_tolerance = float(
            rospy.get_param("~planner_xy_goal_tolerance", 0.3)
        )
        self.min_approach_speed = float(rospy.get_param("~min_approach_speed", 0.15))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))
        self.near_goal_commit_distance = float(
            rospy.get_param("~near_goal_commit_distance", 1.0)
        )
        self.near_goal_timeout = float(
            rospy.get_param("~near_goal_timeout", 15.0)
        )
        self.near_goal_max_regression = float(
            rospy.get_param("~near_goal_max_regression", 0.5)
        )

        # Validate all numeric parameters before accepting control commands.
        goal_speed_cap(
            1.0,
            self.comfortable_decel,
            self.hard_stop_distance,
            self.min_approach_speed,
            self.approach_time,
        )
        validate_goal_distances(
            self.hard_stop_distance, self.planner_xy_goal_tolerance
        )
        timing_parameters = (self.cmd_timeout, self.odom_timeout, self.publish_rate)
        if any(not math.isfinite(value) or value <= 0.0 for value in timing_parameters):
            raise ValueError("timeouts and publish_rate must be positive")
        if (
            not math.isfinite(self.near_goal_commit_distance)
            or self.near_goal_commit_distance <= self.hard_stop_distance
        ):
            raise ValueError(
                "near_goal_commit_distance must be finite and greater than "
                "hard_stop_distance"
            )
        near_goal_limits = (
            self.near_goal_timeout,
            self.near_goal_max_regression,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in near_goal_limits):
            raise ValueError(
                "near_goal_timeout and near_goal_max_regression must be positive"
            )

        self.lock = threading.Lock()
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_odom = None
        self.latest_odom_time = None
        self.current_goal = None
        self.active_goal_id = None
        self.active_goal_is_terminal = True
        self.stop_latched = True
        self.arrival_latched = False
        self.near_goal_started_time = None
        self.near_goal_closest_distance = None
        self.near_goal_last_progress_time = None
        self.near_goal_progress_pose = None
        self.last_cancel_stamp = rospy.Time()
        self.blocked_goal_ids = OrderedDict()

        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=10)
        self.cmd_sub = rospy.Subscriber(
            self.input_cmd_topic, Twist, self.cmd_callback, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=20
        )
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=5
        )
        self.action_goal_sub = rospy.Subscriber(
            self.action_goal_topic,
            MoveBaseActionGoal,
            self.action_goal_callback,
            queue_size=5,
        )
        self.cancel_sub = rospy.Subscriber(
            self.cancel_topic, GoalID, self.cancel_callback, queue_size=5
        )
        self.status_sub = rospy.Subscriber(
            self.status_topic, GoalStatusArray, self.status_callback, queue_size=5
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        rospy.on_shutdown(self.publish_stop)

        rospy.loginfo(
            "GPS goal safety relay: %s -> %s, terminal speed cap=%s, "
            "decel=%.3f m/s^2, hard_stop_distance=%.3f m, "
            "planner_tolerance=%.3f m",
            self.input_cmd_topic,
            self.output_cmd_topic,
            "enabled" if self.speed_cap_enabled else "disabled",
            self.comfortable_decel,
            self.hard_stop_distance,
            self.planner_xy_goal_tolerance,
        )
        rospy.loginfo(
            "GPS near-goal progress monitor: commit=%.2f m, idle timeout=%.1f s, "
            "regression notice=%.2f m; moving reverse/turn maneuvers remain active",
            self.near_goal_commit_distance,
            self.near_goal_timeout,
            self.near_goal_max_regression,
        )

    def cmd_callback(self, msg):
        with self.lock:
            self.latest_cmd = copy.deepcopy(msg)
            self.latest_cmd_time = rospy.Time.now()

    def odom_callback(self, msg):
        with self.lock:
            self.latest_odom = copy.deepcopy(msg)
            self.latest_odom_time = rospy.Time.now()

    def goal_callback(self, msg):
        # /move_base/current_goal has no GoalID. Keep it only as a startup pose
        # source before identity-aware /move_base/goal tracking begins. A delayed
        # pose-only message must never overwrite or unlock a newer action goal.
        with self.lock:
            accepted = self.active_goal_id is None
            if accepted:
                self.current_goal = copy.deepcopy(msg)
                self.active_goal_is_terminal = True
        if accepted:
            rospy.loginfo(
                "GPS goal relay received pose-only target: frame=%s x=%.3f y=%.3f",
                msg.header.frame_id,
                msg.pose.position.x,
                msg.pose.position.y,
            )
        else:
            rospy.logdebug("Ignored identity-free current_goal pose after action goal")

    def _remember_blocked_goal_id(self, identifier):
        if not identifier:
            return
        self.blocked_goal_ids[identifier] = None
        self.blocked_goal_ids.move_to_end(identifier)
        while len(self.blocked_goal_ids) > 100:
            self.blocked_goal_ids.popitem(last=False)

    def _remember_cancel_request(self, msg):
        if msg.id:
            # actionlib remembers an ID-specific cancel that arrives before its
            # goal, so mirror that behavior for cross-topic callback ordering.
            self._remember_blocked_goal_id(msg.id)
            return

        # A zero ID and zero stamp cancels only goals already known. Unlike a
        # timestamped cancel, it must not reject a future action goal.
        if is_zero_stamp(msg.stamp):
            return
        if is_zero_stamp(self.last_cancel_stamp) or msg.stamp > self.last_cancel_stamp:
            self.last_cancel_stamp = msg.stamp

    def _goal_was_stopped(self, goal_id):
        if goal_id.id and goal_id.id in self.blocked_goal_ids:
            return True
        return (
            not is_zero_stamp(self.last_cancel_stamp)
            and goal_id.stamp <= self.last_cancel_stamp
        )

    def _latch_stop(self):
        self.stop_latched = True
        self.latest_cmd = Twist()
        self.latest_cmd_time = None

    def _reset_goal_progress(self):
        self.arrival_latched = False
        self.near_goal_started_time = None
        self.near_goal_closest_distance = None
        self.near_goal_last_progress_time = None
        self.near_goal_progress_pose = None

    def _pause_goal_progress(self):
        # A localization outage already stops this relay. Give the planner its
        # normal idle interval after localization returns, instead of charging
        # the outage against the final maneuver and requiring a new goal.
        self.near_goal_last_progress_time = None
        self.near_goal_progress_pose = None

    def _near_goal_idle_time(self, now, odom):
        position = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (position.x, position.y, yaw)
        if (self.near_goal_last_progress_time is None
                or now < self.near_goal_last_progress_time):
            self.near_goal_last_progress_time = now
            self.near_goal_progress_pose = pose
            return 0.0

        px, py, previous_yaw = self.near_goal_progress_pose
        displacement = math.hypot(position.x - px, position.y - py)
        rotation = abs(math.atan2(math.sin(yaw - previous_yaw),
                                  math.cos(yaw - previous_yaw)))
        twist = odom.twist.twist
        # Use actual odometry, not the requested cmd_vel. Requiring measured
        # motion also prevents small GNSS position jitter at rest from keeping
        # an immobilized final approach alive. Progress is accumulated from
        # the last credited pose, so slow valid maneuvers count too.
        translating = math.isfinite(twist.linear.x) and abs(twist.linear.x) >= 0.02
        turning = math.isfinite(twist.angular.z) and abs(twist.angular.z) >= 0.02
        if ((translating and displacement >= 0.05)
                or (turning and rotation >= 0.05)):
            self.near_goal_last_progress_time = now
            self.near_goal_progress_pose = pose
        return max(0.0, (now - self.near_goal_last_progress_time).to_sec())

    def action_goal_callback(self, msg):
        goal_id = copy.deepcopy(msg.goal_id)
        goal_pose = copy.deepcopy(msg.goal.target_pose)
        with self.lock:
            previous_goal_id = (
                ""
                if self.active_goal_id is None
                else self.active_goal_id.id
            )
            goal_was_stopped = self._goal_was_stopped(goal_id)
            seamless_route_transition = (
                not self.stop_latched
                and not goal_was_stopped
                and is_contiguous_route_goal_transition(
                    previous_goal_id,
                    goal_id.id,
                )
            )
            self.active_goal_id = goal_id
            self.current_goal = goal_pose
            self.active_goal_is_terminal = not is_intermediate_route_goal_id(
                goal_id.id
            )
            self._reset_goal_progress()
            if not seamless_route_transition:
                self._latch_stop()
                if not goal_was_stopped:
                    self.stop_latched = False
            goal_is_stopped = self.stop_latched
            goal_is_terminal = self.active_goal_is_terminal
            if not seamless_route_transition:
                # Fence an unrelated previous goal's command until move_base
                # publishes a fresh command for this action goal. Consecutive
                # segments of one managed route retain the still-fresh command
                # so rolling replacement does not inject a zero-speed pulse.
                self.cmd_pub.publish(Twist())

        if goal_is_stopped:
            rospy.logwarn("Ignoring already-cancelled move_base goal id=%s", goal_id.id)
        else:
            rospy.loginfo(
                "GPS goal relay accepted %s move_base goal "
                "id=%s frame=%s x=%.3f y=%.3f",
                "terminal" if goal_is_terminal else "rolling-intermediate",
                goal_id.id or "<pending-status-id>",
                goal_pose.header.frame_id,
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
            )
            if seamless_route_transition:
                rospy.logdebug(
                    "Retained fresh command across contiguous GPS route segment"
                )

    def cancel_callback(self, msg):
        with self.lock:
            self._remember_cancel_request(msg)
            matches_active = cancel_request_matches_goal(msg, self.active_goal_id)
            if matches_active:
                self._remember_blocked_goal_id(self.active_goal_id.id)
                self._latch_stop()
                # Timer publishes under this same lock, so either its old command
                # precedes this stop or it observes the latch and publishes zero.
                self.cmd_pub.publish(Twist())

        if matches_active:
            rospy.loginfo("GPS goal relay stopped by move_base cancellation")
        else:
            rospy.logdebug("Ignored cancellation for a non-active move_base goal")

    def status_callback(self, msg):
        bound_goal_id = None
        with self.lock:
            active_id = None if self.active_goal_id is None else self.active_goal_id.id
            if (
                self.active_goal_id is not None
                and not active_id
                and not is_zero_stamp(self.active_goal_id.stamp)
            ):
                tracking = [
                    status
                    for status in msg.status_list
                    if (
                        status.status in TRACKING_GOAL_STATES
                        and status.goal_id.id
                        and status.goal_id.stamp >= self.active_goal_id.stamp
                    )
                ]
                if len(tracking) == 1:
                    self.active_goal_id = copy.deepcopy(tracking[0].goal_id)
                    active_id = self.active_goal_id.id
                    self.active_goal_is_terminal = (
                        not is_intermediate_route_goal_id(active_id)
                    )
                    bound_goal_id = active_id
            stop_active = False
            for status in msg.status_list:
                if status.status in STOP_GOAL_STATES:
                    self._remember_blocked_goal_id(status.goal_id.id)
                    if active_id is not None and status.goal_id.id == active_id:
                        stop_active = True
            if stop_active:
                self._latch_stop()
                self.cmd_pub.publish(Twist())
        if bound_goal_id is not None:
            rospy.loginfo(
                "GPS goal relay bound pending action identity to move_base id=%s",
                bound_goal_id,
            )

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            cmd = copy.deepcopy(self.latest_cmd)
            cmd_time = self.latest_cmd_time
            odom = copy.deepcopy(self.latest_odom)
            odom_time = self.latest_odom_time
            goal = copy.deepcopy(self.current_goal)
            goal_is_terminal = self.active_goal_is_terminal

            # Keep cancellation and terminal action states authoritative until
            # move_base publishes a genuinely new identity-bearing action goal.
            if self.stop_latched:
                self.cmd_pub.publish(Twist())
                return

            # During an active goal, position is required to enforce the goal
            # fence. Missing or stale odometry therefore fails closed.
            if goal is None or odom is None or odom_time is None:
                rospy.logwarn_throttle(
                    2.0, "GPS goal relay fail-stop: goal or GPS odometry is missing"
                )
                self._pause_goal_progress()
                self.cmd_pub.publish(Twist())
                return
            if (now - odom_time).to_sec() > self.odom_timeout:
                rospy.logwarn_throttle(
                    2.0, "GPS goal relay fail-stop: GPS odometry is stale"
                )
                self._pause_goal_progress()
                self.cmd_pub.publish(Twist())
                return

            goal_frame = goal.header.frame_id.lstrip("/")
            odom_frame = odom.header.frame_id.lstrip("/")
            if goal_frame and odom_frame and goal_frame != odom_frame:
                rospy.logwarn_throttle(
                    2.0,
                    "GPS goal relay fail-stop: goal frame %s differs from odom frame %s",
                    goal.header.frame_id,
                    odom.header.frame_id,
                )
                self._pause_goal_progress()
                self.cmd_pub.publish(Twist())
                return

            coordinates = (
                goal.pose.position.x,
                goal.pose.position.y,
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
            )
            if any(not math.isfinite(value) for value in coordinates):
                rospy.logwarn_throttle(
                    2.0, "GPS goal relay fail-stop: goal or odometry position is non-finite"
                )
                self._pause_goal_progress()
                self.cmd_pub.publish(Twist())
                return

            dx = goal.pose.position.x - odom.pose.pose.position.x
            dy = goal.pose.position.y - odom.pose.pose.position.y
            distance = math.hypot(dx, dy)

            if goal_is_terminal:
                # Once the vehicle enters the final arrival radius, keep every
                # degree of freedom stopped. Rolling intermediate goals never
                # arm this latch; they are replaced before move_base reaches
                # them.
                if distance <= self.hard_stop_distance:
                    self.arrival_latched = True
                    self._latch_stop()
                    rospy.loginfo(
                        "Goal arrival latched: distance=%.2f m, threshold=%.2f m; "
                        "GPS jitter cannot restart this goal",
                        distance,
                        self.hard_stop_distance,
                    )
                    self.cmd_pub.publish(Twist())
                    return

                # The timeout measures lack of actual motion, not total time
                # in this zone. Ackermann reverse/turn maneuvers can legitimately
                # move away from the target before lining up with it.
                if self.near_goal_started_time is None:
                    if distance <= self.near_goal_commit_distance:
                        self.near_goal_started_time = now
                        self.near_goal_closest_distance = distance
                        rospy.loginfo(
                            "Near-goal progress monitor armed: distance=%.2f m, "
                            "idle timeout=%.1f s",
                            distance,
                            self.near_goal_timeout,
                        )
                if self.near_goal_started_time is not None:
                    self.near_goal_closest_distance = min(
                        self.near_goal_closest_distance,
                        distance,
                    )
                    idle = self._near_goal_idle_time(now, odom)
                    regression = distance - self.near_goal_closest_distance
                    if regression >= self.near_goal_max_regression:
                        rospy.loginfo_throttle(
                            5.0,
                            "Near-goal maneuver: distance=%.2f m, closest=%.2f m, "
                            "idle=%.1f s; distance regression alone does not stop the car",
                            distance, self.near_goal_closest_distance, idle,
                        )
                    if idle >= self.near_goal_timeout:
                        self._latch_stop()
                        rospy.logerr(
                            "Near-goal stop latched: no measured maneuver progress "
                            "for %.1f s (closest %.2f m); "
                            "publish a new goal to release",
                            idle, self.near_goal_closest_distance,
                        )
                        self.cmd_pub.publish(Twist())
                        return

            # A stale navigation command must never be kept alive by this
            # relay. Evaluate the goal fence first so a planner command gap
            # cannot hide an arrival or a stalled final approach.
            if cmd_time is None or (now - cmd_time).to_sec() > self.cmd_timeout:
                self.cmd_pub.publish(Twist())
                return

            # By default, final-goal commands are also forwarded unchanged.
            # Arrival, near-goal, freshness, cancellation, and action-state
            # safety checks above remain active independently of this optional
            # comfort cap.
            if not goal_is_terminal or not self.speed_cap_enabled:
                self.cmd_pub.publish(cmd)
                return

            cap = goal_speed_cap(
                distance,
                self.comfortable_decel,
                self.hard_stop_distance,
                self.min_approach_speed,
                self.approach_time,
            )

            # Only cap excessive forward approach speed. Lower planner commands
            # and reverse pass through. Scale yaw rate to preserve curvature.
            if cmd.linear.x > cap:
                original_speed = cmd.linear.x
                cmd.linear.x = cap
                cmd.angular.z = angular_velocity_at_limited_speed(
                    original_speed, cap, cmd.angular.z
                )
                rospy.loginfo_throttle(
                    1.0,
                    "GPS terminal speed cap: distance=%.2f m, nav=%.2f m/s, "
                    "output=%.2f m/s",
                    distance,
                    original_speed,
                    cap,
                )

            # Serialize the final output with cancel/status stop publications.
            self.cmd_pub.publish(cmd)

    def publish_stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except rospy.ROSException:
            pass


def main():
    rospy.init_node("gps_goal_speed_limiter")
    try:
        GpsGoalSpeedLimiter()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid GPS goal safety relay configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
