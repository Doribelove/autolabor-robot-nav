#!/usr/bin/env python3
"""Apply a comfortable forward-speed cap only near the active GPS goal."""

from collections import OrderedDict
import copy
import math
import threading

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry


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


def goal_speed_cap(distance, comfortable_decel, hard_stop_distance, min_approach_speed):
    """Return the kinematic speed cap for the remaining distance to a goal."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("distance must be a finite non-negative value")
    if not math.isfinite(comfortable_decel) or comfortable_decel <= 0.0:
        raise ValueError("comfortable_decel must be a finite positive value")
    if not math.isfinite(hard_stop_distance) or hard_stop_distance < 0.0:
        raise ValueError("hard_stop_distance must be a finite non-negative value")
    if not math.isfinite(min_approach_speed) or min_approach_speed < 0.0:
        raise ValueError("min_approach_speed must be a finite non-negative value")

    remaining = distance - hard_stop_distance
    if remaining <= 0.0:
        return 0.0

    braking_cap = math.sqrt(2.0 * comfortable_decel * remaining)
    return max(min_approach_speed, braking_cap)


def angular_velocity_at_limited_speed(original_speed, limited_speed, angular_velocity):
    """Scale yaw rate with speed so a limited Ackermann command keeps curvature."""
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
        self.comfortable_decel = float(rospy.get_param("~comfortable_decel", 0.4))
        if rospy.has_param("~hard_stop_distance"):
            self.hard_stop_distance = float(rospy.get_param("~hard_stop_distance"))
        elif rospy.has_param("~goal_tolerance"):
            # Keep old launch files usable, but do not couple new configurations
            # to the local planner's xy_goal_tolerance.
            self.hard_stop_distance = float(rospy.get_param("~goal_tolerance"))
            rospy.logwarn(
                "~goal_tolerance is deprecated for gps_goal_speed_limiter; "
                "use ~hard_stop_distance instead"
            )
        else:
            self.hard_stop_distance = 0.2
        self.planner_xy_goal_tolerance = None
        if rospy.has_param("~planner_xy_goal_tolerance"):
            self.planner_xy_goal_tolerance = float(
                rospy.get_param("~planner_xy_goal_tolerance")
            )
        self.min_approach_speed = float(rospy.get_param("~min_approach_speed", 0.15))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))

        # Validate all numeric parameters before accepting control commands.
        goal_speed_cap(
            1.0,
            self.comfortable_decel,
            self.hard_stop_distance,
            self.min_approach_speed,
        )
        if self.planner_xy_goal_tolerance is not None:
            validate_goal_distances(
                self.hard_stop_distance, self.planner_xy_goal_tolerance
            )
        timing_parameters = (self.cmd_timeout, self.odom_timeout, self.publish_rate)
        if any(not math.isfinite(value) or value <= 0.0 for value in timing_parameters):
            raise ValueError("timeouts and publish_rate must be positive")

        self.lock = threading.Lock()
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_odom = None
        self.latest_odom_time = None
        self.current_goal = None
        self.active_goal_id = None
        self.stop_latched = True
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
            "GPS goal speed limiter: %s -> %s, decel=%.3f m/s^2, "
            "hard_stop_distance=%.3f m, planner_tolerance=%s",
            self.input_cmd_topic,
            self.output_cmd_topic,
            self.comfortable_decel,
            self.hard_stop_distance,
            (
                "unverified"
                if self.planner_xy_goal_tolerance is None
                else "%.3f m" % self.planner_xy_goal_tolerance
            ),
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
        # /move_base/current_goal has no GoalID. It is retained only as a
        # compatibility pose source before an action goal has ever been seen;
        # once identity-aware tracking is active, a delayed pose-only message
        # must not overwrite a newer /move_base/goal target.
        with self.lock:
            accepted = self.active_goal_id is None
            if accepted:
                self.current_goal = copy.deepcopy(msg)
        if accepted:
            rospy.loginfo(
                "Goal slowdown received pose-only target: frame=%s x=%.3f y=%.3f",
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
            # goal, so mirror that behavior for cross-topic callback reordering.
            self._remember_blocked_goal_id(msg.id)
            return

        # A zero ID and zero stamp cancels only goals that are currently known;
        # unlike a timestamped cancel, it must not reject future goals.
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

    def action_goal_callback(self, msg):
        goal_id = copy.deepcopy(msg.goal_id)
        goal_pose = copy.deepcopy(msg.goal.target_pose)
        with self.lock:
            self.active_goal_id = goal_id
            self.current_goal = goal_pose
            self._latch_stop()
            if not self._goal_was_stopped(goal_id):
                self.stop_latched = False
            goal_is_stopped = self.stop_latched

        if goal_is_stopped:
            rospy.logwarn(
                "Ignoring already-cancelled move_base goal id=%s", goal_id.id
            )
        else:
            rospy.loginfo("Goal slowdown accepted move_base goal id=%s", goal_id.id)

    def cancel_callback(self, msg):
        with self.lock:
            self._remember_cancel_request(msg)
            matches_active = cancel_request_matches_goal(msg, self.active_goal_id)
            if matches_active:
                self._remember_blocked_goal_id(self.active_goal_id.id)
                self._latch_stop()
                # Publish while holding the same lock used by timer_callback.
                # This prevents a timer snapshot from publishing an old nonzero
                # command after the cancellation stop.
                self.cmd_pub.publish(Twist())

        if matches_active:
            rospy.loginfo("Goal slowdown relay stopped by move_base cancellation")
        else:
            rospy.logdebug("Ignored cancellation for a non-active move_base goal")

    def status_callback(self, msg):
        with self.lock:
            active_id = None if self.active_goal_id is None else self.active_goal_id.id
            stop_active = False
            for status in msg.status_list:
                if status.status in STOP_GOAL_STATES:
                    self._remember_blocked_goal_id(status.goal_id.id)
                    if active_id is not None and status.goal_id.id == active_id:
                        stop_active = True
            if stop_active:
                self._latch_stop()
                self.cmd_pub.publish(Twist())

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            cmd = copy.deepcopy(self.latest_cmd)
            cmd_time = self.latest_cmd_time
            odom = copy.deepcopy(self.latest_odom)
            odom_time = self.latest_odom_time
            goal = copy.deepcopy(self.current_goal)

            # Keep cancellation, terminal action states, and the electronic-fence
            # stop authoritative until move_base publishes a genuinely new goal.
            if self.stop_latched:
                self.cmd_pub.publish(Twist())
                return

            # A stale navigation command must never be kept alive by this relay.
            if cmd_time is None or (now - cmd_time).to_sec() > self.cmd_timeout:
                self.cmd_pub.publish(Twist())
                return

            # No accepted goal or no fresh pose: preserve the planner command. This
            # keeps the relay from changing obstacle handling if GPS data is absent.
            if goal is None or odom is None or odom_time is None:
                self.cmd_pub.publish(cmd)
                return
            if (now - odom_time).to_sec() > self.odom_timeout:
                rospy.logwarn_throttle(2.0, "Goal slowdown skipped: GPS odometry is stale")
                self.cmd_pub.publish(cmd)
                return

            goal_frame = goal.header.frame_id.lstrip("/")
            odom_frame = odom.header.frame_id.lstrip("/")
            if goal_frame and odom_frame and goal_frame != odom_frame:
                rospy.logwarn_throttle(
                    2.0,
                    "Goal slowdown skipped: goal frame %s differs from odom frame %s",
                    goal.header.frame_id,
                    odom.header.frame_id,
                )
                self.cmd_pub.publish(cmd)
                return

            dx = goal.pose.position.x - odom.pose.pose.position.x
            dy = goal.pose.position.y - odom.pose.pose.position.y
            distance = math.hypot(dx, dy)

            # This is a safety fallback inside the planner's (separate) goal
            # tolerance, so it must stop every degree of freedom rather than only
            # cap positive linear velocity.
            if distance <= self.hard_stop_distance:
                rospy.loginfo_throttle(
                    1.0,
                    "Goal hard stop: distance=%.2f m, threshold=%.2f m",
                    distance,
                    self.hard_stop_distance,
                )
                self.cmd_pub.publish(Twist())
                return

            cap = goal_speed_cap(
                distance,
                self.comfortable_decel,
                self.hard_stop_distance,
                self.min_approach_speed,
            )

            # Only cap excessive forward approach speed. Lower planner commands
            # (including an obstacle stop) and reverse recovery pass through. If
            # forward speed is capped, scale yaw rate too so curvature is unchanged.
            if cmd.linear.x > cap:
                original_speed = cmd.linear.x
                cmd.linear.x = cap
                cmd.angular.z = angular_velocity_at_limited_speed(
                    original_speed, cap, cmd.angular.z
                )
                rospy.loginfo_throttle(
                    1.0,
                    "Goal slowdown: distance=%.2f m, nav=%.2f m/s, output=%.2f m/s",
                    distance,
                    original_speed,
                    cap,
                )

            # Publishing while holding self.lock serializes timer output with
            # cancellation/status stops and closes the stale-command race.
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
        rospy.logfatal("Invalid GPS goal speed limiter configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
