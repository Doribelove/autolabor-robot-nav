#!/usr/bin/env python3
"""Apply a comfortable forward-speed cap only near the active GPS goal."""

import copy
import math
import threading

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def goal_speed_cap(distance, comfortable_decel, goal_tolerance, min_approach_speed):
    """Return the kinematic speed cap for the remaining distance to a goal."""
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("distance must be a finite non-negative value")
    if not math.isfinite(comfortable_decel) or comfortable_decel <= 0.0:
        raise ValueError("comfortable_decel must be a finite positive value")
    if not math.isfinite(goal_tolerance) or goal_tolerance < 0.0:
        raise ValueError("goal_tolerance must be a finite non-negative value")
    if not math.isfinite(min_approach_speed) or min_approach_speed < 0.0:
        raise ValueError("min_approach_speed must be a finite non-negative value")

    remaining = distance - goal_tolerance
    if remaining <= 0.0:
        return 0.0

    braking_cap = math.sqrt(2.0 * comfortable_decel * remaining)
    return max(min_approach_speed, braking_cap)


class GpsGoalSpeedLimiter:
    def __init__(self):
        self.input_cmd_topic = rospy.get_param("~input_cmd_topic", "/cmd_vel_navigation")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.comfortable_decel = float(rospy.get_param("~comfortable_decel", 0.4))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.5))
        self.min_approach_speed = float(rospy.get_param("~min_approach_speed", 0.15))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 50.0))

        # Validate all numeric parameters before accepting control commands.
        goal_speed_cap(1.0, self.comfortable_decel, self.goal_tolerance, self.min_approach_speed)
        if self.cmd_timeout <= 0.0 or self.odom_timeout <= 0.0 or self.publish_rate <= 0.0:
            raise ValueError("timeouts and publish_rate must be positive")

        self.lock = threading.Lock()
        self.latest_cmd = Twist()
        self.latest_cmd_time = None
        self.latest_odom = None
        self.latest_odom_time = None
        self.current_goal = None
        self.cancelled = False

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
        self.cancel_sub = rospy.Subscriber(
            self.cancel_topic, GoalID, self.cancel_callback, queue_size=5
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        rospy.on_shutdown(self.publish_stop)

        rospy.loginfo(
            "GPS goal speed limiter: %s -> %s, decel=%.3f m/s^2, tolerance=%.3f m",
            self.input_cmd_topic,
            self.output_cmd_topic,
            self.comfortable_decel,
            self.goal_tolerance,
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
        with self.lock:
            self.current_goal = copy.deepcopy(msg)
            self.cancelled = False
        rospy.loginfo(
            "Goal slowdown tracking target: frame=%s x=%.3f y=%.3f",
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def cancel_callback(self, _msg):
        with self.lock:
            self.cancelled = True
            self.latest_cmd = Twist()
            self.latest_cmd_time = rospy.Time.now()
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Goal slowdown relay stopped by move_base cancellation")

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            cmd = copy.deepcopy(self.latest_cmd)
            cmd_time = self.latest_cmd_time
            odom = copy.deepcopy(self.latest_odom)
            odom_time = self.latest_odom_time
            goal = copy.deepcopy(self.current_goal)
            cancelled = self.cancelled

        # Keep cancellation and the electronic-fence stop authoritative until
        # move_base accepts and publishes a new current goal.
        if cancelled:
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
        cap = goal_speed_cap(
            distance,
            self.comfortable_decel,
            self.goal_tolerance,
            self.min_approach_speed,
        )

        # Only cap forward approach speed. Lower planner commands (including an
        # obstacle stop), reverse recovery, and all angular commands pass through.
        if cmd.linear.x > cap:
            original_speed = cmd.linear.x
            cmd.linear.x = cap
            rospy.loginfo_throttle(
                1.0,
                "Goal slowdown: distance=%.2f m, nav=%.2f m/s, output=%.2f m/s",
                distance,
                original_speed,
                cap,
            )

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
