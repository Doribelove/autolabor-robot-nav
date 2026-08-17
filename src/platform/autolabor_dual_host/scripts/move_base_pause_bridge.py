#!/usr/bin/env python3
"""Pause move_base and safely reissue the retained target pose on resume."""

import copy
import json
import threading

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse


class MoveBasePauseBridge:
    def __init__(self):
        self.action_goal_topic = rospy.get_param(
            "~action_goal_topic", "/move_base/goal"
        )
        self.simple_goal_topic = rospy.get_param(
            "~simple_goal_topic", "/move_base_simple/goal"
        )
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.reissue_on_resume = self._strict_bool("~reissue_on_resume", True)
        self.lock = threading.RLock()
        self.paused = False
        self.retained_pose = None
        self.retained_goal_id = ""

        self.cancel_pub = rospy.Publisher(self.cancel_topic, GoalID, queue_size=5)
        self.goal_pub = rospy.Publisher(
            self.simple_goal_topic, PoseStamped, queue_size=1
        )
        self.paused_pub = rospy.Publisher("~paused", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber(
            self.action_goal_topic,
            MoveBaseActionGoal,
            self._goal_callback,
            queue_size=10,
        )
        self.service = rospy.Service("~set_paused", SetBool, self._set_paused)
        self._publish_status("ready")

    @staticmethod
    def _strict_bool(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not bool:
            raise ValueError("{} must be a YAML boolean".format(name))
        return value

    def _goal_callback(self, message):
        with self.lock:
            if self.paused:
                return
            self.retained_pose = copy.deepcopy(message.goal.target_pose)
            self.retained_goal_id = str(message.goal_id.id)
        self._publish_status("navigation goal retained")

    def _publish_cancel_all(self):
        cancel = GoalID()
        cancel.stamp = rospy.Time()
        cancel.id = ""
        self.cancel_pub.publish(cancel)

    def _set_paused(self, request):
        reissue = None
        with self.lock:
            requested = bool(request.data)
            if requested == self.paused:
                state = "paused" if requested else "running"
                return SetBoolResponse(success=True, message="already {}".format(state))
            self.paused = requested
            if requested:
                self._publish_cancel_all()
                message = "move_base goal canceled and retained"
            else:
                if self.reissue_on_resume and self.retained_pose is not None:
                    reissue = copy.deepcopy(self.retained_pose)
                    reissue.header.stamp = rospy.Time.now()
                    message = "retained move_base target reissued"
                else:
                    message = "navigation resumed without a retained target"
        if reissue is not None:
            self.goal_pub.publish(reissue)
        self._publish_status(message)
        return SetBoolResponse(success=True, message=message)

    def _publish_status(self, reason):
        with self.lock:
            payload = {
                "paused": self.paused,
                "has_retained_goal": self.retained_pose is not None,
                "retained_goal_id": self.retained_goal_id,
                "reason": reason,
                "reissue_on_resume": self.reissue_on_resume,
            }
        self.paused_pub.publish(Bool(data=payload["paused"]))
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("navigation_pause", anonymous=False)
    MoveBasePauseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
