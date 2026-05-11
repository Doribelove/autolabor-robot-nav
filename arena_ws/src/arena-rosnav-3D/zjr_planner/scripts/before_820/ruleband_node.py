#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf

from RuleBand_API_main.rule_api import RuleBandAPI  # 根据你的包路径调整

class RuleBandNode:
    def __init__(self):
        rospy.init_node('ruleband_node', anonymous=False)

        # 参数
        self.scenario_topic = rospy.get_param('~scenario_topic', '/arena/scenario')
        self.goal_topic     = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.frame_id       = rospy.get_param('~frame_id', 'map')

        # 发布器：子目标点
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)

        # 订阅器：场景 JSON
        self.scenario_sub = rospy.Subscriber(
            self.scenario_topic,
            String,
            self.scenario_callback,
            queue_size=1
        )

        # RuleBand API 实例
        self.api = RuleBandAPI(device='cpu')

        rospy.loginfo(f"[ruleband_node] Ready, listening on {self.scenario_topic}")

    def scenario_callback(self, msg: String):
        """
        收到场景 JSON 字符串后：
         1. 解析成 dict
         2. 调用 RuleBandAPI.predict_from_scenario 输出子目标
         3. 发布 PoseStamped 到 move_base_simple/goal
        """
        try:
            scenario = json.loads(msg.data)
        except json.JSONDecodeError as e:
            rospy.logerr(f"[ruleband_node] JSON decode error: {e}")
            return

        # 生成子目标
        try:
            x, y = self.api.predict_from_scenario(scenario, debug=False)
        except Exception as e:
            rospy.logerr(f"[ruleband_node] RuleBandAPI error: {e}")
            return

        # 发布 PoseStamped
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # 默认面向朝向不变，若需设定 yaw 可在此添加：
        yaw = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        rospy.loginfo(f"[ruleband_node] Publish sub-goal: x={x:.3f}, y={y:.3f}")
        self.goal_pub.publish(goal)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = RuleBandNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
