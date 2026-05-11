#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
nav_goal_utils.py
向 /move_base_simple/goal 发送目标点（类似 RViz 的 2D Nav Goal）。
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations as tft

def _ensure_node(node_name="nav_goal_client"):
    """若尚未初始化 rospy 节点，则初始化"""
    if not rospy.core.is_initialized():
        rospy.init_node(node_name, anonymous=True)

def send_nav_goal(x, y, yaw,
                  frame_id="map",
                  goal_topic="/move_base_simple/goal"):
    """
    向 move_base_simple/goal 发布目标点
    参数：
        x, y, yaw : 目标点 (米, 弧度)
        frame_id  : 坐标系（通常 "map"）
        goal_topic: 目标话题（默认 /move_base_simple/goal）
    返回：
        True（代表成功发布）
    """
    _ensure_node()
    pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)

    # 等待至少有一个订阅者（防止消息丢失）
    start_time = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.05)
        if (rospy.Time.now() - start_time).to_sec() > 2.0:
            break

    # 构造并发布目标
    q = tft.quaternion_from_euler(0, 0, yaw)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time(0)  # 关键改动：使用 stamp=0，避免仿真时间不同步问题
    goal.header.frame_id = frame_id
    goal.pose = Pose()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    pub.publish(goal)
    rospy.loginfo(f"[send_nav_goal] Published goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad to {goal_topic}")
    return True

# 测试运行： python3 nav_goal_utils.py 1.0 1.0 1.57
if __name__ == "__main__":
    import sys
    if len(sys.argv) not in (3, 4):
        print("Usage: python3 nav_goal_utils.py x y [yaw_rad]")
        sys.exit(0)
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) == 4 else 0.0
    send_nav_goal(x, y, yaw)
