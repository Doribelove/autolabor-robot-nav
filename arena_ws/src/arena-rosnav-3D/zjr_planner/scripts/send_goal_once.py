#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def send_goal_once(x, y, yaw=0.0, frame_id="map"):
    """
    发布一次目标点到 /move_base_simple/goal
    :param x: 目标点 X 坐标
    :param y: 目标点 Y 坐标
    :param yaw: 朝向 (弧度制)，默认 0
    :param frame_id: 坐标系，默认 'map'
    """
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    rospy.init_node('send_goal_once', anonymous=True)

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = frame_id
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # 把 yaw 转成四元数
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    # 发布一次
    pub.publish(goal)
    rospy.loginfo(f"Sent goal: ({x}, {y}, yaw={yaw})")

if __name__ == "__main__":
    try:
        send_goal_once(1.73, -9.57, yaw=0.0)
    except rospy.ROSInterruptException:
        pass
