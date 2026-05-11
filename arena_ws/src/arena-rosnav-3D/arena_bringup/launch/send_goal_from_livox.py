#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def send_goal_in_odom(x, y, yaw):
    rospy.init_node("livox_goal_sender")

    # TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # MoveBase Client
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # 创建 livox_frame 下的目标
    goal_livox = PoseStamped()
    goal_livox.header.frame_id = "livox_frame"
    goal_livox.header.stamp = rospy.Time.now()
    goal_livox.pose.position.x = x
    goal_livox.pose.position.y = y
    goal_livox.pose.position.z = 0.0
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    goal_livox.pose.orientation.x = q[0]
    goal_livox.pose.orientation.y = q[1]
    goal_livox.pose.orientation.z = q[2]
    goal_livox.pose.orientation.w = q[3]

    # 转换到 odom
    try:
        goal_odom = tf_buffer.transform(goal_livox, "odom", rospy.Duration(1.0))
    except tf2_ros.TransformException as ex:
        rospy.logerr("Transform failed: %s", ex)
        return

    # 发送给 move_base
    goal = MoveBaseGoal()
    goal.target_pose = goal_odom
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reached!")

if __name__ == "__main__":
    send_goal_in_odom(1.0, 0.0, 0.0)  # 示例：x=1.0m, y=0.0m, yaw=0

