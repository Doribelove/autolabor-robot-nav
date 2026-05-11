#!/home/robot/python_env/rosnav/bin/python3
import os
import glob
import subprocess
import warnings
import time
import math
import argparse
import json

import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tft

# ROS services
from zjr_planner.srv import SaveScan, GenerateRRTPaths

# Pipeline modules
from pointcloud_generate import generate_pointcloud_grid
from merge_data import merge_start_and_paths
from ruleband_api import RuleBandAPI

# Action / message imports for move_base override
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def fallback_publish_goal_once(x, y, yaw=0.0, topic="/move_base_simple/goal", wait_conn=0.9):
    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=False)
    start_t = time.time()
    while pub.get_num_connections() == 0 and time.time() - start_t < wait_conn and not rospy.is_shutdown():
        rospy.sleep(0.02)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.pose = Pose(Point(float(x), float(y), 0.0), Quaternion(q[0], q[1], q[2], q[3]))
    pub.publish(goal)
    rospy.loginfo("Published fallback pose to %s: x=%.3f y=%.3f", topic, x, y)
    return True

def send_action_goal_and_cancel(x, y, yaw=0.0, duration=0.5, wait_server=5.0):
    """
    使用 move_base action 发送 goal 并在 duration 秒后 cancel（干净地释放控制权）。
    返回 True 表示使用 action 成功（即 action server 可用并已 send/cancel），False 表示 action server 不可用。
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server (%.1fs)...", wait_server)
    if not client.wait_for_server(rospy.Duration(wait_server)):
        rospy.logwarn("move_base action server not available after %.1fs", wait_server)
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    q = tft.quaternion_from_euler(0.0, 0.0, float(yaw))
    goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    rospy.loginfo("Sending action goal to move_base: x=%.3f y=%.3f yaw=%.3f", x, y, yaw)
    client.send_goal(goal)


def rrt_first_point_avg(goal_x, goal_y, num_paths=3, step=0.5, max_iters=5000):
    """
    调用 /generate_rrt_paths 服务生成 RRT 路径，
    并返回三条路径第一个点的平均值 (avg_x, avg_y)
    """
    rospy.wait_for_service('/generate_rrt_paths')
    try:
        rrt_srv = rospy.ServiceProxy('/generate_rrt_paths', GenerateRRTPaths)
        resp = rrt_srv(goal_x, goal_y, num_paths, step, max_iters)
        if not resp.success:
            rospy.logwarn("RRT path generation failed.")
            return None

        import json
        paths_json = json.loads(resp.json)
        first_points = []
        for k, v in paths_json.items():
            path = v["path"]
            if len(path) > 0:
                first_points.append(path[0]["position"])
            if len(first_points) >= 3:
                break

        if len(first_points) == 0:
            rospy.logwarn("No valid paths returned.")
            return None

        avg_x = sum(p[0] for p in first_points) / len(first_points)
        avg_y = sum(p[1] for p in first_points) / len(first_points)
        rospy.loginfo("Average first point of RRT paths: (%.3f, %.3f)", avg_x, avg_y)
        return avg_x, avg_y

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None
def main_loop(period=0.9, final_goal_x=None, final_goal_y=None):
    while not rospy.is_shutdown():
        loop_start = time.time()
        x_sub,y_sub = rrt_first_point_avg(final_goal_x, final_goal_y)
        if x_sub is not None:
            rospy.loginfo("Generated subgoal: (%.3f, %.3f)", x_sub, y_sub)
            # 2) override by sending subgoal (duration 0.5s)
            ok = send_action_goal_and_cancel(x_sub,y_sub, yaw=0.0, duration=0.4, wait_server=2.0)
            if not ok:
                rospy.logwarn("Action server unavailable; fallback publish for subgoal")
                fallback_publish_goal_once(x_sub, y_sub, yaw=0.0)

            # # 3) immediately send final goal (duration 0.3s)
            # if final_goal_x is not None and final_goal_y is not None:
            #     ok2 = send_action_goal_and_cancel(final_goal_x, final_goal_y, yaw=0.0, duration=0.3, wait_server=2.0)
            #     if not ok2:
            #         rospy.logwarn("Action server unavailable; fallback publish for final goal")
            #         fallback_publish_goal_once(final_goal_x, final_goal_y, yaw=0.0)
            # else:
            #     rospy.logwarn("Final goal not provided; skipping final-goal publish.")
        # 控制周期为 period 秒（减去本次循环实际耗时）
        elapsed = time.time() - loop_start
        to_sleep = period - elapsed
        if to_sleep > 0:
            rospy.sleep(to_sleep)
        else:
            # pipeline 或 action 超时导致本轮超时，直接进入下一轮（不 sleep）
            rospy.logwarn("Loop overran desired period by %.3fs", -to_sleep)

if __name__ == "__main__":
    rospy.init_node("rrt_avg_first_point_node")
    try:
        # 等待 /goal 话题发布一次消息
        msg = rospy.wait_for_message("/goal", PoseStamped, timeout=5.0)
        final_goal_x = msg.pose.position.x
        final_goal_y = msg.pose.position.y
        rospy.loginfo("Received final goal from /goal: x=%.3f y=%.3f", final_goal_x, final_goal_y)
    except rospy.ROSException:
        rospy.logwarn("No /goal message received, using default values")
        final_goal_x = 1.73
        final_goal_y = -9.57
    try:
        main_loop(period=0.9, final_goal_x=final_goal_x, final_goal_y=final_goal_y)
    except rospy.ROSInterruptException:
        pass