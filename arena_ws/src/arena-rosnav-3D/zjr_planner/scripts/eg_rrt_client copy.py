#!/usr/bin/env python3
import rospy
from zjr_planner.srv import GenerateRRTPaths

def call_rrt_service(goal_x, goal_y, num_paths=3, step=0.5, max_iters=5000):
    rospy.wait_for_service('/generate_rrt_paths')
    try:
        rrt_srv = rospy.ServiceProxy('/generate_rrt_paths', GenerateRRTPaths)
        resp = rrt_srv(goal_x, goal_y, num_paths, step, max_iters)
        if resp.success:
            print("RRT paths saved to:", resp.filename)
            print("JSON data:", resp.json)
        else:
            print("RRT path generation failed.")
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('rrt_client_node')
    # 目标位置
    goal_x = 5.0
    goal_y = 3.0
    call_rrt_service(goal_x, goal_y)
