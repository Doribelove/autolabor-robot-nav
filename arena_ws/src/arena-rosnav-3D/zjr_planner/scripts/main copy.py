#!/usr/bin/env python3
# use_merge_example.py
import json
import os
import glob
import subprocess
import warnings
import sys
import select
import time
from pointcloud_generate import generate_pointcloud_grid, live_lidar_plot
from rrt_path_generate_removable import RRTMultiPathPlanner
from merge_data import merge_start_and_paths
from nav_goal_utils import send_nav_goal
from ruleband_api import RuleBandAPI
import rospy
from nav_msgs.msg import Odometry
import math
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import tf.transformations as tft

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")

def run_rrt_once(goal_x, goal_y, out_file=None, num_paths=3, step=0.2, max_iters=5000):
    """
    调用 rrt_path_generate.py 一次，生成路径并保存到 out_file
    """
    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    cmd = [
        "python3",
        "/home/robot/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py",
        "--goal", str(goal_x), str(goal_y),
        "--num_paths", str(num_paths),
        "--step", str(step),
        "--max_iters", str(max_iters)
    ]

    if out_file:
        cmd.extend(["--out", out_file])

    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    print("STDOUT:", result.stdout)
    print("STDERR:", result.stderr)

    if result.returncode != 0:
        raise RuntimeError(f"rrt script failed (rc={result.returncode})")

    return out_file


def main(x,y):
    # 1) 保存一次激光点云
    saved_file = generate_pointcloud_grid(index=5)  
    print(f"已保存到: {saved_file}")

    # 2) 生成目标点
    start = (x, y)

    # 3) 使用 rrt 生成路径并保存
    path_file = run_rrt_once(
        x, y,
        out_file="~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json"
    )

    # 4) 合并成 example_data.json
    saved = merge_start_and_paths(
        data_start_path="data/pointcloud/data_start_5.json",
        paths_path="data/ros_data/paths.json",
        start=start,
        out_path="data_json/example_data.json"
    )
    print("Saved merged file to:", saved)

    # 5) 使用 ruleband api 预测子目标
    api = RuleBandAPI(device="cpu")
    x_sub, y_sub = api.predict_from_file("data_json/example_data.json", debug=True)
    print(f"Sampled rule-based sub-goal → ({x:.2f}, {y:.2f})")

    # 6) 删除 data_json 下所有 json
    folder_path = "./data_json"
    json_files = glob.glob(os.path.join(folder_path, "*.json"))
    for file in json_files:
        os.remove(file)
        print(f"已删除 {file}")
    
    return x_sub,y_sub

def is_robot_near_goal(goal_x, goal_y, r, timeout=5.0):
    """
    判断无人车当前位置是否在目标点 r 半径范围内

    :param goal_x: 目标点 x 坐标
    :param goal_y: 目标点 y 坐标
    :param r: 判定半径
    :param timeout: 等待 /odom 超时时间 (秒)
    :return: True (在范围内), False (不在范围内 或 超时)
    """
    try:
        # 等待一次 /odom 消息
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("等待 /odom 超时")
        return False

    # 提取当前位置
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 计算欧几里得距离
    dist = math.hypot(goal_x - x, goal_y - y)
    rospy.loginfo(f"当前位置: ({x:.3f}, {y:.3f}), 目标点: ({goal_x:.3f}, {goal_y:.3f}), 距离: {dist:.3f}")

    return dist < r

# 设置机器人在 Gazebo 中的初始位置
def set_robot_position(x, y, yaw):
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'turtlebot3_waffle'  # 根据实际模型名称修改
        quaternion = tft.quaternion_from_euler(0, 0, yaw)
        model_state.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))
        set_model_state(model_state)
        rospy.loginfo(f"Robot moved to position ({x}, {y}) with yaw {yaw}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# 发布初始位姿到导航栈
def publish_initial_pose(x, y, yaw):
    global initial_pose_pub
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()

    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    initial_pose.pose.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))

    # 设置协方差为一个较小的值，表示定位精度高
    initial_pose.pose.covariance = [0.1] * 36

    initial_pose_pub.publish(initial_pose)
    rospy.sleep(0.1)
    rospy.loginfo(f"Initial pose published to ({x}, {y}) with yaw {yaw}.")

# 发布导航目标点
def send_navigation_goal(x, y, yaw):
    nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 等待发布器准备好

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = rospy.Time.now()

    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    goal_pose.pose = Pose(Point(x, y, 0), Quaternion(*quaternion))

    nav_goal_pub.publish(goal_pose)
    rospy.loginfo(f"Navigation goal sent to ({x}, {y}) with yaw {yaw}.")


if __name__ == "__main__":
    
    goal_x,goal_y = 1.73, -9.57
    start_x,start_y = -3.71 , 9.21
    radius = 0.5
    cnt=0
    rospy.sleep(0.2) 
    set_robot_position(start_x, start_y, 0)
    rospy.sleep(0.2) 
    # 发布初始位姿到导航栈
    publish_initial_pose(start_x, start_y, 0)

    while True:
        time.sleep(0.4)
        x_sub,y_sub=main(goal_x,goal_y)
        print(x_sub,y_sub)
        send_nav_goal(x_sub, y_sub, 0)
        cnt+=1
        if is_robot_near_goal(goal_x, goal_y, radius):
            print("机器人到达目标点附近，重新规划")
            rospy.sleep(0.2) 
            set_robot_position(start_x, start_y, 0)
            rospy.sleep(0.2) 
            # 发布初始位姿到导航栈
            publish_initial_pose(start_x, start_y, 0)
            cnt=0
            continue
        if cnt>30:
            print("机器人无法到达目标点，重新规划")
            rospy.sleep(0.2) 
            set_robot_position(start_x, start_y, 0)
            rospy.sleep(0.2) 
            # 发布初始位姿到导航栈
            publish_initial_pose(start_x, start_y, 0)
            cnt=0
            continue




    print("over")

    # live_lidar_plot("/scan")

