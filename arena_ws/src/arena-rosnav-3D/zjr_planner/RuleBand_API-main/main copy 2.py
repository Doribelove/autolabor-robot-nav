#!/usr/bin/env python3
# use_merge_example_fixed.py
"""
修正版：在 scenario 仿真环境中
1) 把 robot_position / robot_goal 在 json 中设为占位值（例如 [0.0,0.0]）
2) 运行此脚本 -> 在 Gazebo 中设置机器人初始位姿（/gazebo/set_model_state）
3) 根据 pipeline 生成子目标并发布导航目标（move_base 或 /move_base_simple/goal）
"""

import json
import os
import glob
import subprocess
import warnings
import sys
import time
import math

# 你原来的模块依赖（假设这些模块在你的 workspace 中存在）
from pointcloud_generate import generate_pointcloud_grid, live_lidar_plot
from rrt_path_generate_removable import RRTMultiPathPlanner
from merge_data import merge_start_and_paths
from nav_goal_utils import send_nav_goal  # 我们优先用这个，如果不存在会回退到内部实现
from ruleband_api import RuleBandAPI

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import tf.transformations as tft

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
warnings.filterwarnings("ignore", category=UserWarning, message=".*libiomp5md")


def run_rrt_once(goal_x, goal_y, out_file=None, num_paths=3, step=0.3, max_iters=4000):
    """
    调用外部 rrt_path_generate.py 生成路径并保存到 out_file（如果提供）
    返回 out_file 的绝对路径（如果 out_file 为 None 则返回 None）
    """
    if out_file:
        out_file = os.path.expanduser(out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

    # 请确保下面路径是你系统上实际存在的 rrt_path_generate.py 脚本路径
    rrt_script = "/home/robot/catin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py"
    # 如果上面路径不对，请把 rrt_script 改为正确路径。作为回退，尝试相对路径
    if not os.path.exists(rrt_script):
        # 尝试仓库相对路径（回退）
        rrt_script = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/rrt_path_generate.py")

    cmd = [
        "python3",
        rrt_script,
        "--goal", str(goal_x), str(goal_y),
        "--num_paths", str(num_paths),
        "--step", str(step),
        "--max_iters", str(max_iters)
    ]

    if out_file:
        cmd.extend(["--out", out_file])

    rospy.loginfo(f"Running RRT: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    rospy.loginfo("RRT STDOUT: " + (result.stdout or "<empty>"))
    if result.stderr:
        rospy.logwarn("RRT STDERR: " + result.stderr)

    if result.returncode != 0:
        raise RuntimeError(f"rrt script failed (rc={result.returncode})")

    return out_file


def pipeline_generate_subgoal(start_x,start_y,goal_x, goal_y):
    """
    执行点云保存 -> RRT -> 合并 -> RuleBand 预测，返回 (x_sub, y_sub)
    """
    # 1) 保存一次激光点云 (generate_pointcloud_grid 返回 data_start_path)
    saved_file = generate_pointcloud_grid(index=5)
    rospy.loginfo(f"Pointcloud saved to: {saved_file}")

    # 2) start is the robot current start in our pipeline (we keep it same as caller)
    goal = (goal_x, goal_y)  # NOTE: caller may pass start differently; here we keep signature
    start=(start_x,start_y)
    # 3) 使用 RRT 生成路径并保存 (指定输出文件)
    out_paths = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/paths.json")
    try:
        path_file = run_rrt_once(goal_x, goal_y, out_file=out_paths)
    except Exception as e:
        rospy.logerr(f"RRT generation failed: {e}")
        raise

    # 4) 合并成 example_data.json
    # 使用刚保存的 saved_file 做为 data_start_path，paths_path 使用 path_file
    merged_out = os.path.expanduser("~/catkin_arena/src/zjr_planner/RuleBand_API-main/data_json/example_data.json")
    try:
        saved = merge_start_and_paths(
            start=start,
            goal=goal,
            out_path=merged_out
        )
    except Exception as e:
        rospy.logerr(f"merge_start_and_paths failed: {e}")
        raise
    rospy.loginfo("Saved merged file to: %s" % saved)

    # 5) 使用 ruleband api 预测子目标
    api = RuleBandAPI(device="cpu")
    try:
        x_sub, y_sub = api.predict_from_file(merged_out, debug=True)
    except Exception as e:
        rospy.logerr(f"RuleBand prediction failed: {e}")
        raise

    rospy.loginfo(f"Sampled rule-based sub-goal → ({x_sub:.2f}, {y_sub:.2f})")

    # 6) 删除 data_json 下所有 json（可选清理）
    folder_path = os.path.dirname(merged_out)
    json_files = glob.glob(os.path.join(folder_path, "*.json"))
    for file in json_files:
        try:
            os.remove(file)
            rospy.loginfo(f"Deleted {file}")
        except Exception as e:
            rospy.logwarn(f"Failed to delete {file}: {e}")

    return x_sub, y_sub

def get_robot_pose(timeout=2.0):
    """
    从 /odom 获取机器人当前位姿，返回 (x, y, yaw)
    如果超时或失败返回 None
    """
    try:
        # 等待一次 Odometry 消息
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /odom")
        return None

    # 提取位置
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 提取四元数并转换为 yaw
    q = msg.pose.pose.orientation
    import tf.transformations as tft
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

    rospy.loginfo(f"Robot pose from /odom → x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f}")
    return x, y, yaw

def is_robot_near_goal(goal_x, goal_y, r, timeout=5.0):
    """
    判断机器人当前 /odom 是否在目标 r 半径内
    """
    try:
        msg = rospy.wait_for_message("/odom", Odometry, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Waiting for /odom timed out")
        return False

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    dist = math.hypot(goal_x - x, goal_y - y)
    rospy.loginfo(f"Robot pos ({x:.3f},{y:.3f}) target ({goal_x:.3f},{goal_y:.3f}) dist {dist:.3f}")
    return dist < r


def set_robot_position(x, y, yaw, model_name="turtlebot3", reference_frame="world", timeout=10.0):
    """
    使用 /gazebo/set_model_state 将机器人瞬移到指定位姿
    """
    try:
        rospy.wait_for_service('/gazebo/set_model_state', timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("/gazebo/set_model_state service not available (timeout)")
        return False

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        ms = ModelState()
        ms.model_name = model_name
        ms.reference_frame = reference_frame
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        ms.pose = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
        # clear velocities
        ms.twist.linear.x = 0.0
        ms.twist.linear.y = 0.0
        ms.twist.linear.z = 0.0
        ms.twist.angular.x = 0.0
        ms.twist.angular.y = 0.0
        ms.twist.angular.z = 0.0

        resp = set_state(ms)
        # Some gazebo versions return bool in resp.success / resp.status_message
        try:
            success = resp.success
        except Exception:
            success = True  # 若响应中无 success 字段， assume success if no exception
        if success:
            rospy.loginfo(f"SetModelState succeeded for {model_name}")
            return True
        else:
            rospy.logwarn(f"SetModelState returned failure: {resp}")
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call /gazebo/set_model_state failed: {e}")
        return False


def publish_initial_pose(x, y, yaw, initial_pose_pub):
    """
    发布 /initialpose (PoseWithCovarianceStamped) 给 AMCL 或定位节点
    """
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    initial_pose.pose.pose = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
    # 简单 covariance（对角小值表明高置信度）
    cov = [0.0] * 36
    cov[0] = 0.05
    cov[7] = 0.05
    cov[35] = 0.05
    initial_pose.pose.covariance = cov

    # 发布多次以确保定位节点接收到
    for _ in range(3):
        initial_pose_pub.publish(initial_pose)
        rospy.sleep(0.1)
    rospy.loginfo(f"Published initial pose to ({x},{y},{yaw})")


def fallback_send_nav_goal(x, y, yaw, frame_id="map"):
    """
    如果外部 send_nav_goal 不可用，使用 /move_base_simple/goal 发布 PoseStamped
    """
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    # wait for publisher connections
    timeout = rospy.Time.now() + rospy.Duration(5.0)
    while pub.get_num_connections() == 0 and rospy.Time.now() < timeout:
        rospy.sleep(0.1)
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = rospy.Time.now()
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    goal.pose = Pose(Point(x, y, 0.0), Quaternion(q[0], q[1], q[2], q[3]))
    pub.publish(goal)
    rospy.loginfo(f"Published fallback nav goal ({x},{y},{yaw}) to /move_base_simple/goal")


def main_loop(start_x, start_y, start_yaw, final_goal_x, final_goal_y, model_name):
    rospy.loginfo("Entering main loop")
    # 创建 initial pose publisher（用于 AMCL）
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)  # give time for connections

    # 参数化：重试次数、半径等
    radius = rospy.get_param('~goal_radius', 0.5)
    max_replans = rospy.get_param('~max_replans', 30)

    # 先设置机器人初始位姿（gazebo）
    ok = set_robot_position(start_x, start_y, start_yaw, model_name=model_name)
    if not ok:
        rospy.logwarn("Initial set_model_state failed; continuing but position may be incorrect")

    # 通知定位系统初始位姿
    publish_initial_pose(start_x, start_y, start_yaw, initial_pose_pub)

    rate = rospy.Rate(2.5)  # 2.5 Hz loop (~0.4s per iter)
    replans = 0

    # 检查 send_nav_goal 是否可用
    use_external_send = True
    try:
        # 如果 send_nav_goal 是个可调用对象，优先使用
        if not callable(send_nav_goal):
            use_external_send = False
    except Exception:
        use_external_send = False

    while not rospy.is_shutdown():
        try:
            # 生成并获取子目标
            pose = get_robot_pose()
            if pose:
               cur_x, cur_y, yaw = pose
               print(f"Current robot pose: x={cur_x}, y={cur_y}, yaw={yaw}")
            x_sub, y_sub = pipeline_generate_subgoal(cur_x,cur_y,final_goal_x, final_goal_y)
            rospy.loginfo(f"Subgoal from pipeline: ({x_sub:.2f},{y_sub:.2f})")

            # 发送子目标到导航
            if use_external_send:
                try:
                    send_nav_goal(x_sub, y_sub, 0.0)  # 假定 send_nav_goal(x,y,yaw)
                    rospy.loginfo("Used external send_nav_goal to send goal")
                except Exception as e:
                    rospy.logwarn(f"External send_nav_goal failed: {e}; falling back")
                    fallback_send_nav_goal(x_sub, y_sub, 0.0)
            else:
                fallback_send_nav_goal(x_sub, y_sub, 0.0)

            replans += 1

            # 等一段时间并检测机器人是否到达最终目标附近
            # 这里我们检测距离 final_goal_x/final_goal_y（最终目标），而不是子目标
            # 每次循环等待若干秒做判断
            for _ in range(5):  # 检测若干次以防偶发消息延迟
                if rospy.is_shutdown():
                    break
                if is_robot_near_goal(final_goal_x, final_goal_y, radius, timeout=2.0):
                    rospy.loginfo("Robot near final goal; resetting for next episode")
                    # 复位机器人到 start 并发布 initialpose
                    set_robot_position(start_x, start_y, start_yaw, model_name=model_name)
                    publish_initial_pose(start_x, start_y, start_yaw, initial_pose_pub)
                    replans = 0
                    break
                rospy.sleep(0.5)

            if replans > max_replans:
                rospy.logwarn("Too many replans without reaching final goal; resetting robot and retrying")
                set_robot_position(start_x, start_y, start_yaw, model_name=model_name)
                publish_initial_pose(start_x, start_y, start_yaw, initial_pose_pub)
                replans = 0

            rate.sleep()
        except Exception as e:
            rospy.logerr(f"Exception in main loop: {e}")
            # 遇到错误短暂等待后继续
            rospy.sleep(1.0)


if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node('use_merge_example', anonymous=True)



    # 从参数或环境读取一些值（可通过 rosparam/launch 覆盖）
    model_name = rospy.get_param('~model_name', 'turtlebot3')
    # 默认 start/goal（如果你想要由外部传入，可在 roslaunch 时用 args 覆盖）
    start_x = float(rospy.get_param('~start_x', -3.71))
    start_y = float(rospy.get_param('~start_y', 9.21))
    start_yaw = float(rospy.get_param('~start_yaw', 0.0))
    final_goal_x = float(rospy.get_param('~final_goal_x', 1.73))
    final_goal_y = float(rospy.get_param('~final_goal_y', -9.57))

    rospy.loginfo(f"Node started. model_name={model_name}, start=({start_x},{start_y}), goal=({final_goal_x},{final_goal_y})")

    try:
        main_loop(start_x, start_y, start_yaw, final_goal_x, final_goal_y, model_name)
    except rospy.ROSInterruptException:
        pass
